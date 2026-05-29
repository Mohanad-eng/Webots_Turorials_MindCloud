import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('webots_pkg_sim')

    urdf_file = os.path.join(pkg_share, 'urdf', 'gazebo_file.urdf')
    controllers_file = os.path.join(pkg_share, 'config', 'controllers.yaml')

    # FIX: Gazebo resource path
    pkg_parent = os.path.dirname(pkg_share)
    set_gz_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=pkg_parent
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Robot state publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_file).read(),
            'use_sim_time': True
        }]
    )

    # Spawn robot
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'new_rover',
            '-z', '0.5'
        ],
        output='screen'
    )

    # ros2_control node (FIXED)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'use_sim_time': True},
            urdf_file,
            controllers_file
        ],
        output='screen'
    )

    # Delay controller spawners (IMPORTANT FIX)
    steering_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['steering_controller'],
                output='screen'
            )
        ]
    )

    wheel_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['wheel_controller'],
                output='screen'
            )
        ]
    )

    # Swerve node
    swerve_node = Node(
        package='webots_pkg_sim',
        executable='swerve_g',
        output='screen'
    )

    return LaunchDescription([
        set_gz_path,
        gazebo,
        rsp,
        spawn,
        control_node,
        steering_spawner,
        wheel_spawner,
        swerve_node
    ])