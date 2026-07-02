"""
falak_launch.py  -  ROS2 launch file for the falak rover in Webots R2025a.

Usage:
  ros2 launch webots_pkg_sim falak.launch.py

Mesh path strategy
------------------
Webots resolves Mesh { url "..." } relative to the .wbt file location.
After `colcon build`, the world lands at:
  install/webots_pkg_sim/share/webots_pkg_sim/worlds/falak.wbt
and meshes at:
  install/webots_pkg_sim/share/webots_pkg_sim/meshes/*.STL

"../meshes/" from worlds/ resolves correctly in that layout, BUT only if
Webots is launched from the install tree and the relative path is right.
The safest approach (used by the working new_rover world) is absolute paths.

At launch time we:
  1. Read the template falak.wbt
  2. Replace "../meshes/" with the absolute installed meshes directory
  3. Write to /tmp/falak_runtime.wbt
  4. Pass that file to WebotsLauncher

robot_description for WebotsController must be the URDF file path (not XML).
robot_state_publisher receives the XML string via Python dict (safe).
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    pkg_share = get_package_share_directory("webots_pkg_sim")

    world_template = os.path.join(pkg_share, "worlds", "falak.wbt")
    urdf_path      = os.path.join(pkg_share, "urdf",   "falak.urdf")
    meshes_dir     = os.path.join(pkg_share, "meshes_1")  # absolute path

    # ------------------------------------------------------------------
    # 1. Patch the world file: replace relative mesh paths with absolute
    # ------------------------------------------------------------------
    with open(world_template, "r") as f:
        world_content = f.read()

    # Replace every occurrence of  "../meshes/  with the absolute path.
    # The trailing slash is kept; filenames are unchanged.
    world_content = world_content.replace(
        '"../meshes_1/', '"' + meshes_dir + "/"
    )

    # Write patched world to a temp file that persists for the session
    tmp_world = tempfile.NamedTemporaryFile(
        mode="w", suffix="_falak_runtime.wbt", delete=False
    )
    tmp_world.write(world_content)
    tmp_world.flush()
    tmp_world.close()
    runtime_world_path = tmp_world.name

    # ------------------------------------------------------------------
    # 2. Read URDF for robot_state_publisher
    # ------------------------------------------------------------------
    with open(urdf_path, "r") as f:
        robot_description_xml = f.read()

    # ------------------------------------------------------------------
    # 3. Webots simulator  (uses patched world with absolute mesh paths)
    # ------------------------------------------------------------------
    webots = WebotsLauncher(
        world=runtime_world_path,
    )

    # ------------------------------------------------------------------
    # 4. Extern controller
    #    robot_description = URDF FILE PATH (not XML string)
    #    webots_ros2_driver reads the file to find <webots><plugin>
    # ------------------------------------------------------------------
    driver = WebotsController(
        robot_name="falak",
        parameters=[
            {
                "robot_description": urdf_path,
                "use_sim_time": True,
            }
        ],
    )

    # ------------------------------------------------------------------
    # 5. robot_state_publisher  (needs XML string)
    # ------------------------------------------------------------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_xml,
                "use_sim_time": True,
            }
        ],
    )

    # ------------------------------------------------------------------
    # 6. Shut down everything when Webots exits
    # ------------------------------------------------------------------
    shutdown_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=webots,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription(
        [
            webots,
            driver,
            rsp,
            shutdown_handler,
        ]
    )