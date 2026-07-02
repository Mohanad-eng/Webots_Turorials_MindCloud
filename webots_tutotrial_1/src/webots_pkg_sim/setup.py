from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'webots_pkg_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*')
        ),
        (
            os.path.join('share', package_name, 'meshes'),
            glob('meshes/*')
        ),
        (
            os.path.join('share', package_name, 'meshes_1'),
            glob('meshes_1/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohanad',
    maintainer_email='eng.mohanadalexu@gmail.com',
    description='Webots ROS2 simulation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webots_driver = webots_pkg_sim.webots_driver:main',
            'rover_driver = webots_pkg_sim.rover_driver:main',
            'costmap = webots_pkg_sim.costmap:main',
            'costmap_lifecycle = webots_pkg_sim.costmap_lifecyle:main',
            'vo_odom = webots_pkg_sim.vo_code:main',
            'swerve_g = webots_pkg_sim.swerve:main',
            'falak_driver = webots_pkg_sim.driver_falak:main',
        ],
    },
)