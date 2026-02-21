from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uav_ugv_teaming'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roger',
    maintainer_email='fowler.ro@northeastern.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_odom_publisher = uav_ugv_teaming.simple_odom_publisher:main',
            'waypoint_navigator = uav_ugv_teaming.waypoint_navigator:main',
            'send_waypoint = uav_ugv_teaming.send_waypoint:main',
            'load_plan = uav_ugv_teaming.load_plan:main',
            'plan_manager = uav_ugv_teaming.plan_manager:main',
            'mesh_scaler = uav_ugv_teaming.mesh_scaler:main',
            'collision_force_injector = uav_ugv_teaming.collision_force_injector:main',
            'state_override = uav_ugv_teaming.state_override:main',
            'wrench_collector = uav_ugv_teaming.wrench_collector:main',
            'landing_commander = uav_ugv_teaming.landing_commander:main',
        ],
    },
)
