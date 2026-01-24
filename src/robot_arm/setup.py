from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- THIS IS THE CRITICAL PART ---
        # This line tells colcon to install all launch files from the 'launch' directory.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A package for real-time robotic arm control via motion capture.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'angle_sub = robot_arm.forward_kinematics:main',
        'IK_service = robot_arm.inverse_kinematics_node:main',
        'move_client = robot_arm.pick_up_client:main',
        'vel_kin_service = robot_arm.velocity_kinematics:main',
        'incremental_control = robot_arm.incremental_joint_control:main',
        'inverse_kinematics_node = robot_arm.inverse_kinematics_node:main',
        'websocket_bridge_node = robot_arm.websocket_bridge_node:main',
        'realtime_control_node = robot_arm.realtime_control_node:main',
        'basic_robot_control = robot_arm.basic_robot_control:main',       
        ],
    },
)
