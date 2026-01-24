from setuptools import find_packages, setup

package_name = 'teleop_omx'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/websocket_robot_control.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'opencv-python', 
        'mediapipe',
        'matplotlib',
        'websocket-client',  # Add WebSocket client dependency
    ],
    zip_safe=True,
    maintainer='harshrocks',
    maintainer_email='harshchhajed30@gmail.com',
    description='Enhanced teleoperation nodes with WebSocket integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cv_controller_node = teleop_omx.cv_controller_node:main',
            'vel_calc = teleop_omx.vel_calc:main',
            'vel_kinematics = teleop_omx.vel_kinematics:main',
            'forward = teleop_omx.forward:main',
            'home_pos = teleop_omx.basic_robot_control:main',
            'omx_velocity_controller = teleop_omx.omx_velocity_controller:main',
            'websocket_bridge_node = teleop_omx.websocket_bridge_node:main',  # New
            'enhanced_velocity_controller = teleop_omx.enhanced_velocity_controller:main',  # New
            'enhanced_vel_kinematics = teleop_omx.enhanced_vel_kinematics:main',  # New
        ],
    },
)
