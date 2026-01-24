from setuptools import find_packages, setup
import os
from glob import glob

package_name = "omx_position_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Install launch files
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        # CRITICAL: Add the msg and srv directories to the install
        (os.path.join("share", package_name, "msg"), glob("msg/*.msg")),
        (os.path.join("share", package_name, "srv"), glob("srv/*.srv")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Harsh Chhajed",
    maintainer_email="harshchhajed30@gmail.com",
    description="A package for real-time robotic arm control via motion capture.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "inverse_kinematics_node = omx_position_control.inverse_kinematics_node:main",
            "websocket_bridge_node = omx_position_control.websocket_bridge_node:main",
            "websocket_bridge_node2 = omx_position_control.websocket_bridge_node2:main",
            "realtime_control_node = omx_position_control.realtime_control_node:main",
            "basic_robot_control = omx_position_control.basic_robot_control:main",
            "basic_robot_control2 = omx_position_control.basic_robot_control2:main",
            "forward_kinematics= omx_position_control.forward_kinematics:main",
            "rmse_evaluator = omx_position_control.rmse_evaluator:main",
            "trajectory_evaluator = omx_position_control.trajectory_evaluator:main",
            "master_evaluator_node = omx_position_control.master_evaluator_node:main",
        ],
    },
)
