from setuptools import setup, find_packages
import os
from glob import glob

package_name = "odrive_cam_logger"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="raianbsbrl",
    maintainer_email="raianbsbrl@todo.todo",
    description="Camera + ODrive data collection (no Sensapex). Logs encoder counts and saves frames.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odrive_driver_node = odrive_cam_logger.odrive_driver_node:main",
            "camera_node = odrive_cam_logger.camera_node:main",
            "logger_node = odrive_cam_logger.logger_node:main",
            "gui_node = odrive_cam_logger.gui_node:main",
        ],
    },
)
