from glob import glob
from os import path

from setuptools import find_packages, setup

package_name = "robis_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (path.join("share", package_name, "config"), glob("config/*.yaml")),
        (
            path.join("share", package_name, "launch"),
            glob("launch/*launch.[pxy][yma]*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Miguel Grigorio",
    maintainer_email="miguelgrigorio@todo.todo",
    description="Camera and lightweight vision behaviors for the RobIS D2X platform.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "usb_camera_node = robis_vision.usb_camera_node:main",
            "ball_follower = robis_vision.ball_follower:main",
        ],
    },
)
