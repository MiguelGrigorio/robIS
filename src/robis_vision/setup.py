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
        ("share/" + package_name, ["README.md"]),
        (path.join("share", package_name, "config"), glob("config/*.yaml")),
        (
            path.join("share", package_name, "launch"),
            glob("launch/*launch.[pxy][yma]*"),
        ),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest", "flake8"]},
    zip_safe=True,
    maintainer="Miguel Grigorio",
    maintainer_email="miguelgrigorio@todo.todo",
    description="Camera and lightweight vision behaviors for the RobIS D2X platform.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "usb_camera_node = robis_vision.usb_camera_node:main",
            "image_preview = robis_vision.image_preview:main",
            "ball_follower = robis_vision.ball_follower:main",
            "open_vocab_detector = robis_vision.open_vocab_detector:main",
            "oracle_detector = robis_vision.oracle_detector:main",
            "depth_estimator = robis_vision.depth_estimator:main",
            "detector_benchmark = robis_vision.detector_benchmark:main",
            "detection_runtime_monitor = robis_vision.detection_runtime_monitor:main",
            "detector_evaluator = robis_vision.detector_evaluator:main",
            "detector_report = robis_vision.detector_report:main",
            "semantic_dataset_recorder = robis_vision.semantic_dataset_recorder:main",
            "semantic_to_yolo = robis_vision.semantic_to_yolo:main",
            "depth_benchmark = robis_vision.depth_benchmark:main",
            "yolo26_detector = robis_vision.yolo26_detector:main",
            "ollama_detector = robis_vision.ollama_detector:main",
        ],
    },
)
