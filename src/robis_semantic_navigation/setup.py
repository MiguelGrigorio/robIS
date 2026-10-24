from glob import glob
from os import path

from setuptools import find_packages, setup

package_name = "robis_semantic_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (path.join("share", package_name, "config"), glob("config/*.yaml")),
        (path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest", "flake8"]},
    zip_safe=True,
    maintainer="Miguel Grigorio",
    maintainer_email="miguelgrigorio@todo.todo",
    description="Persistent semantic navigation for the robIS robot.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "object_localizer = robis_semantic_navigation.object_localizer:main",
        "semantic_navigation_server = robis_semantic_navigation.server:main",
        "goto = robis_semantic_navigation.goto:main",
        "stop = robis_semantic_navigation.stop:main",
        "localization_evaluator = robis_semantic_navigation.localization_evaluator:main",
        "localization_report = robis_semantic_navigation.localization_report:main",
        "localization_ablation = robis_semantic_navigation.localization_ablation:main",
        "fixed_camera_replay = robis_semantic_navigation.fixed_camera_replay:main",
        "fixed_camera_stream = robis_semantic_navigation.fixed_camera_stream:main",
    ]},
)
