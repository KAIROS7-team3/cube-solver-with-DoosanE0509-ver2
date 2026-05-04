from setuptools import find_packages, setup
import os
from glob import glob

package_name = "cube_perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "google-genai",
        "python-dotenv",
    ],
    package_data={"cube_perception": ["prompts/*.txt"]},
    zip_safe=True,
    maintainer="junsoo",
    maintainer_email="seojunsoo312@gmail.com",
    description="Rubik cube perception (ROS 2)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "vla_detection_node = cube_perception.vla_detection_node:main",
            "color_extraction_node = cube_perception.color_extraction_node:main",
            "debug_viewer_node = cube_perception.debug_viewer_node:main",
            "service_tester_gui_node = cube_perception.service_tester_gui_node:main",
        ],
    },
)
