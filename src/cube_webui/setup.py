from glob import glob

from setuptools import find_packages, setup

package_name = "cube_webui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        (
            "share/" + package_name + "/static",
            glob("cube_webui/static/*"),
        ),
    ],
    include_package_data=True,
    package_data={package_name: ["static/*"]},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hhyukjin",
    maintainer_email="hhyukjin@todo.todo",
    description="Web UI for the cube solver orchestrator.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "webui_server = cube_webui.server:main",
        ],
    },
)
