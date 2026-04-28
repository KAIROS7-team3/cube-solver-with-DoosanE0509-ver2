from setuptools import find_packages, setup

package_name = "cube_perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
)
