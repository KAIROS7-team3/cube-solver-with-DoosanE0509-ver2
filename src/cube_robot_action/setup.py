from setuptools import setup
import os
from glob import glob

package_name = 'cube_robot_action'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'action'),
            glob('action/*.action')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'robot_action_server_node = cube_robot_action.robot_action_server_node:main',
        ],
    },
)
