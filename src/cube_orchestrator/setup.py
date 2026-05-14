from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cube_orchestrator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hhyukjin',
    maintainer_email='hhyukjin@todo.todo',
    description='FSM orchestrator for the cube solver system',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'master_orchestrator_node = cube_orchestrator.master_orchestrator_node:main',
        ],
    },
)
