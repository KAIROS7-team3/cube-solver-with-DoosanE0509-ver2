from setuptools import find_packages, setup

package_name = 'rh_p12_rna_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cube_team',
    maintainer_email='cube@todo.com',
    description='ROS 2 Action 기반 RH-P12-RN(A) 그리퍼 제어 노드',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gripper_node = rh_p12_rna_controller.gripper_node:main',
            'gripper_service_node = rh_p12_rna_controller.gripper_service_node:main',
        ],
    },
)
