from setuptools import find_packages, setup

package_name = 'cube_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', [
            'srv/DetectCubePose.srv',
            'srv/ExtractFace.srv',
        ]),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'google-genai',
        'python-dotenv',
    ],
    package_data={
        'cube_perception': ['prompts/*.txt'],
    },
    zip_safe=True,
    maintainer='junsoo',
    maintainer_email='seojunsoo312@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vla_detection_node = cube_perception.vla_detection_node:main',
            'color_extraction_node = cube_perception.color_extraction_node:main',
        ],
    },
)
