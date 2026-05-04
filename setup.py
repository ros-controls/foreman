from glob import glob
from setuptools import setup, find_packages

package_name = 'foreman'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.xml')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'controller_manager_msgs',
        'lifecycle_msgs',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='Nikola Banovic',
    maintainer_email='nibanovic@gmail.com',
    description='Bringup component manager for ros2 control',
    license='Copyright (c) 2026, b-robotized GmbH.',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'foreman_node=foreman.node:main',
        ],
    },
)
