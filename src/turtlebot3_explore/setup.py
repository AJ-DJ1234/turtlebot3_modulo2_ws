# setup.py

from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_explore'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu_email@example.com',
    description='Nodo de ROS2 para la exploración autónoma de un robot móvil utilizando Nav2 y SLAM Toolbox.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explore = turtlebot3_explore.explore:main',
        ],
    },
)
