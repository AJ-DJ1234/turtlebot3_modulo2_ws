from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_poi_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install the package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include package.xml in the share directory
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','PyYAML'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Package for navigating to Points of Interest',
    license='License Declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'poi_manager = turtlebot3_poi_navigation.poi_manager:main',
        ],
    },
)
