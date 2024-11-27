from setuptools import setup

package_name = 'motor_position_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gerardo',
    maintainer_email='gerardo@example.com',
    description='A package for controlling motor positions.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_position_controller = motor_position_controller_node:main',
        ],
    },
)
