from setuptools import find_packages, setup

package_name = 'interface_turtlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ajdj',
    maintainer_email='alanjoshua1976@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interface_bot = interface_turtlebot.interface_bot:main',
            'interface = interface_turtlebot.interface:main',
            'interface_aj = interface_turtlebot.interface_aj:main',
        ],
    },
)
