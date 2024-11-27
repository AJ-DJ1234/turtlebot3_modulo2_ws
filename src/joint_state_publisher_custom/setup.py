from setuptools import setup

package_name = 'joint_state_publisher_custom'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Custom Joint State Publisher for Parallel Mechanism Robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = joint_state_publisher_custom.joint_state_publisher:main',
        ],
    },
)
