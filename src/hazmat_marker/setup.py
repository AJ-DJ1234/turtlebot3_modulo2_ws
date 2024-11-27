from setuptools import setup

package_name = 'hazmat_marker'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'tf-transformations'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Package to mark hazmat locations on map in RViz based on detections',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hazmat_marker_al = hazmat_marker.hazmat_marker_al:main',
            'hazmat_marker = hazmat_marker.hazmat_marker:main',
            'hazmat_marker_aj = hazmat_marker.hazmat_marker_aj:main',
        ],
    },
)
