from setuptools import find_packages, setup

package_name = 'camara'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),  # Asegúrate de incluir package.xml
    ],
    install_requires=[
        'setuptools',
        'torch',
        'opencv-python',
        'numpy'
    ],
    zip_safe=True,
    maintainer='alonso',
    maintainer_email='alonso@todo.todo',
    description='Procesamiento de imágenes con detección HAZMAT',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'camara_client = camara.camara_client:main',
            'camara_server = camara.camara_server:main'
        ],
    },
)
