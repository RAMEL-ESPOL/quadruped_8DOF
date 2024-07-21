import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cinematic_control'

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
    maintainer='julio',
    maintainer_email='jdender@espol.edu.ec',
    description='Paquete que contiene los nodos de la cinematica',
    license='Apache LIcense 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_hip1_cmd = cinematic_control.velocity_hip1_cmd:main',
            'velocity_hip2_cmd = cinematic_control.velocity_hip2_cmd:main',
            'velocity_hip3_cmd = cinematic_control.velocity_hip3_cmd:main',
            'velocity_hip4_cmd = cinematic_control.velocity_hip4_cmd:main',
        ],
    },
)
