from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quadruped_vrep'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['quadruped_vrep', 'quadruped_vrep.*'],exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name), glob('launch/*.launch.py'))        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juan',
    maintainer_email='maranmen@espol.edu.ec',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],  # Dependencias para pruebas
    },
    entry_points={
        'console_scripts': [
            'communication=quadruped_vrep.communication:main'
        ],
    },
)
