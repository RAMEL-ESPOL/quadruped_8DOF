from setuptools import setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'quadruped_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maranmen',
    maintainer_email='maranmen@espol.edu.ec',
    description='ws quadruped',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'continuous_lifter = quadruped_model.continuous_lifter:main',
            'move_robot = quadruped_model.move_robot:main',
            'vrep_communication= quadruped_model.vrep_communication:main', 
        ], 
      },
)