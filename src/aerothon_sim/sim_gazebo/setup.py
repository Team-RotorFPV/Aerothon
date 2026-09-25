import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sim_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'materials'), glob('materials/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Rotor FPV',
    maintainer_email='sarthakkhubchandanik@gmail.com',
    description='Gazebo Harmonic Mission 2 world + bridge.',
    license='MIT',
    entry_points={'console_scripts': [
        'odom_tf = sim_gazebo.odom_tf:main',
    ]},
)
