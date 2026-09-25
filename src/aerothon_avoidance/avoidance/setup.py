from setuptools import find_packages, setup

package_name = 'avoidance'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/costmap_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Rotor FPV',
    maintainer_email='sarthakkhubchandanik@gmail.com',
    description='Reactive corridor-centering + obstacle-braking velocity controller.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'velocity_controller = avoidance.velocity_controller:main',
        ],
    },
)
