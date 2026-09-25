from setuptools import find_packages, setup

package_name = 'gcs_aggregator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='Team Rotor FPV',
    maintainer_email='sarthakkhubchandanik@gmail.com',
    description='ROS2 -> WebSocket JSON aggregator + command relay for the Tauri GCS.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'aggregator = gcs_aggregator.aggregator:main',
            'readiness = gcs_aggregator.readiness_node:main',
        ],
    },
)
