from setuptools import find_packages, setup

package_name = 'mission_bt'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Rotor FPV',
    maintainer_email='sarthakkhubchandanik@gmail.com',
    description='py_trees Mission 2 orchestration with abort-from-anywhere guard.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_tree = mission_bt.mission_tree:main',
        ],
    },
)
