from setuptools import find_packages, setup

package_name = 'camera_ctrl'

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
    description='Camera pointing as a commanded, read-back-confirmed state.',
    license='MIT',
    entry_points={'console_scripts': [
        'camera_ctrl_node = camera_ctrl.camera_ctrl_node:main']},
)
