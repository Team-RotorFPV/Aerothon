from setuptools import setup

package_name = 'perception_overlay'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Rotor FPV',
    maintainer_email='teamrotorfpv@vit.ac.in',
    description="One camera feed with every detector's findings drawn on it.",
    license='MIT',
    entry_points={
        'console_scripts': [
            'overlay_node = perception_overlay.overlay_node:main',
        ],
    },
)
