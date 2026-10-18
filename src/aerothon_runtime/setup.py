from setuptools import find_packages, setup

setup(name='aerothon_runtime', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/aerothon_runtime']),
                  ('share/aerothon_runtime', ['package.xml'])],
      install_requires=['setuptools', 'aiohttp'],
      entry_points={'console_scripts': [
          'web_host = aerothon_runtime.web:main',
          'execution_node = aerothon_runtime.execution_node:main',
          'remote_devices = aerothon_runtime.remote_devices:main',
          'laptop_gateway = aerothon_runtime.gateway:main']})
