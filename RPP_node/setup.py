from setuptools import setup
import os
from glob import glob
package_name = 'RPP_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Añade estas líneas para instalar los archivos de launch, config y waypoints
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'waypoints'), glob('waypoints/*.csv')),
  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='crespo',
    maintainer_email='crespo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rpp_node_executable = RPP_node.rpp_node:main',
        ],
    },
)
