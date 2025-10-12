from setuptools import setup
import os
from glob import glob
package_name = 'autonomy_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'autonomy_manager'), glob('autonmoy_manager/*.py')),
        (os.path.join('share', package_name, 'src'),   glob('src/*.py')),
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
            'autonomy_manager_node = autonomy_manager.autonomy_manager_node:main',
        ],
    },
)
