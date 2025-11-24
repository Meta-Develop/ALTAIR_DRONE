from setuptools import setup
import os
from glob import glob

package_name = 'system_monitor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='System Monitor for ALTAIR Hexacopter',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'watchdog = system_monitor.watchdog:main',
        ],
    },
)