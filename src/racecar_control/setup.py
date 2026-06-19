from setuptools import setup
from glob import glob
import os

package_name = 'racecar_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='车辆控制：Pure Pursuit 横向 + PID 纵向',
    license='MIT',
    entry_points={
        'console_scripts': [
            'main_controller_node = racecar_control.main_controller_node:main',
        ],
    },
)
