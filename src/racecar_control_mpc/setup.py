from setuptools import setup
from glob import glob
import os

package_name = 'racecar_control_mpc'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='MPC 模型预测控制',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mpc_node = racecar_control_mpc.mpc_node:main',
        ],
    },
)
