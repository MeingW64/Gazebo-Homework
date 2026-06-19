from setuptools import setup

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='xiaoting',
    maintainer_email='BPJY@outlook.com',
    description='路径规划节点',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'path_planning_node = path_planning.path_planning_node:main',
        ],
    },
)
