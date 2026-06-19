from setuptools import find_packages, setup

package_name = 'percep_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 文件要安装到 share 目录，ros2 launch 才能找到
        ('share/' + package_name + '/launch', ['launch/perception.launch.py',
                                                'launch/ekf.launch.py',
                                                'launch/slam.launch.py']),
        # config 文件 likewise
        ('share/' + package_name + '/config', ['config/ekf.yaml',
                                                'config/slam_toolbox.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='易达良',
    maintainer_email='202530462184@mail.scut.edu.cn',
    description='感知组位姿估计包：融合传感器数据，估计车辆位姿，建图并输出给规划控制',
    license='SCUT FSAC',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 节点名 = 包.模块:main
            'vehicle_state_publisher = percep_localization.vehicle_state_publisher:main',
            'ekf_fusion_node = percep_localization.ekf_fusion_node:main',
            'cone_map_node = percep_localization.cone_map_node:main',
        ],
    },
)
