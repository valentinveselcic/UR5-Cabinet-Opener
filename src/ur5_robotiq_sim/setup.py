from setuptools import find_packages, setup

package_name = 'ur5_robotiq_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ur5_robotiq_view.launch.py', 'launch/ur5_sim_control.launch.py', 'launch/ur5_robot_sim_moveit.launch.py', 'launch/ur5_rsp.launch.py', 'launch/lv3_spawner.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/ur5_robotiq_3f.xacro', 'urdf/ur5_robotiq_3f_macro.xacro', 'urdf/ur_gz.xacro', 'urdf/ur_real.xacro', ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='simundicv@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'open_cabinet = ur5_robotiq_sim.open_cabinet:main',
        ],
    },
)
