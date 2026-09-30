from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    

    ur5_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur5_robotiq_sim'),
                'launch',
                'ur5_robot_sim_moveit.launch.py'
            ])
        ])
    )

    cabinet_urdf_path = 'src/cabinet.urdf'
    
    spawn_cabinet_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', cabinet_urdf_path,
            '-name', 'cabinet',
            '-allow_renaming', 'true',
            '-x', '0', 
            '-y', '0.15', 
            '-z', '0.998', 
            '-R', '0', 
            '-P', '0', 
            '-Y', '-1.57'
        ],
        output='screen'
    )

    delayed_spawn = TimerAction(period=5.0, actions=[spawn_cabinet_node])

    return LaunchDescription([
        ur5_sim_launch,
        delayed_spawn,
    ])