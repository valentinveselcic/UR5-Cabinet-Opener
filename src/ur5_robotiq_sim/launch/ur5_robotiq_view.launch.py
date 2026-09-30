from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    pkg_path = get_package_share_directory('ur5_robotiq_sim')
    xacro_file = os.path.join(pkg_path, 'urdf', 'ur5_robotiq_3f.xacro')

    # Process Xacro
    robot_description = xacro.process_file(
    xacro_file,
    mappings={
        "use_sim": "true",
        "robot_ip": "192.168.0.10"
    }
).toxml()

    return LaunchDescription([

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        )
    ])
