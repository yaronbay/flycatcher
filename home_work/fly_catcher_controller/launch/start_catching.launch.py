import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('fly_catcher_controller')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'fly_catcher.rviz')

    return LaunchDescription([
        Node(package="tf2_ros", executable="static_transform_publisher", 
             arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"]),
        
        Node(package="fly_catcher_simulator", executable="fly", name="fly_sim"),
        Node(package="fly_catcher_simulator", executable="drone", name="drone_sim"),
        
        Node(package="fly_catcher_controller", executable="pursuit_node", output="screen"),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])