import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('fly_catcher_controller')
    
    # Path to your specific rviz file
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'fly_catcher.rviz')

    return LaunchDescription([
        # Static TF to keep map and base_link aligned
        Node(package='tf2_ros', executable='static_transform_publisher', 
             arguments=['0','0','0','0','0','0','map','base_link']),
        
        # Simulators
        Node(package='fly_catcher_simulator', executable='fly', name='fly_sim'),
        Node(package='fly_catcher_simulator', executable='drone', name='drone_sim'),
        
        # The Circular Pursuit Node
        Node(package='fly_catcher_controller', executable='circular_pursuit_node', 
             output='screen'),
        
        # RViz with the correct config file
        Node(package='rviz2', executable='rviz2', 
             arguments=['-d', rviz_config_path])
    ])