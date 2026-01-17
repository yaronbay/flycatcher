import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Setup paths
    pkg_name = 'fly_catcher_controller'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Path to the RViz config
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'fly_catcher.rviz')
    
    # Path to the NEW YAML parameter file
    # (Ensure this file exists at fly_catcher_controller/config/params.yaml)
    params_config_path = os.path.join(pkg_share, 'config', 'params.yaml')

    # 2. Static Transform Publisher (Map to Base Link)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )

    # 3. Fly Simulator Node
    fly_sim = Node(
        package='fly_catcher_simulator',
        executable='fly',
        name='fly_sim'
    )

    # 4. Drone Simulator Node
    drone_sim = Node(
        package='fly_catcher_simulator',
        executable='drone',
        name='drone_sim'
    )

    # 5. Velocity Pursuit Controller
    # We replaced the hard-coded dictionary with the path to your YAML file
    velocity_pursuit = Node(
        package=pkg_name,
        executable='velocity_pursuit_node',
        name='velocity_pursuit_node',  # Node name MUST match the top heading in YAML
        output='screen',
        parameters=[params_config_path] 
    )

    # 6. RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        static_tf,
        fly_sim,
        drone_sim,
        velocity_pursuit,
        rviz2
    ])