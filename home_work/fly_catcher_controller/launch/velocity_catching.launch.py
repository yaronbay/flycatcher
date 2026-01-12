import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get directories
    pkg_name = 'fly_catcher_controller'
    pkg_share = get_package_share_directory(pkg_name)
    
    # 2. Path to your existing RViz file
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'fly_catcher.rviz')

    # 3. Static Transform Publisher (Map to Base Link)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )

    # 4. Fly Simulator Node
    fly_sim = Node(
        package='fly_catcher_simulator',
        executable='fly',
        name='fly_sim'
    )

    # 5. Drone Simulator Node
    drone_sim = Node(
        package='fly_catcher_simulator',
        executable='drone',
        name='drone_sim'
    )

    # 6. Velocity Pursuit Controller (The new KF node)
    velocity_pursuit = Node(
        package=pkg_name,
        executable='velocity_pursuit_node',
        output='screen',
        parameters=[{'catch_threshold': 0.2}]
    )

    # 7. RViz2 with your specific config file
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