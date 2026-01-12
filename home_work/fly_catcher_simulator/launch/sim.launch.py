from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # map -> base_link (identity). RViz will be happy.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_base_link_tf",
            arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"],
            output="screen",
        ),

        Node(
            package="fly_catcher_simulator",
            executable="fly",
            name="fly_sim",
            output="screen",
        ),
        Node(
            package="fly_catcher_simulator",
            executable="drone",
            name="drone_sim",
            output="screen",
        ),
    ])
