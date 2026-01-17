import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Paths
    pkg_name = 'fly_catcher_controller'
    pkg_share = get_package_share_directory(pkg_name)
    params_config_path = os.path.join(pkg_share, 'config', 'params.yaml')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'fly_catcher.rviz')

    # --- 1. DEFINE ALL TUNABLE ARGUMENTS ---
    # We create a list of tuples: (arg_name, default_value, description)
    tunable_params = [
        # Controller Gains
        ('kp_dist', '50.0', 'Proportional gain for distance to target'),
        ('k_ff_vel', '50.0', 'Feed-forward gain for velocity matching'),
        ('look_ahead_ratio', '0.3', 'Ratio for predictive interception'),
        ('min_look_ahead', '0.1', 'Minimum prediction time in seconds'),
        ('max_look_ahead', '0.9', 'Maximum prediction time in seconds'),
        
        # Kalman Filter
        ('q_noise', '0.01', 'Process noise covariance'),
        ('r_noise', '0.001', 'Measurement noise covariance'),
        
        # Logic & Constraints
        ('catch_threshold', '0.2', 'Distance in meters to trigger catch'),
        ('max_thrust', '100.0', 'Maximum drone thrust'),
        ('min_thrust', '15.0', 'Minimum drone thrust'),
        
        # Fly Simulation Overrides
        ('fly_x', '0.0', 'Initial fly X position'),
        ('fly_y', '10.0', 'Initial fly Y position'),
        ('fly_z', '3.0', 'Initial fly Z position'),
        ('fly_speed', '1.0', 'Maximum fly speed')
    ]

    # --- 2. PREPARE LAUNCH CONFIGURATIONS ---
    launch_args = []
    controller_overrides = {}
    
    for name, default, help_text in tunable_params:
        # Declare the argument for the CLI
        launch_args.append(DeclareLaunchArgument(name, default_value=default, description=help_text))
        
        # If it's a controller param, add to the controller override dictionary
        if name not in ['fly_x', 'fly_y', 'fly_z', 'fly_speed']:
            controller_overrides[name] = LaunchConfiguration(name)

    # --- 3. DEFINE NODES ---

    # Static Transform: map -> base_link
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )

    # Fly Simulator
    fly_sim = Node(
        package='fly_catcher_simulator',
        executable='fly',
        name='fly_sim',
        parameters=[
            params_config_path,
            {
                'x0': LaunchConfiguration('fly_x'),
                'y0': LaunchConfiguration('fly_y'),
                'z0': LaunchConfiguration('fly_z'),
                'speed_max': LaunchConfiguration('fly_speed')
            }
        ]
    )

    # Drone Simulator
    drone_sim = Node(
        package='fly_catcher_simulator',
        executable='drone',
        name='drone_sim'
    )

    # Velocity Pursuit Controller (The Brain)
    pursuit_node = Node(
        package=pkg_name,
        executable='velocity_pursuit_node',
        name='velocity_pursuit_node',
        output='screen',
        parameters=[
            params_config_path,   # Default values from YAML
            controller_overrides  # Overridden by CLI arguments
        ]
    )

    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path]
    )

    # --- 4. RETURN DESCRIPTION ---
    return LaunchDescription(launch_args + [
        static_tf,
        fly_sim,
        drone_sim,
        pursuit_node,
        rviz2
    ])