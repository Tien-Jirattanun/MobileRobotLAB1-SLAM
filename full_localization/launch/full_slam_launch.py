from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Launch arguments
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value='mapper_params_base_link.yaml',
        description='Path to slam_toolbox parameters')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock')

    # Static TF: base_link -> base_scan
    # IMPORTANT: Adjust the z value (0.1) if your laser is at a different height
    static_tf_base_to_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'base_scan'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Your localization node (publishes odom -> base_link TF)
    localization_node = Node(
        package='full_localization',  # Change to your package name
        executable='localization_node',
        name='localization_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    slam_node = Node(
        package='full_localization',  # Change to your package name
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # slam_toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_slam_params)
    ld.add_action(declare_use_sim_time)
    
    # Add nodes
    ld.add_action(static_tf_base_to_scan)  # TF first
    ld.add_action(localization_node)       # Then localization
    ld.add_action(slam_node)       # Then SLAM
    ld.add_action(slam_toolbox_node)       # Then SLAM

    return ld