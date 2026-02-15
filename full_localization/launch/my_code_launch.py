from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    
    # Your localization node (publishes odom -> base_link TF)
    localization_node = Node(
        package='full_localization',  # Change to your package name
        executable='localization_node',
        name='localization_node',
        output='screen',
    )
    
    slam_node = Node(
        package='full_localization',  # Change to your package name
        executable='slam_node',
        name='slam_node',
        output='screen',
    )

    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(localization_node)       # Then localization
    ld.add_action(slam_node)       # Then SLAM

    return ld