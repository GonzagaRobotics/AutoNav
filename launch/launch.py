from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    static_dir = LaunchConfiguration('static_dir')
    site_name = LaunchConfiguration('site_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'static_dir',
            description='Path to the static files directory'
        ),
        DeclareLaunchArgument(
            'site_name',
            description='Name of the site the rover is in'
        ),
        # Node(
        #     package='auto_nav',
        #     executable='auto_nav',
        # ),
        Node(
            package='pathfinder',
            executable='pathfinder',
            parameters=[
                {'static_dir': static_dir},
                {'site_name': site_name}
            ]
        )
    ])
