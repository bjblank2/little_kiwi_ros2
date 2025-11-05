from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Serial port for SO101 arm'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',
            description='Position publish rate (Hz) - start slow for debugging'
        ),
        DeclareLaunchArgument(
            'use_normalized',
            default_value='false',
            description='Use normalized values (false = raw values)'
        ),
        
        Node(
            package='kiwi_robot',
            executable='so101_test_node',
            name='so101_test_node',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'use_normalized': LaunchConfiguration('use_normalized'),
            }],
            output='screen'
        ),
    ])



