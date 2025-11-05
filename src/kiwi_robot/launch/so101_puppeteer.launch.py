from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Serial port for SO101 puppeteer arm'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Joint state publish rate (Hz)'
        ),
        DeclareLaunchArgument(
            'arm_id',
            default_value='puppeteer_arm',
            description='Arm ID for namespace'
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='',
            description='Path to calibration JSON file'
        ),
        
        Node(
            package='kiwi_robot',
            executable='so101_puppeteer',
            name='so101_puppeteer',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'arm_id': LaunchConfiguration('arm_id'),
                'calibration_file': LaunchConfiguration('calibration_file'),
            }],
            output='screen'
        ),
    ])

