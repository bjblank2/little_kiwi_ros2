from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM1',
            description='Serial port for SO101 puppet arm'
        ),
        DeclareLaunchArgument(
            'arm_id',
            default_value='puppet_arm',
            description='Arm ID for namespace'
        ),
        DeclareLaunchArgument(
            'puppeteer_arm_id',
            default_value='puppeteer_arm',
            description='Puppeteer arm ID to follow'
        ),
        DeclareLaunchArgument(
            'max_relative_target',
            default_value='20.0',
            description='Maximum relative target movement (degrees)'
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='',
            description='Path to calibration JSON file'
        ),
        
        Node(
            package='kiwi_robot',
            executable='so101_puppet',
            name='so101_puppet',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'arm_id': LaunchConfiguration('arm_id'),
                'puppeteer_arm_id': LaunchConfiguration('puppeteer_arm_id'),
                'max_relative_target': LaunchConfiguration('max_relative_target'),
                'calibration_file': LaunchConfiguration('calibration_file'),
            }],
            output='screen'
        ),
    ])

