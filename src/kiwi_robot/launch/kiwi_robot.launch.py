from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_puppeteer',
            default_value='true',
            description='Start SO101 puppeteer node'
        ),
        DeclareLaunchArgument(
            'start_puppet',
            default_value='true',
            description='Start SO101 puppet node'
        ),
        DeclareLaunchArgument(
            'start_base',
            default_value='true',
            description='Start Kiwi base node'
        ),
        DeclareLaunchArgument(
            'start_cam',
            default_value='true',
            description='Start Kiwi camera node'
        ),
        
        # Puppeteer
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('kiwi_robot'),
                    'launch',
                    'so101_puppeteer.launch.py'
                ])
            ]),
            condition=IfCondition(LaunchConfiguration('start_puppeteer'))
        ),
        
        # Puppet
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('kiwi_robot'),
                    'launch',
                    'so101_puppet.launch.py'
                ])
            ]),
            condition=IfCondition(LaunchConfiguration('start_puppet'))
        ),
        
        # Base
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('kiwi_robot'),
                    'launch',
                    'kiwi_base.launch.py'
                ])
            ]),
            condition=IfCondition(LaunchConfiguration('start_base'))
        ),
        
        # Camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('kiwi_robot'),
                    'launch',
                    'kiwi_cam.launch.py'
                ])
            ]),
            condition=IfCondition(LaunchConfiguration('start_cam'))
        ),
    ])
