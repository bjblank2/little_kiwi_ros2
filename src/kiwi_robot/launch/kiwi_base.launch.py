from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='1.0',
            description='Maximum linear speed (m/s)'
        ),
        DeclareLaunchArgument(
            'max_angular_speed',
            default_value='2.0',
            description='Maximum angular speed (rad/s)'
        ),
        DeclareLaunchArgument(
            'axis_linear_x',
            default_value='1',
            description='Joystick axis for linear X (forward/back)'
        ),
        DeclareLaunchArgument(
            'axis_linear_y',
            default_value='0',
            description='Joystick axis for linear Y (strafe)'
        ),
        DeclareLaunchArgument(
            'axis_angular_z',
            default_value='3',
            description='Joystick axis for angular Z (rotation)'
        ),
        DeclareLaunchArgument(
            'wheel_separation_x',
            default_value='0.3',
            description='Wheel separation in X direction (m)'
        ),
        DeclareLaunchArgument(
            'wheel_separation_y',
            default_value='0.3',
            description='Wheel separation in Y direction (m)'
        ),
        DeclareLaunchArgument(
            'wheel_radius',
            default_value='0.05',
            description='Wheel radius (m)'
        ),
        
        Node(
            package='kiwi_robot',
            executable='kiwi_base',
            name='kiwi_base',
            parameters=[{
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
                'axis_linear_x': LaunchConfiguration('axis_linear_x'),
                'axis_linear_y': LaunchConfiguration('axis_linear_y'),
                'axis_angular_z': LaunchConfiguration('axis_angular_z'),
                'wheel_separation_x': LaunchConfiguration('wheel_separation_x'),
                'wheel_separation_y': LaunchConfiguration('wheel_separation_y'),
                'wheel_radius': LaunchConfiguration('wheel_radius'),
            }],
            output='screen'
        ),
    ])

