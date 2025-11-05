from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for SO101 calibration node.
    
    NOTE: This launch file is provided for convenience, but the calibration node
    requires interactive input (stdin) which is not available when launched via
    ros2 launch. For best results, run the calibration node directly:
    
        ros2 run kiwi_robot so101_calibrate --ros-args \
            -p port:=/dev/ttyACM0 -p arm_id:=puppeteer_arm
    
    If you use this launch file, you may encounter hanging at input prompts.
    """
    return LaunchDescription([
        LogInfo(
            msg=[
                '\n',
                '='*60, '\n',
                'WARNING: Calibration requires interactive input.\n',
                'If the node hangs waiting for input, run it directly instead:\n',
                '  ros2 run kiwi_robot so101_calibrate --ros-args ',
                '-p port:=', LaunchConfiguration('port'), ' ',
                '-p arm_id:=', LaunchConfiguration('arm_id'), '\n',
                '='*60, '\n',
            ]
        ),
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Serial port for SO101 arm'
        ),
        DeclareLaunchArgument(
            'arm_id',
            default_value='arm',
            description='Arm ID (e.g., puppeteer_arm or puppet_arm)'
        ),
        DeclareLaunchArgument(
            'output_file',
            default_value='',
            description='Output calibration file path (default: {arm_id}_calibration.json)'
        ),
        
        Node(
            package='kiwi_robot',
            executable='so101_calibrate',
            name='so101_calibrate',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'arm_id': LaunchConfiguration('arm_id'),
                'output_file': LaunchConfiguration('output_file'),
            }],
            output='screen'
        ),
    ])

