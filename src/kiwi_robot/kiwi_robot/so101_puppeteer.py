#!/usr/bin/env python3

"""
SO101 Puppeteer Node
This node reads joint positions from the SO101 arm (leader/teleop arm)
and publishes them as JointState messages so the puppet arm can replicate movements.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import json
from pathlib import Path

# Import SO101 communication components (local implementation)
from kiwi_robot.so101 import Motor, MotorCalibration, MotorNormMode, FeetechMotorsBus, OperatingMode


class SO101PuppeteerNode(Node):
    """
    ROS2 node for the SO-101 arm puppeteer (leader/teleop input).
    This node reads the joint positions from the puppeteer arm and publishes them
    so that the puppet arm can mimic the movements.
    """

    def __init__(self):
        super().__init__('so101_puppeteer')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('arm_id', 'puppeteer_arm')
        self.declare_parameter('use_degrees', True)
        self.declare_parameter('calibration_file', '')

        # Handle string-to-number conversion from launch files
        port_val = self.get_parameter('port').value
        self.port = str(port_val) if port_val is not None else '/dev/ttyACM0'
        
        publish_rate_val = self.get_parameter('publish_rate').value
        self.publish_rate = float(publish_rate_val) if publish_rate_val is not None else 50.0
        
        arm_id_val = self.get_parameter('arm_id').value
        self.arm_id = str(arm_id_val) if arm_id_val is not None else 'puppeteer_arm'
        
        use_degrees_val = self.get_parameter('use_degrees').value
        self.use_degrees = bool(use_degrees_val) if use_degrees_val is not None else True
        
        calibration_file_val = self.get_parameter('calibration_file').value
        calibration_file = str(calibration_file_val) if calibration_file_val is not None else ''

        # Load calibration if provided
        calibration = None
        if calibration_file:
            try:
                calibration = self.load_calibration(calibration_file)
                self.get_logger().info(f'Loaded calibration from {calibration_file}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load calibration: {e}')

        # Setup motor normalization mode
        norm_mode_body = MotorNormMode.DEGREES if self.use_degrees else MotorNormMode.RANGE_M100_100

        # Initialize the SO-101 arm bus using FeetechMotorsBus
        self.bus = FeetechMotorsBus(
            port=self.port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
                "elbow_flex": Motor(3, "sts3215", norm_mode_body),
                "wrist_flex": Motor(4, "sts3215", norm_mode_body),
                "wrist_roll": Motor(5, "sts3215", norm_mode_body),
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=calibration,
        )

        # Map SO101 joint names to our joint names (elbow_flex -> elbow, wrist_flex -> wrist_pitch)
        self.joint_name_map = {
            'shoulder_pan': 'shoulder_pan',
            'shoulder_lift': 'shoulder_lift',
            'elbow_flex': 'elbow',
            'wrist_flex': 'wrist_pitch',
            'wrist_roll': 'wrist_roll',
            'gripper': 'wrist_yaw'  # Using gripper as wrist_yaw or could skip it
        }

        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(
            JointState,
            f'{self.arm_id}/joint_states',
            30
        )

        # Initialize the SO-101 arm connection
        self.init_arm()

        # Timer for publishing joint states
        self.timer = self.create_timer(
            1.0 / float(self.publish_rate),
            self.publish_joint_states
        )

        self.get_logger().info(
            f'SO-101 Puppeteer Node initialized on port {self.port}'
        )

    def load_calibration(self, path):
        """Load calibration JSON file and convert to MotorCalibration dict."""
        try:
            with open(path, 'r') as f:
                calib_data = json.load(f)
            
            calibration = {}
            for joint_name, calib_dict in calib_data.items():
                calibration[joint_name] = MotorCalibration(
                    id=calib_dict['id'],
                    drive_mode=calib_dict.get('drive_mode', 0),
                    homing_offset=calib_dict['homing_offset'],
                    range_min=calib_dict['range_min'],
                    range_max=calib_dict['range_max']
                )
            return calibration
        except Exception as e:
            self.get_logger().error(f'Failed to load calibration file {path}: {e}')
            return None

    def init_arm(self):
        """Initialize the SO-101 arm connection."""
        try:
            self.bus.connect(handshake=True)
            
            # Disable torque first so arm can be moved manually
            self.bus.disable_torque()
            
            # Configure motors (with torque disabled)
            self.bus.configure_motors()
            for motor in self.bus.motors:
                self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            
            # Ensure torque stays disabled (configure_motors might re-enable it)
            self.bus.disable_torque()
            
            # Read initial wrist_roll position for offset
            # try:
            #     initial_positions = self.bus.sync_read("Present_Position", normalize=False)
            #     if 'wrist_roll' in initial_positions:
            #         self.wrist_roll_offset = initial_positions['wrist_roll']
            #         self.get_logger().info(f'Set wrist_roll offset to {self.wrist_roll_offset}')
            # except Exception as e:
            #     self.get_logger().warn(f'Could not read initial offset: {e}')

            self.get_logger().info(
                f'Successfully connected to SO-101 puppeteer arm on {self.port}'
            )
            self.get_logger().info(
                '⚠️  TORQUE IS DISABLED - You can now manually move the arm'
            )
        except Exception as e:
            self.get_logger().error(
                f'Failed to initialize SO-101 arm: {str(e)}'
            )
            raise

    def publish_joint_states(self):
        """Read joint positions and publish them as JointState message."""
        if not self.bus.is_connected:
            self.get_logger().warn(
                'Arm not connected, attempting to reconnect...'
            )
            try:
                self.init_arm()
            except Exception:
                return
            return

        try:
            # Read current joint positions (in degrees if use_degrees=True)
            positions = self.bus.sync_read("Present_Position", normalize=True)
            
            # Map to our joint names and collect
            joint_names = []
            joint_positions = []
            
            for so101_name, our_name in self.joint_name_map.items():
                if so101_name in positions:
                    val = float(positions[so101_name])
                    # Apply wrist_roll offset if set
                    # if so101_name == 'wrist_roll' and self.wrist_roll_offset is not None:
                    #     # Get raw position for offset calculation
                    #     raw_positions = self.bus.sync_read("Present_Position", normalize=False)
                    #     if 'wrist_roll' in raw_positions:
                    #         raw_val = raw_positions['wrist_roll']
                    #         # Convert to degrees with offset
                    #         val = val - ((raw_val - self.wrist_roll_offset) * 360.0 / 4095.0)
                    
                    joint_names.append(our_name)
                    joint_positions.append(val)

            if not joint_positions:
                return

            # Create and publish JointState message
            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = joint_names
            joint_state_msg.position = joint_positions
            joint_state_msg.velocity = [0.0] * len(joint_positions)
            joint_state_msg.effort = [0.0] * len(joint_positions)
            
            self.joint_state_pub.publish(joint_state_msg)
            
        except Exception as e:
            self.get_logger().error(
                f'Error publishing joint states: {str(e)}'
            )

    def on_shutdown(self):
        """Cleanup on shutdown."""
        if self.bus.is_connected:
            try:
                self.bus.disconnect(disable_torque=True)
                self.get_logger().info('SO-101 puppeteer arm disconnected')
            except Exception as e:
                self.get_logger().error(
                    f'Error disconnecting arm: {str(e)}'
                )


def main(args=None):
    rclpy.init(args=args)

    node = SO101PuppeteerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()