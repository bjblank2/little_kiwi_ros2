#!/usr/bin/env python3

"""
SO101 Puppet Node
This node subscribes to joint states from the puppeteer arm and replicates
the movements by sending commands to the puppet SO101 arm.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from std_msgs.msg import Header
import json
from pathlib import Path

# Import SO101 communication components (local implementation)
from kiwi_robot.so101 import Motor, MotorCalibration, MotorNormMode, FeetechMotorsBus, OperatingMode


class SO101PuppetNode(Node):
    """
    ROS2 node for the SO-101 arm puppet (follower).
    This node subscribes to joint states from the puppeteer arm and mimics
    the movements by sending the same joint positions to the puppet arm.
    """

    def __init__(self):
        super().__init__('so101_puppet')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('arm_id', 'puppet_arm')
        self.declare_parameter('puppeteer_arm_id', 'puppeteer_arm')
        self.declare_parameter('max_relative_target', 20.0)
        self.declare_parameter('use_degrees', True)
        self.declare_parameter('calibration_file', '')

        # Handle string-to-number conversion from launch files
        port_val = self.get_parameter('port').value
        self.port = str(port_val) if port_val is not None else '/dev/ttyACM1'
        
        arm_id_val = self.get_parameter('arm_id').value
        self.arm_id = str(arm_id_val) if arm_id_val is not None else 'puppet_arm'
        
        puppeteer_id_val = self.get_parameter('puppeteer_arm_id').value
        self.puppeteer_arm_id = str(puppeteer_id_val) if puppeteer_id_val is not None else 'puppeteer_arm'
        
        max_target_val = self.get_parameter('max_relative_target').value
        self.max_relative_target = float(max_target_val) if max_target_val is not None else 20.0
        
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

        # Map our joint names to SO101 joint names
        self.joint_name_map = {
            'shoulder_pan': 'shoulder_pan',
            'shoulder_lift': 'shoulder_lift',
            'elbow': 'elbow_flex',
            'wrist_pitch': 'wrist_flex',
            'wrist_roll': 'wrist_roll',
            'wrist_yaw': 'gripper'  # Map wrist_yaw to gripper if needed
        }

        # Teleoperation state (disabled by default - must be enabled via service)
        self.teleop_enabled = False
        self.current_joint_states = {}
        self.last_puppeteer_msg = None  # Cache last received puppeteer joint state

        # Subscriber for puppeteer arm joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            f'{self.puppeteer_arm_id}/joint_states',
            self.joint_state_callback,
            30
        )

        # Publisher for puppet arm joint states (for monitoring)
        self.puppet_joint_state_pub = self.create_publisher(
            JointState,
            f'{self.arm_id}/joint_states',
            30
        )

        # Service for enabling/disabling teleoperation
        self.teleop_service = self.create_service(
            SetBool,
            f'{self.arm_id}/set_teleop',
            self.teleop_service_callback
        )

        # Initialize the SO-101 arm connection
        self.init_arm()

        # Timer for publishing puppet joint states
        self.timer = self.create_timer(0.1, self.publish_puppet_states)

        self.get_logger().info(
            f'SO-101 Puppet Node initialized on port {self.port}'
        )
        self.get_logger().info(
            f'Teleoperation DISABLED by default - torque is disabled'
        )
        self.get_logger().info(
            f'To enable: ros2 service call {self.arm_id}/set_teleop std_srvs/srv/SetBool "{{data: true}}"'
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

    def teleop_service_callback(self, request, response):
        """Service callback to enable/disable teleoperation and torque."""
        self.teleop_enabled = request.data

        if self.teleop_enabled:
            # Enable torque when teleoperation is activated
            try:
                # Before enabling torque, set goal position to current position to prevent unwanted movement
                # Then immediately send puppeteer's position if available
                try:
                    # Read current puppet arm position
                    puppet_current = self.bus.sync_read("Present_Position", normalize=True)
                    
                    # Set goal to current position first (so it doesn't move when torque is enabled)
                    self.bus.sync_write("Goal_Position", puppet_current)
                    
                    # Small delay to let the goal position register
                    import time
                    time.sleep(0.1)
                except Exception as e:
                    self.get_logger().warn(f'Could not read puppet current position: {e}')
                
                # Now enable torque
                self.bus.enable_torque()
                
                # Immediately send puppeteer's last known position to sync arms
                if self.last_puppeteer_msg is not None:
                    try:
                        goal_positions = {}
                        for i, joint_name in enumerate(self.last_puppeteer_msg.name):
                            if i < len(self.last_puppeteer_msg.position):
                                so101_name = self.joint_name_map.get(joint_name)
                                if so101_name:
                                    goal_positions[so101_name] = float(self.last_puppeteer_msg.position[i])
                        
                        if goal_positions:
                            self.bus.sync_write("Goal_Position", goal_positions)
                            self.get_logger().info(
                                '✓ Set puppet to puppeteer position upon enabling torque'
                            )
                    except Exception as e:
                        self.get_logger().warn(
                            f'Could not sync to puppeteer position: {e}. '
                            'Will sync on next joint state message.'
                        )
                else:
                    self.get_logger().info(
                        'No puppeteer position available yet. Will sync when first message arrives.'
                    )
                
                self.get_logger().info(
                    '✓ Teleoperation ENABLED - torque enabled, puppet will mimic puppeteer'
                )
                response.message = "Teleoperation enabled and torque enabled"
                response.success = True
            except Exception as e:
                self.get_logger().error(
                    f'Failed to enable torque: {e}'
                )
                response.message = f"Failed to enable torque: {e}"
                response.success = False
        else:
            # Disable torque when teleoperation is disabled
            try:
                self.bus.disable_torque()
                self.get_logger().info(
                    '✓ Teleoperation DISABLED - torque disabled, puppet will not move'
                )
                response.message = "Teleoperation disabled and torque disabled"
                response.success = True
            except Exception as e:
                self.get_logger().error(
                    f'Failed to disable torque: {e}'
                )
                response.message = f"Failed to disable torque: {e}"
                response.success = False

        return response

    def init_arm(self):
        """Initialize the SO-101 arm connection."""
        try:
            self.bus.connect(handshake=True)
            
            # Configure motors (disable torque temporarily for configuration)
            self.bus.disable_torque()
            self.bus.configure_motors()
            for motor in self.bus.motors:
                self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                # Set P_Coefficient to lower value to avoid shakiness
                self.bus.write("P_Coefficient", motor, 16)
                self.bus.write("I_Coefficient", motor, 0)
                self.bus.write("D_Coefficient", motor, 32)
                
                if motor == "gripper":
                    self.bus.write("Max_Torque_Limit", motor, 500)
                    self.bus.write("Protection_Current", motor, 250)
                    self.bus.write("Overload_Torque", motor, 25)
            
            # Keep torque disabled by default - will be enabled via service call
            # Torque will be enabled when teleoperation is activated
            
            self.get_logger().info(
                f'Successfully connected to SO-101 puppet arm on {self.port}'
            )
            self.get_logger().info(
                '⚠️  Torque DISABLED by default - arm can be moved manually'
            )
            self.get_logger().info(
                f'Enable teleoperation via service: ros2 service call {self.arm_id}/set_teleop std_srvs/srv/SetBool "{{data: true}}"'
            )
        except Exception as e:
            self.get_logger().error(
                f'Failed to initialize SO-101 arm: {str(e)}'
            )
            raise

    def joint_state_callback(self, msg):
        """Callback for puppeteer arm joint states."""
        # Cache the latest message for use when enabling teleoperation
        self.last_puppeteer_msg = msg
        
        if not self.bus.is_connected:
            self.get_logger().warn(
                'Puppet arm not connected, attempting to reconnect...'
            )
            try:
                self.init_arm()
            except Exception:
                return
            return

        if not self.teleop_enabled:
            # Only log once per minute to avoid spam (teleop is disabled by default)
            if not hasattr(self, '_last_teleop_warn') or \
               (self.get_clock().now().nanoseconds - self._last_teleop_warn) > 60_000_000_000:
                self.get_logger().info(
                    'Teleoperation is DISABLED - puppet will not move. '
                    f'Enable teleoperation and torque with: ros2 service call {self.arm_id}/set_teleop std_srvs/srv/SetBool "{{data: true}}"'
                )
                self._last_teleop_warn = self.get_clock().now().nanoseconds
            return

        try:
            # Build goal position dictionary mapping our names to SO101 names
            goal_positions = {}
            
            for i, joint_name in enumerate(msg.name):
                if i < len(msg.position):
                    # Map from our joint name to SO101 name
                    so101_name = self.joint_name_map.get(joint_name)
                    if so101_name:
                        goal_positions[so101_name] = float(msg.position[i])
            
            if not goal_positions:
                # Log once if no joints are mapped
                if not hasattr(self, '_no_joints_warned'):
                    self.get_logger().warn(
                        f'Received joint states but no joints mapped. '
                        f'Received joints: {msg.name}'
                    )
                    self._no_joints_warned = True
                return

            # Apply safety limits if needed
            if self.max_relative_target > 0:
                try:
                    present_positions = self.bus.sync_read("Present_Position", normalize=True)
                    
                    # Clip goal positions that are too far from present
                    for so101_name, goal_pos in goal_positions.items():
                        if so101_name in present_positions:
                            present_pos = present_positions[so101_name]
                            diff = abs(goal_pos - present_pos)
                            if diff > self.max_relative_target:
                                # Clamp to max relative target
                                if goal_pos > present_pos:
                                    goal_positions[so101_name] = present_pos + self.max_relative_target
                                else:
                                    goal_positions[so101_name] = present_pos - self.max_relative_target
                except Exception as e:
                    self.get_logger().warn(f'Could not apply safety limits: {e}')

            # Send command to puppet arm
            self.bus.sync_write("Goal_Position", goal_positions)
            self.current_joint_states = goal_positions.copy()
                    
        except Exception as e:
            self.get_logger().error(
                f'Error sending command to puppet arm: {str(e)}'
            )

    def publish_puppet_states(self):
        """Publish current puppet arm joint states."""
        if not self.bus.is_connected:
            return

        try:
            # Read current joint positions
            positions = self.bus.sync_read("Present_Position", normalize=True)
            self.current_joint_states = positions.copy()

            # Map SO101 names to our names
            joint_names = []
            joint_positions = []
            
            # Reverse mapping: SO101 name -> our name
            reverse_map = {v: k for k, v in self.joint_name_map.items()}
            
            for so101_name, position in positions.items():
                our_name = reverse_map.get(so101_name)
                if our_name:
                    joint_names.append(our_name)
                    joint_positions.append(float(position))

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
            
            self.puppet_joint_state_pub.publish(joint_state_msg)
            
        except Exception as e:
            self.get_logger().error(
                f'Error publishing puppet joint states: {str(e)}'
            )

    def on_shutdown(self):
        """Cleanup on shutdown."""
        if self.bus.is_connected:
            try:
                self.bus.disconnect(disable_torque=True)
                self.get_logger().info('SO-101 puppet arm disconnected')
            except Exception as e:
                self.get_logger().error(
                    f'Error disconnecting arm: {str(e)}'
                )


def main(args=None):
    rclpy.init(args=args)

    node = SO101PuppetNode()

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