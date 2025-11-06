#!/usr/bin/env python3

"""
Kiwi Mobile Puppet Node
Unified node that combines:
- SO101 puppet arm control (from so101_puppet)
- Kiwi base joystick control (from kiwi_base)
- Wheel servo control (from wheel_hw_bridge)

Uses a single serial port to control both arm and base servos.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from std_msgs.msg import Header
import json
import math
from pathlib import Path
import time

# Import SO101 communication components (local implementation)
from kiwi_robot.so101 import Motor, MotorCalibration, MotorNormMode, FeetechMotorsBus, OperatingMode


class KiwiMobilePuppetNode(Node):
    """
    Unified ROS2 node for controlling both the SO101 puppet arm and Kiwi base.
    Uses a single serial port to communicate with all servos.
    """

    def __init__(self):
        super().__init__('kiwi_mobile_puppet')

        # ========== Parameters ==========
        # Serial port (shared by all servos)
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 1_000_000)
        
        # Arm parameters
        self.declare_parameter('arm_id', 'puppet_arm')
        self.declare_parameter('puppeteer_arm_id', 'puppeteer_arm')
        self.declare_parameter('max_relative_target', 20.0)
        self.declare_parameter('use_degrees', True)
        self.declare_parameter('arm_calibration_file', '')
        
        # Base parameters - joystick mapping
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_linear_y', 0)
        self.declare_parameter('axis_angular_z', 3)
        
        # Base parameters - wheel geometry
        self.declare_parameter('wheel_separation_x', 0.3)
        self.declare_parameter('wheel_separation_y', 0.3)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('ticks_per_rev', 4096)
        
        # Wheel configuration (3-wheel omni base)
        self.declare_parameter('wheel_count', 3)  # 3 or 4 wheels

        # Get parameter values
        port_val = self.get_parameter('port').value
        self.port = str(port_val) if port_val is not None else '/dev/ttyACM1'
        
        baudrate_val = self.get_parameter('baudrate').value
        self.baudrate = int(baudrate_val) if baudrate_val is not None else 1_000_000
        
        arm_id_val = self.get_parameter('arm_id').value
        self.arm_id = str(arm_id_val) if arm_id_val is not None else 'puppet_arm'
        
        puppeteer_id_val = self.get_parameter('puppeteer_arm_id').value
        self.puppeteer_arm_id = str(puppeteer_id_val) if puppeteer_id_val is not None else 'puppeteer_arm'
        
        max_target_val = self.get_parameter('max_relative_target').value
        self.max_relative_target = float(max_target_val) if max_target_val is not None else 20.0
        
        use_degrees_val = self.get_parameter('use_degrees').value
        self.use_degrees = bool(use_degrees_val) if use_degrees_val is not None else True
        
        arm_calibration_file_val = self.get_parameter('arm_calibration_file').value
        arm_calibration_file = str(arm_calibration_file_val) if arm_calibration_file_val is not None else ''
        
        # Base parameters
        max_linear_val = self.get_parameter('max_linear_speed').value
        self.max_linear_speed = float(max_linear_val) if max_linear_val is not None else 1.0
        
        max_angular_val = self.get_parameter('max_angular_speed').value
        self.max_angular_speed = float(max_angular_val) if max_angular_val is not None else 2.0
        
        axis_x_val = self.get_parameter('axis_linear_x').value
        self.axis_linear_x = int(axis_x_val) if axis_x_val is not None else 1
        
        axis_y_val = self.get_parameter('axis_linear_y').value
        self.axis_linear_y = int(axis_y_val) if axis_y_val is not None else 0
        
        axis_z_val = self.get_parameter('axis_angular_z').value
        self.axis_angular_z = int(axis_z_val) if axis_z_val is not None else 3
        
        wheel_x_val = self.get_parameter('wheel_separation_x').value
        self.wheel_separation_x = float(wheel_x_val) if wheel_x_val is not None else 0.3
        
        wheel_y_val = self.get_parameter('wheel_separation_y').value
        self.wheel_separation_y = float(wheel_y_val) if wheel_y_val is not None else 0.3
        
        wheel_r_val = self.get_parameter('wheel_radius').value
        self.wheel_radius = float(wheel_r_val) if wheel_r_val is not None else 0.05
        
        ticks_per_rev_val = self.get_parameter('ticks_per_rev').value
        self.ticks_per_rev = int(ticks_per_rev_val) if ticks_per_rev_val is not None else 4096
        
        wheel_count_val = self.get_parameter('wheel_count').value
        self.wheel_count = int(wheel_count_val) if wheel_count_val is not None else 3

        # Load arm calibration if provided
        arm_calibration = None
        if arm_calibration_file:
            try:
                arm_calibration = self.load_calibration(arm_calibration_file)
                self.get_logger().info(f'Loaded arm calibration from {arm_calibration_file}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load arm calibration: {e}')

        # Setup motor normalization mode
        norm_mode_body = MotorNormMode.DEGREES if self.use_degrees else MotorNormMode.RANGE_M100_100

        # Define wheel names and IDs based on wheel count
        if self.wheel_count == 3:
            self.wheel_names = ["wheel_1", "wheel_2", "wheel_3"]
            self.wheel_ids = {"wheel_1": 7, "wheel_2": 8, "wheel_3": 9}
        else:  # 4 wheels (mecanum)
            self.wheel_names = ["wheel_1", "wheel_2", "wheel_3", "wheel_4"]
            self.wheel_ids = {"wheel_1": 7, "wheel_2": 8, "wheel_3": 9, "wheel_4": 10}

        # Initialize unified FeetechMotorsBus with ALL servos (arm + wheels)
        all_motors = {
            # Arm servos (IDs 1-6)
            "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
            "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
            "elbow_flex": Motor(3, "sts3215", norm_mode_body),
            "wrist_flex": Motor(4, "sts3215", norm_mode_body),
            "wrist_roll": Motor(5, "sts3215", norm_mode_body),
            "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        }
        
        # Add wheel servos
        for wheel_name in self.wheel_names:
            all_motors[wheel_name] = Motor(
                self.wheel_ids[wheel_name],
                "sts3215",
                MotorNormMode.RANGE_M100_100
            )

        self.bus = FeetechMotorsBus(
            port=self.port,
            motors=all_motors,
            calibration=arm_calibration,
        )

        # Map joint names for arm
        self.joint_name_map = {
            'shoulder_pan': 'shoulder_pan',
            'shoulder_lift': 'shoulder_lift',
            'elbow': 'elbow_flex',
            'wrist_pitch': 'wrist_flex',
            'wrist_roll': 'wrist_roll',
            'wrist_yaw': 'gripper'
        }

        # Arm teleoperation state
        self.teleop_enabled = False
        self.current_arm_joint_states = {}
        self.last_puppeteer_msg = None

        # Wheel state
        self.current_wheel_velocities = {wheel: 0.0 for wheel in self.wheel_names}

        # ========== ROS Subscribers ==========
        # Joystick input for base control
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Puppeteer arm joint states for arm control
        self.joint_state_sub = self.create_subscription(
            JointState,
            f'{self.puppeteer_arm_id}/joint_states',
            self.arm_joint_state_callback,
            30
        )

        # ========== ROS Publishers ==========
        # Base cmd_vel (for compatibility)
        self.twist_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Puppet arm joint states
        self.puppet_arm_joint_state_pub = self.create_publisher(
            JointState,
            f'{self.arm_id}/joint_states',
            30
        )

        # Wheel joint states
        self.wheel_joint_state_pub = self.create_publisher(
            JointState,
            '/wheel_joint_states',
            10
        )

        # ========== ROS Services ==========
        # Service for enabling/disabling arm teleoperation
        self.teleop_service = self.create_service(
            SetBool,
            f'{self.arm_id}/set_teleop',
            self.teleop_service_callback
        )

        # ========== Timers ==========
        # Timer for publishing arm joint states
        self.arm_state_timer = self.create_timer(0.1, self.publish_arm_states)
        
        # Timer for publishing wheel joint states
        self.wheel_state_timer = self.create_timer(0.05, self.publish_wheel_states)

        # Initialize the unified servo connection
        self.init_servos()

        self.get_logger().info(
            f'Kiwi Mobile Puppet Node initialized on port {self.port}'
        )
        self.get_logger().info(
            f'Arm teleoperation DISABLED by default'
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

    def init_servos(self):
        """Initialize the unified servo connection (arm + wheels)."""
        try:
            self.bus.connect(handshake=True)
            
            # Configure all motors
            self.bus.disable_torque()
            self.bus.configure_motors()
            
            # Configure arm motors (position mode)
            arm_motors = ["shoulder_pan", "shoulder_lift", "elbow_flex", 
                          "wrist_flex", "wrist_roll", "gripper"]
            for motor in arm_motors:
                if motor in self.bus.motors:
                    self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                    self.bus.write("P_Coefficient", motor, 16)
                    self.bus.write("I_Coefficient", motor, 0)
                    self.bus.write("D_Coefficient", motor, 32)
                    
                    if motor == "gripper":
                        self.bus.write("Max_Torque_Limit", motor, 500)
                        self.bus.write("Protection_Current", motor, 250)
                        self.bus.write("Overload_Torque", motor, 25)
            
            # Configure wheel motors (velocity mode)
            for wheel in self.wheel_names:
                if wheel in self.bus.motors:
                    self.bus.write("Operating_Mode", wheel, OperatingMode.VELOCITY.value)
            
            # Enable torque for wheels immediately (they need to be controlled)
            # Arm torque will be enabled via service call
            self.bus.enable_torque(motors=self.wheel_names)
            
            self.get_logger().info(
                f'Successfully connected to unified servo bus on {self.port}'
            )
            self.get_logger().info(
                f'Configured {len(arm_motors)} arm servos and {len(self.wheel_names)} wheel servos'
            )
            self.get_logger().info(
                '⚠️  Arm torque DISABLED by default - wheels are ENABLED'
            )
            
        except Exception as e:
            self.get_logger().error(
                f'Failed to initialize servos: {str(e)}'
            )
            raise

    def teleop_service_callback(self, request, response):
        """Service callback to enable/disable arm teleoperation and torque."""
        self.teleop_enabled = request.data

        if self.teleop_enabled:
            try:
                # Read current arm position
                try:
                    arm_motors = ["shoulder_pan", "shoulder_lift", "elbow_flex", 
                                 "wrist_flex", "wrist_roll", "gripper"]
                    arm_current = self.bus.sync_read("Present_Position", arm_motors, normalize=True)
                    
                    # Set goal to current position first
                    self.bus.sync_write("Goal_Position", arm_current)
                    time.sleep(0.1)
                except Exception as e:
                    self.get_logger().warn(f'Could not read arm current position: {e}')
                
                # Enable arm torque
                arm_motors = ["shoulder_pan", "shoulder_lift", "elbow_flex", 
                             "wrist_flex", "wrist_roll", "gripper"]
                self.bus.enable_torque(motors=arm_motors)
                
                # Sync to puppeteer position if available
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
                            self.get_logger().info('✓ Set puppet to puppeteer position')
                    except Exception as e:
                        self.get_logger().warn(f'Could not sync to puppeteer position: {e}')
                
                self.get_logger().info('✓ Arm teleoperation ENABLED')
                response.message = "Arm teleoperation enabled"
                response.success = True
            except Exception as e:
                self.get_logger().error(f'Failed to enable arm torque: {e}')
                response.message = f"Failed to enable arm torque: {e}"
                response.success = False
        else:
            try:
                # Disable arm torque
                arm_motors = ["shoulder_pan", "shoulder_lift", "elbow_flex", 
                             "wrist_flex", "wrist_roll", "gripper"]
                self.bus.disable_torque(motors=arm_motors)
                
                self.get_logger().info('✓ Arm teleoperation DISABLED')
                response.message = "Arm teleoperation disabled"
                response.success = True
            except Exception as e:
                self.get_logger().error(f'Failed to disable arm torque: {e}')
                response.message = f"Failed to disable arm torque: {e}"
                response.success = False

        return response

    def joy_callback(self, msg):
        """Callback for joystick messages - controls base movement."""
        if not self.bus.is_connected:
            return

        # Check if axes are available
        if (self.axis_linear_x >= len(msg.axes) or
            self.axis_linear_y >= len(msg.axes) or
            self.axis_angular_z >= len(msg.axes)):
            self.get_logger().warn_throttle(
                2.0,
                'Axis index out of range for received Joy message'
            )
            return

        # Get joystick values
        linear_x_raw = msg.axes[self.axis_linear_x] if self.axis_linear_x < len(msg.axes) else 0.0
        linear_y_raw = msg.axes[self.axis_linear_y] if self.axis_linear_y < len(msg.axes) else 0.0
        angular_z_raw = msg.axes[self.axis_angular_z] if self.axis_angular_z < len(msg.axes) else 0.0

        # Apply dead zone
        dead_zone = 0.1
        if abs(linear_x_raw) < dead_zone:
            linear_x_raw = 0.0
        if abs(linear_y_raw) < dead_zone:
            linear_y_raw = 0.0
        if abs(angular_z_raw) < dead_zone:
            angular_z_raw = 0.0

        # Scale to max speeds
        vx = linear_x_raw * self.max_linear_speed
        vy = linear_y_raw * self.max_linear_speed
        omega = angular_z_raw * self.max_angular_speed

        # Calculate wheel velocities based on wheel count
        if self.wheel_count == 3:
            wheel_velocities = self.omni3_kinematics(vx, vy, omega)
        else:
            wheel_velocities = self.mecanum_kinematics(vx, vy, omega)

        # Publish cmd_vel for compatibility
        twist_msg = Twist()
        twist_msg.linear.x = vx
        twist_msg.linear.y = vy
        twist_msg.angular.z = omega
        self.twist_pub.publish(twist_msg)

        # Send wheel velocity commands directly to servos
        try:
            wheel_vel_cmds = {}
            for i, wheel_name in enumerate(self.wheel_names):
                if i < len(wheel_velocities):
                    # Convert m/s to rad/s, then to raw servo value
                    rad_per_sec = wheel_velocities[i] / self.wheel_radius
                    raw_vel = self.rad_s_to_raw(rad_per_sec)
                    wheel_vel_cmds[wheel_name] = raw_vel
                    self.current_wheel_velocities[wheel_name] = rad_per_sec
            
            if wheel_vel_cmds:
                self.bus.sync_write("Goal_Velocity", wheel_vel_cmds)
        except Exception as e:
            self.get_logger().error(f'Failed to send wheel velocities: {e}')

    def omni3_kinematics(self, vx, vy, omega):
        """
        Calculate 3-wheel omni wheel velocities from base velocities.
        
        Args:
            vx: Linear velocity in x direction [m/s]
            vy: Linear velocity in y direction [m/s]
            omega: Angular velocity [rad/s]
            
        Returns:
            List of wheel velocities [w1, w2, w3] [m/s]
        """
        R = (self.wheel_separation_x + self.wheel_separation_y) / 4.0  # Approximate base radius
        
        # 3-wheel omni arrangement (120 degrees apart)
        w1 = (-math.sin(math.radians(60))*vx + math.cos(math.radians(60))*vy + R*omega) / self.wheel_radius
        w2 = (-math.sin(math.radians(180))*vx + math.cos(math.radians(180))*vy + R*omega) / self.wheel_radius
        w3 = (-math.sin(math.radians(300))*vx + math.cos(math.radians(300))*vy + R*omega) / self.wheel_radius
        
        return [w1 * self.wheel_radius, w2 * self.wheel_radius, w3 * self.wheel_radius]

    def mecanum_kinematics(self, vx, vy, omega):
        """
        Calculate mecanum wheel velocities from base velocities.
        
        For a mecanum base with 4 wheels arranged in a rectangle:
        - Front-left, Front-right, Back-left, Back-right
        
        Args:
            vx: Linear velocity in x direction (forward/backward) [m/s]
            vy: Linear velocity in y direction (left/right strafe) [m/s]
            omega: Angular velocity (rotation) [rad/s]
            
        Returns:
            List of wheel velocities [front_left, front_right, back_left, back_right] [m/s]
        """
        L = self.wheel_separation_x / 2.0
        W = self.wheel_separation_y / 2.0
        
        front_left = vx + vy + (L + W) * omega
        front_right = vx - vy - (L + W) * omega
        back_left = vx - vy + (L + W) * omega
        back_right = vx + vy - (L + W) * omega
        
        return [front_left, front_right, back_left, back_right]

    def rad_s_to_raw(self, rad_s: float) -> int:
        """Convert rad/s to raw servo velocity value."""
        deg_s = rad_s * 180.0 / math.pi
        ticks_per_deg = self.ticks_per_rev / 360.0
        return int(deg_s * ticks_per_deg)

    def arm_joint_state_callback(self, msg):
        """Callback for puppeteer arm joint states."""
        self.last_puppeteer_msg = msg
        
        if not self.bus.is_connected:
            self.get_logger().warn('Servo bus not connected, attempting to reconnect...')
            try:
                self.init_servos()
            except Exception:
                return
            return

        if not self.teleop_enabled:
            return

        try:
            goal_positions = {}
            
            for i, joint_name in enumerate(msg.name):
                if i < len(msg.position):
                    so101_name = self.joint_name_map.get(joint_name)
                    if so101_name:
                        goal_positions[so101_name] = float(msg.position[i])
            
            if not goal_positions:
                return

            # Apply safety limits
            if self.max_relative_target > 0:
                try:
                    arm_motors = ["shoulder_pan", "shoulder_lift", "elbow_flex", 
                                 "wrist_flex", "wrist_roll", "gripper"]
                    present_positions = self.bus.sync_read("Present_Position", arm_motors, normalize=True)
                    
                    for so101_name, goal_pos in goal_positions.items():
                        if so101_name in present_positions:
                            present_pos = present_positions[so101_name]
                            diff = abs(goal_pos - present_pos)
                            if diff > self.max_relative_target:
                                if goal_pos > present_pos:
                                    goal_positions[so101_name] = present_pos + self.max_relative_target
                                else:
                                    goal_positions[so101_name] = present_pos - self.max_relative_target
                except Exception as e:
                    self.get_logger().warn(f'Could not apply safety limits: {e}')

            # Send command to puppet arm
            self.bus.sync_write("Goal_Position", goal_positions)
            self.current_arm_joint_states = goal_positions.copy()
                    
        except Exception as e:
            self.get_logger().error(f'Error sending command to puppet arm: {str(e)}')

    def publish_arm_states(self):
        """Publish current puppet arm joint states."""
        if not self.bus.is_connected:
            return

        try:
            arm_motors = ["shoulder_pan", "shoulder_lift", "elbow_flex", 
                         "wrist_flex", "wrist_roll", "gripper"]
            positions = self.bus.sync_read("Present_Position", arm_motors, normalize=True)

            self.current_arm_joint_states = positions.copy()

            # Map SO101 names to our names
            joint_names = []
            joint_positions = []
            reverse_map = {v: k for k, v in self.joint_name_map.items()}
            
            for so101_name, position in positions.items():
                our_name = reverse_map.get(so101_name)
                if our_name:
                    joint_names.append(our_name)
                    joint_positions.append(float(position))

            if not joint_positions:
                return

            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = joint_names
            joint_state_msg.position = joint_positions
            joint_state_msg.velocity = [0.0] * len(joint_positions)
            joint_state_msg.effort = [0.0] * len(joint_positions)
            
            self.puppet_arm_joint_state_pub.publish(joint_state_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing arm joint states: {str(e)}')

    def publish_wheel_states(self):
        """Publish current wheel joint states."""
        if not self.bus.is_connected:
            return

        try:
            positions = self.bus.sync_read("Present_Position", self.wheel_names, normalize=False)
            velocities = self.bus.sync_read("Present_Velocity", self.wheel_names, normalize=False)

            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = [f"{wheel}_joint" for wheel in self.wheel_names]
            joint_state_msg.position = [
                self.raw_pos_to_rad(positions[wheel]) for wheel in self.wheel_names
            ]
            joint_state_msg.velocity = [
                self.raw_vel_to_rad_s(velocities[wheel]) for wheel in self.wheel_names
            ]
            joint_state_msg.effort = [0.0] * len(self.wheel_names)
            
            self.wheel_joint_state_pub.publish(joint_state_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing wheel joint states: {str(e)}')

    def raw_pos_to_rad(self, raw: int) -> float:
        """Convert raw position to radians."""
        deg = raw / (self.ticks_per_rev / 360.0)
        return deg * math.pi / 180.0

    def raw_vel_to_rad_s(self, raw: int) -> float:
        """Convert raw velocity to rad/s."""
        deg_s = raw / (self.ticks_per_rev / 360.0)
        return deg_s * math.pi / 180.0

    def on_shutdown(self):
        """Cleanup on shutdown."""
        if self.bus.is_connected:
            try:
                # Disable torque for all motors
                self.bus.disable_torque()
                self.bus.disconnect(disable_torque=False)  # Already disabled above
                self.get_logger().info('Servo bus disconnected')
            except Exception as e:
                self.get_logger().error(f'Error disconnecting servos: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = KiwiMobilePuppetNode()

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

