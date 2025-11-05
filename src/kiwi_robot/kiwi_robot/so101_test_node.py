#!/usr/bin/env python3

"""
SO101 Test Node
Simple test node to read positions from SO101 arm via serial communication
and publish them as a topic. Used for debugging serial communication issues.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

# Import SO101 communication components
from kiwi_robot.so101 import Motor, MotorCalibration, MotorNormMode, FeetechMotorsBus, OperatingMode


class SO101TestNode(Node):
    """
    Simple test node for SO101 arm serial communication.
    Reads positions and publishes them to verify communication is working.
    """

    def __init__(self):
        super().__init__('so101_test_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('publish_rate', 10.0)  # Hz - start slow for debugging
        self.declare_parameter('use_degrees', True)
        self.declare_parameter('use_normalized', False)  # Read raw values first

        # Handle string-to-number conversion from launch files
        port_val = self.get_parameter('port').value
        self.port = str(port_val) if port_val is not None else '/dev/ttyACM0'
        
        publish_rate_val = self.get_parameter('publish_rate').value
        self.publish_rate = float(publish_rate_val) if publish_rate_val is not None else 10.0
        
        use_degrees_val = self.get_parameter('use_degrees').value
        self.use_degrees = bool(use_degrees_val) if use_degrees_val is not None else True
        
        use_normalized_val = self.get_parameter('use_normalized').value
        self.use_normalized = bool(use_normalized_val) if use_normalized_val is not None else False

        # Setup motor normalization mode
        norm_mode_body = MotorNormMode.DEGREES if self.use_degrees else MotorNormMode.RANGE_M100_100

        # Initialize the SO-101 arm bus using FeetechMotorsBus
        self.get_logger().info(f'Initializing SO-101 arm on port {self.port}...')
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
            calibration=None,  # No calibration for raw reading
        )

        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(
            JointState,
            'so101_test/joint_states',
            10
        )

        # Statistics
        self.read_count = 0
        self.error_count = 0
        self.last_positions = {}

        # Initialize the SO-101 arm connection
        self.init_arm()

        # Timer for reading and publishing joint states
        self.timer = self.create_timer(
            1.0 / float(self.publish_rate),
            self.read_and_publish_positions
        )

        self.get_logger().info(
            f'SO-101 Test Node initialized on port {self.port}'
        )
        self.get_logger().info(
            f'Publishing at {self.publish_rate} Hz, normalized={self.use_normalized}'
        )

    def init_arm(self):
        """Initialize the SO-101 arm connection."""
        try:
            self.get_logger().info('Connecting to arm...')
            self.bus.connect(handshake=True)
            self.get_logger().info('Connection successful!')
            
            # Configure motors and DISABLE torque so arm can be moved manually
            self.get_logger().info('Configuring motors and disabling torque...')
            self.bus.disable_torque()  # Disable torque first
            self.bus.configure_motors()
            
            # Set operating mode but keep torque disabled
            for motor in self.bus.motors:
                self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            
            # Ensure torque stays disabled
            self.bus.disable_torque()
            
            self.get_logger().info(
                f'Successfully connected to SO-101 arm on {self.port}'
            )
            self.get_logger().info(
                '⚠️  TORQUE IS DISABLED - You can now manually move the arm'
            )
            
            # Do an initial test read
            self.get_logger().info('Performing initial position read test...')
            self.test_read_positions()
            
        except Exception as e:
            self.get_logger().error(
                f'Failed to initialize SO-101 arm: {str(e)}'
            )
            raise

    def test_read_positions(self):
        """Test reading positions from all motors individually."""
        self.get_logger().info('='*60)
        self.get_logger().info('Testing individual motor position reads:')
        self.get_logger().info('='*60)
        
        for motor_name in self.bus.motors:
            try:
                # Read raw position
                raw_pos = self.bus.read("Present_Position", motor_name, normalize=False)
                self.get_logger().info(
                    f'  {motor_name:15s} (ID {self.bus.motors[motor_name].id:2d}): '
                    f'raw = {raw_pos:5d}'
                )
                
                # Try reading again to see if it updates
                time.sleep(0.1)
                raw_pos2 = self.bus.read("Present_Position", motor_name, normalize=False)
                if raw_pos2 != raw_pos:
                    self.get_logger().info(
                        f'                    Second read: raw = {raw_pos2:5d} (changed!)'
                    )
                else:
                    self.get_logger().warn(
                        f'                    Second read: raw = {raw_pos2:5d} (unchanged - move joint to test)'
                    )
                    
            except Exception as e:
                self.get_logger().error(f'  {motor_name}: Failed to read - {e}')
        
        self.get_logger().info('='*60)

    def read_and_publish_positions(self):
        """Read joint positions and publish them as JointState message."""
        if not self.bus.is_connected:
            self.get_logger().warn('Arm not connected!')
            return

        self.read_count += 1
        
        try:
            # Read current joint positions
            # Try both methods: individual reads and sync_read
            positions = {}
            
            # Method 1: Individual reads (more reliable for debugging)
            self.get_logger().debug('Reading positions individually...')
            for motor_name in self.bus.motors:
                try:
                    pos = self.bus.read(
                        "Present_Position", 
                        motor_name, 
                        normalize=self.use_normalized
                    )
                    positions[motor_name] = float(pos)
                except Exception as e:
                    self.get_logger().error(f'Failed to read {motor_name}: {e}')
                    self.error_count += 1
                    # Use last known position if available
                    positions[motor_name] = self.last_positions.get(motor_name, 0.0)
            
            # Check if positions changed
            positions_changed = False
            for motor_name, pos in positions.items():
                if motor_name in self.last_positions:
                    if abs(pos - self.last_positions[motor_name]) > 1.0:  # Changed by more than 1 unit
                        positions_changed = True
                        self.get_logger().info(
                            f'Position changed: {motor_name} = {self.last_positions[motor_name]:.2f} -> {pos:.2f}'
                        )
            
            self.last_positions = positions.copy()
            
            # Log every 50 reads (5 seconds at 10Hz) or when positions change
            if self.read_count % 50 == 0 or positions_changed:
                self.get_logger().info(
                    f'Read #{self.read_count}: Errors={self.error_count}, '
                    f'Positions changed: {positions_changed}'
                )
                if positions:
                    pos_str = ', '.join([f'{k}={v:.1f}' for k, v in list(positions.items())[:3]])
                    self.get_logger().info(f'  Positions: {pos_str}...')

            # Create and publish JointState message
            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = list(positions.keys())
            joint_state_msg.position = list(positions.values())
            joint_state_msg.velocity = [0.0] * len(positions)
            joint_state_msg.effort = [0.0] * len(positions)
            
            self.joint_state_pub.publish(joint_state_msg)
            
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(
                f'Error reading/publishing positions (#{self.read_count}): {str(e)}'
            )

    def on_shutdown(self):
        """Cleanup on shutdown."""
        if self.bus.is_connected:
            try:
                # Ensure torque stays disabled when shutting down
                self.bus.disable_torque()
                self.bus.disconnect(disable_torque=False)  # Already disabled, don't disable again
                self.get_logger().info('SO-101 test arm disconnected')
            except Exception as e:
                self.get_logger().error(f'Error disconnecting arm: {e}')
        
        self.get_logger().info(
            f'Shutdown: Total reads={self.read_count}, Errors={self.error_count}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SO101TestNode()

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


