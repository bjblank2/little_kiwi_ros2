#!/usr/bin/env python3

"""
Kiwi Base Node
This node listens to joy node messages and controls three mecanum wheels
on the base platform, converting joystick input to wheel velocities.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math


class KiwiBaseNode(Node):
    """
    ROS2 node for controlling Kiwi base with mecanum wheels.
    Converts joystick input to mecanum wheel velocities.
    """

    def __init__(self):
        super().__init__('kiwi_base')

        # Parameters for joystick mapping
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('axis_linear_x', 1)   # Left stick vertical (forward/back)
        self.declare_parameter('axis_linear_y', 0)   # Left stick horizontal (strafe left/right)
        self.declare_parameter('axis_angular_z', 3)   # Right stick horizontal (rotation)
        
        # Parameters for mecanum wheel geometry
        self.declare_parameter('wheel_separation_x', 0.3)  # Distance between left/right wheels
        self.declare_parameter('wheel_separation_y', 0.3)  # Distance between front/back wheels
        self.declare_parameter('wheel_radius', 0.05)      # Wheel radius in meters

        # Handle string-to-number conversion from launch files
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

        # Subscriber for joystick input
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Publisher for wheel velocities (using Twist for now, can be customized)
        # For mecanum wheels, we typically need individual wheel velocities
        # Here we publish twist, but you could create a custom message for wheel velocities
        self.twist_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Optional: Publish individual wheel velocities (would require custom message)
        # For now, using cmd_vel which is standard for differential/base controllers

        self.get_logger().info('Kiwi Base Node initialized')
        self.get_logger().info(f'Max linear speed: {self.max_linear_speed} m/s')
        self.get_logger().info(f'Max angular speed: {self.max_angular_speed} rad/s')

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
        # Mecanum wheel velocity calculation
        # Each wheel contributes to both linear and angular motion
        L = self.wheel_separation_x / 2.0  # Half of wheelbase width
        W = self.wheel_separation_y / 2.0  # Half of wheelbase length
        
        front_left = vx + vy + (L + W) * omega
        front_right = vx - vy - (L + W) * omega
        back_left = vx - vy + (L + W) * omega
        back_right = vx + vy - (L + W) * omega
        
        return [front_left, front_right, back_left, back_right]

    def joy_callback(self, msg):
        """Callback for joystick messages."""
        # Check if axes are available
        if (self.axis_linear_x >= len(msg.axes) or
            self.axis_linear_y >= len(msg.axes) or
            self.axis_angular_z >= len(msg.axes)):
            self.get_logger().warn_throttle(
                2.0,
                'Axis index out of range for received Joy message'
            )
            return

        # Get joystick values (typically -1.0 to 1.0)
        linear_x_raw = msg.axes[self.axis_linear_x] if self.axis_linear_x < len(msg.axes) else 0.0
        linear_y_raw = msg.axes[self.axis_linear_y] if self.axis_linear_y < len(msg.axes) else 0.0
        angular_z_raw = msg.axes[self.axis_angular_z] if self.axis_angular_z < len(msg.axes) else 0.0

        # Apply dead zone (small values near zero)
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

        # Calculate mecanum wheel velocities
        wheel_velocities = self.mecanum_kinematics(vx, vy, omega)

        # For now, publish as Twist message (standard cmd_vel)
        # A more advanced implementation would publish individual wheel velocities
        twist_msg = Twist()
        twist_msg.linear.x = vx
        twist_msg.linear.y = vy
        twist_msg.angular.z = omega

        self.twist_pub.publish(twist_msg)

        # Log wheel velocities for debugging
        self.get_logger().debug(
            f'Wheel velocities: FL={wheel_velocities[0]:.2f}, '
            f'FR={wheel_velocities[1]:.2f}, '
            f'BL={wheel_velocities[2]:.2f}, '
            f'BR={wheel_velocities[3]:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = KiwiBaseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
