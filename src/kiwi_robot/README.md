# Kiwi Robot - Clean SO101 Arm and Base Implementation

A clean ROS2 implementation for controlling the SO101 arm in puppeteer/puppet mode and the Kiwi base with mecanum wheels.

## Package Structure

This package contains four main nodes:

### 1. `so101_puppeteer`
The puppeteer (leader/teleop) node that reads joint positions from a SO101 arm via serial communication and publishes them as `JointState` messages.

**Topics:**
- Publishes: `/{arm_id}/joint_states` (sensor_msgs/JointState)

**Parameters:**
- `port` (string, default: '/dev/ttyACM0'): Serial port for the SO101 arm
- `baudrate` (int, default: 115200): Serial communication baudrate
- `publish_rate` (float, default: 50.0): Joint state publishing rate in Hz
- `arm_id` (string, default: 'puppeteer_arm'): Namespace identifier for the arm
- `calibration_file` (string, default: ''): Path to calibration JSON file

### 2. `so101_puppet`
The puppet (follower) node that subscribes to joint states from the puppeteer and replicates the movements by sending commands to the puppet SO101 arm.

**Topics:**
- Subscribes: `/{puppeteer_arm_id}/joint_states` (sensor_msgs/JointState)
- Publishes: `/{arm_id}/joint_states` (sensor_msgs/JointState)

**Services:**
- `/{arm_id}/set_teleop` (std_srvs/SetBool): Enable/disable teleoperation

**Parameters:**
- `port` (string, default: '/dev/ttyACM1'): Serial port for the SO101 arm
- `baudrate` (int, default: 115200): Serial communication baudrate
- `arm_id` (string, default: 'puppet_arm'): Namespace identifier for the arm
- `puppeteer_arm_id` (string, default: 'puppeteer_arm'): ID of the puppeteer arm to follow
- `max_relative_target` (float, default: 20.0): Maximum relative movement per command in degrees
- `calibration_file` (string, default: ''): Path to calibration JSON file

### 3. `kiwi_base`
Node that converts joystick input from the joy node to mecanum wheel velocities.

**Topics:**
- Subscribes: `/joy` (sensor_msgs/Joy)
- Publishes: `/cmd_vel` (geometry_msgs/Twist)

**Parameters:**
- `max_linear_speed` (float, default: 1.0): Maximum linear speed in m/s
- `max_angular_speed` (float, default: 2.0): Maximum angular speed in rad/s
- `axis_linear_x` (int, default: 1): Joystick axis for forward/backward
- `axis_linear_y` (int, default: 0): Joystick axis for strafe left/right
- `axis_angular_z` (int, default: 3): Joystick axis for rotation
- `wheel_separation_x` (float, default: 0.3): Distance between left/right wheels in meters
- `wheel_separation_y` (float, default: 0.3): Distance between front/back wheels in meters
- `wheel_radius` (float, default: 0.05): Wheel radius in meters

### 4. `kiwi_cam`
Node that publishes camera images from webcams as ROS Image messages.

**Topics:**
- Publishes: `/{camera_name}/image_raw` (sensor_msgs/Image) for each camera

**Parameters:**
- `camera_ids` (list, default: [0]): List of camera device IDs
- `frame_rate` (float, default: 30.0): Frame rate in FPS
- `camera_names` (list, default: ['camera']): List of camera names for topic namespaces
- `width` (int, default: 640): Camera image width
- `height` (int, default: 480): Camera image height

## Building

```bash
cd /home/kiwi/ros2_ws
colcon build --packages-select kiwi_robot
source install/setup.bash
```

## Calibration

Before using the SO101 arms, they need to be calibrated. Run the calibration node once for each arm (puppeteer and puppet).

### Calibrate Puppeteer Arm

```bash
ros2 launch kiwi_robot so101_calibrate.launch.py \
    port:=/dev/ttyACM0 \
    arm_id:=puppeteer_arm \
    output_file:=calibration/puppeteer_arm.json
```

### Calibrate Puppet Arm

```bash
ros2 launch kiwi_robot so101_calibrate.launch.py \
    port:=/dev/ttyACM1 \
    arm_id:=puppet_arm \
    output_file:=calibration/puppet_arm.json
```

Or run directly:
```bash
ros2 run kiwi_robot so101_calibrate --ros-args \
    -p port:=/dev/ttyACM0 \
    -p arm_id:=puppeteer_arm \
    -p output_file:=calibration/puppeteer_arm.json
```

**Calibration Procedure:**
1. The node will connect to the arm and disable torque
2. You'll be prompted to move the arm to the middle of its range of motion
3. Homing offsets will be recorded
4. You'll be prompted to move all joints through their full ranges of motion
5. Min/max ranges will be recorded
6. Calibration data will be written to the motors and saved to a JSON file

**Note:** The calibration node uses placeholder serial communication functions. You'll need to adapt the `disable_torque()`, `set_position_mode()`, `read_joint_position()`, and `write_calibration()` methods in `so101_calibrate.py` to match your actual SO101 hardware protocol.

## Usage

### Launch Individual Nodes

**Puppeteer:**
```bash
ros2 launch kiwi_robot so101_puppeteer.launch.py \
    port:=/dev/ttyACM0 \
    calibration_file:=calibration/puppeteer_arm.json
```

**Puppet:**
```bash
ros2 launch kiwi_robot so101_puppet.launch.py \
    port:=/dev/ttyACM1 \
    calibration_file:=calibration/puppet_arm.json
```

**Base:**
```bash
ros2 launch kiwi_robot kiwi_base.launch.py
```

**Camera:**
```bash
ros2 launch kiwi_robot kiwi_cam.launch.py camera_ids:="[0,1]"
```

### Launch All Nodes

```bash
ros2 launch kiwi_robot kiwi_robot.launch.py
```

## Notes

- The SO101 arm communication protocol is a placeholder implementation. You may need to adjust the serial communication protocol in `so101_puppeteer.py` and `so101_puppet.py` based on your actual SO101 hardware protocol.

- The mecanum wheel kinematics in `kiwi_base.py` calculates velocities for 4 wheels. If your base has only 3 wheels, you'll need to adjust the kinematics accordingly.

- Camera support requires OpenCV and cv_bridge ROS package to be installed.

## Dependencies

- ROS2 Humble
- rclpy
- sensor_msgs
- geometry_msgs
- std_msgs
- std_srvs
- cv_bridge (ROS package)
- pyserial (Python package)
- opencv-python (Python package)
- numpy (Python package)
- scservo_sdk (Python package) - Feetech servo SDK for SO101 arm communication

**Note:** This package includes a standalone implementation of SO101 communication functionality copied from lerobot, so the lerobot library is NOT required as a dependency.

