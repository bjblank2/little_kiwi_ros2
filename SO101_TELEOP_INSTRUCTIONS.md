# SO-101 Teleoperation Instructions

This guide explains how to run your SO-101 arms in teleoperation mode (puppeteer/puppet).

## Overview

- **Puppeteer Arm (Leader)**: The arm you manually move. It reads joint positions and publishes them.
- **Puppet Arm (Follower)**: The arm that automatically follows the puppeteer arm's movements.

## Prerequisites

1. Both arms should be calibrated:
   - `puppeteer_arm_calibration.json`
   - `puppet_arm_calibration.json`

2. Know your serial ports:
   - Check with: `ls /dev/ttyACM*`
   - Default: puppeteer on `/dev/ttyACM0`, puppet on `/dev/ttyACM1`

## Running the Arms

### Option 1: Launch Both Together (Recommended)

Launch both arms simultaneously:

```bash
ros2 launch kiwi_robot so101_puppeteer.launch.py \
    port:=/dev/ttyACM0 \
    calibration_file:=puppeteer_arm_calibration.json &

ros2 launch kiwi_robot so101_puppet.launch.py \
    port:=/dev/ttyACM1 \
    calibration_file:=puppet_arm_calibration.json
```

Or use the main launch file (if configured):

```bash
ros2 launch kiwi_robot kiwi_robot.launch.py \
    start_puppeteer:=true \
    start_puppet:=true
```

### Option 2: Run Separately in Different Terminals

**Terminal 1 - Puppeteer (Leader Arm):**
```bash
ros2 launch kiwi_robot so101_puppeteer.launch.py \
    port:=/dev/ttyACM0 \
    calibration_file:=puppeteer_arm_calibration.json \
    publish_rate:=50.0 \
    arm_id:=puppeteer_arm
```

**Terminal 2 - Puppet (Follower Arm):**
```bash
ros2 launch kiwi_robot so101_puppet.launch.py \
    port:=/dev/ttyACM1 \
    calibration_file:=puppet_arm_calibration.json \
    arm_id:=puppet_arm \
    puppeteer_arm_id:=puppeteer_arm \
    max_relative_target:=20.0
```

### Option 3: Run Directly (Without Launch Files)

**Puppeteer:**
```bash
ros2 run kiwi_robot so101_puppeteer \
    --ros-args \
    -p port:=/dev/ttyACM0 \
    -p calibration_file:=puppeteer_arm_calibration.json \
    -p publish_rate:=50.0 \
    -p arm_id:=puppeteer_arm
```

**Puppet:**
```bash
ros2 run kiwi_robot so101_puppet \
    --ros-args \
    -p port:=/dev/ttyACM1 \
    -p calibration_file:=puppet_arm_calibration.json \
    -p arm_id:=puppet_arm \
    -p puppeteer_arm_id:=puppeteer_arm \
    -p max_relative_target:=20.0
```

## Quick Start (Default Ports)

If your arms are connected to the default ports (`/dev/ttyACM0` and `/dev/ttyACM1`):

```bash
# Terminal 1
ros2 launch kiwi_robot so101_puppeteer.launch.py \
    calibration_file:=puppeteer_arm_calibration.json

# Terminal 2  
ros2 launch kiwi_robot so101_puppet.launch.py \
    calibration_file:=puppet_arm_calibration.json
```

## Parameters

### Puppeteer Node Parameters:
- `port`: Serial port (default: `/dev/ttyACM0`)
- `publish_rate`: Publishing frequency in Hz (default: `50.0`)
- `arm_id`: Namespace for the arm (default: `puppeteer_arm`)
- `calibration_file`: Path to calibration JSON file

### Puppet Node Parameters:
- `port`: Serial port (default: `/dev/ttyACM1`)
- `arm_id`: Namespace for this arm (default: `puppet_arm`)
- `puppeteer_arm_id`: Name of the puppeteer arm to follow (default: `puppeteer_arm`)
- `max_relative_target`: Maximum relative movement per step in degrees (default: `20.0`)
- `calibration_file`: Path to calibration JSON file

## How It Works

1. **Puppeteer arm** reads joint positions from the SO-101 servos and publishes them on topic:
   - `/puppeteer_arm/joint_states` (or your custom `arm_id`)

2. **Puppet arm** subscribes to the puppeteer's joint states and replicates the movements.

3. Move the puppeteer arm manually, and the puppet arm should follow!

## Troubleshooting

1. **Port not found**: Check which ports your arms are connected to:
   ```bash
   ls -l /dev/ttyACM*
   ```

2. **Arm not responding**: Make sure the calibration files are correct and the arms are powered on.

3. **Puppet not following**: 
   - Check that puppeteer is publishing: `ros2 topic echo /puppeteer_arm/joint_states`
   - Verify puppet is subscribed: `ros2 topic info /puppeteer_arm/joint_states`

4. **Wrong port order**: If your arms are swapped, just swap the port numbers in the launch commands.

## Stopping the Nodes

Press `Ctrl+C` in each terminal, or kill the processes:
```bash
pkill -f so101_puppeteer
pkill -f so101_puppet
```

