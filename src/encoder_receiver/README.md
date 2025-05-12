# DJI Encoder Receiver Package

This ROS2 package reads encoder data from DJI motors via serial port and publishes odometry information.

## Overview

The package reads encoder data from DJI motors connected to a microcontroller that sends the data in a specific protocol format. The data is received via serial communication, processed, and published as ROS2 odometry messages.

## Dependencies

- ROS2 (tested on Foxy and Humble)
- serial package (`sudo apt install ros-$ROS_DISTRO-serial-driver`)

## Installation

1. Clone this repository into your ROS2 workspace:
   ```bash
   cd ~/your_workspace/src
   # If the package is not already there
   # git clone <repository-url>
   ```

2. Build the package:
   ```bash
   cd ~/your_workspace
   colcon build --packages-select encoder_receiver
   source install/setup.bash
   ```

## Usage

### Testing Serial Communication

Before running the ROS2 node, you can test the serial communication with the provided Python script:

```bash
cd ~/your_workspace
source install/setup.bash
python3 src/encoder_receiver/scripts/test_serial_protocol.py --port /dev/ttyACM2 --baud 115200
```

### Running the ROS2 Node

Launch the ROS2 node:

```bash
cd ~/your_workspace
source install/setup.bash
ros2 launch encoder_receiver dji_encoder_receiver.launch.py
```

### Launch Parameters

The following parameters can be customized in the launch file:

- `serial_port`: Serial port for DJI encoder data (default: `/dev/ttyACM2`)
- `baud_rate`: Baud rate for serial communication (default: `115200`)
- `wheel_radius`: Wheel radius in meters (default: `0.076`)
- `wheel_base`: Wheel base (distance between wheels) in meters (default: `0.4`)
- `encoder_counts_per_rev`: Encoder counts per revolution (default: `8192.0`)
- `publish_rate`: Odometry publish rate in Hz (default: `50.0`)

Example of launching with custom parameters:

```bash
ros2 launch encoder_receiver dji_encoder_receiver.launch.py serial_port:=/dev/ttyACM0 wheel_radius:=0.08
```

## Published Topics

- `/odom` (`nav_msgs/msg/Odometry`): Publishes odometry information based on encoder data

## TF Transforms

- The node broadcasts the transform from `odom` frame to `base_link` frame

## Protocol Details

The DJI encoder data protocol follows this format:

```
Header:
- SOF (1 byte): 0xA5
- Data Length (2 bytes): Length of the data section in little-endian format
- Sequence Number (1 byte): Incremental sequence number
- CRC8 (1 byte): Header checksum

Command ID (2 bytes): 0x0301 for chassis status

Data:
- Encoder Counts (8 bytes): Four 16-bit encoder values for the four wheels

CRC16 (2 bytes): Frame checksum
```

Example frame: `A5 08 00 51 63 01 03 94 16 69 00 51 19 EA 0E`

## License

TODO 