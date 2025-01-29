# jetson-firmware
 This project creates a real-time control system that integrates machine learning with embedded processing for adaptive sensor and actuator control.

## Prerequisites

- Docker
- ROS2 Humble
- Python 3.10 or higher
- ODrive hardware support
# Newton Project

This document provides instructions for setting up and using the Newton project.

## Prerequisites

- Docker
- ROS2 Humble
- Python 3.10 or higher
- ODrive
- CAN BUS (refer to nvidia's guide)[https://docs.nvidia.com/jetson/archives/r35.4.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html]

## Installation

1. Clone the repository and navigate to the project directory:
```bash
cd newton-jetson-software
```
2. Make the setup script executable and run it: 
```bash
chmod +x install.sh
./setup.sh
```

1. Make the setup script executable and run it:
```bash
chmod +x setup.sh
./setup.sh
```

This will:
- Set up the newton command in `/usr/local/bin`
- Make the newton script executable
- Create necessary symbolic links

## Project Structure

The project uses the following directory structure:
- `/lib/ros_odrive/` - Contains ODrive ROS2 packages
- `/lib/jetson-containers/` - Contains Jetson-specific container configurations
- `/modules/motor/` - Contains motor control modules


## Usage 
### Building Packages
Build ODrive CAN package:
```bash
newton build odrive_can
```

Build ODrive motor package:
```bash
newton build odrive_motor
```

### Running Containers

Run the latest container:
```bash
newton run latest
```

Run a specific container:
```bash
newton run <container_name>
```

To stop and exit the container:
- Press `Ctrl + D` or type `newton stop` in the container shell
- This will safely stop and exit the container session

Note: The containers are run with interactive mode (-it flag) which means:
- You'll get an interactive terminal session
- The container runs in the foreground
- Container includes features like:
  - Host network access for ROS2 communication
  - Full device access for hardware communication
  - Automatic volume mounting of your workspace

### Sourcing Environments

Source Python environment:
```bash
newton source python
```

Source ROS2 environment:
```bash
newton source ros
```

Source ODrive CAN package:
```bash
newton source odrive_can
```

## Build Dependencies

When building packages:
1. ODrive CAN package must be built before the motor package.
2. The system will automatically check and build dependencies if needed
3. ROS2 Humble environment will be sourced if not already done

## Package Verification

After building packages, verify installation with:
```bash
ros2 pkg list | grep odrive_can
ros2 pkg list | grep odrive_motor
```