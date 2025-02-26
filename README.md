# Newton Project
 This project creates a real-time control system that integrates machine learning with embedded processing for adaptive sensor and actuator control.

## Prerequisites
- Docker
- ROS2 Humble
- Python 3.10 or higher
- ODrive
- CAN BUS (refer to nvidia's guide)[https://docs.nvidia.com/jetson/archives/r35.4.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html]

## Installation

1. Clone the repository and navigate to the project directory using 
```bash
git clone --recurse-submodules https://github.com/TheNewtonCapstone/newton-jetson-software.git 
```

```bash
cd newton-jetson-software
```
2. Make the setup script executable and run it: 
```bash
chmod +x install.sh && ./install.sh
```

3. Make the setup script executable and run it:
```bash
source setup.sh
```
This will:
- Set up the newton command in `/usr/local/bin`
- Make the newton script executable
- Create necessary symbolic links

You must source once the container is running as well. 

## Project Structure
The project uses the following directory structure:
- `/core/src/ Contains ros packages`
  - `controller`
  - `imu`
  - `motor_driver`
  - `/imu/`
- `/lib/ros_odrive/` - Contains ODrive ROS2 packages
- `/lib/jetson-containers/` - Contains Jetson-specific container configurations
- `/modules/motor/` - Contains motor control modules


## Usage 
### Building Packages
```
# Build individual packages
newton build odrive_can     # Build ODrive CAN package
newton build motor_driver   # Build motor driver
newton build imu_driver     # Build IMU driver
newton build controller     # Build controller

# Build all packages
newton build all
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

### Sourcing Environments

environment must be source from Python
For this branch's purpose all we need to source is the motor
The motor package is build within core/srr. all you need is to source the motor driver

Source ROS2 environment:
```bash
source /opt/ros/humble/setup.bash
```

Source ODrive CAN package:
```bash
source core/ros_odrive/install/setup.bash 
```
Source motor_driver:
```bash
source core/
```

## Cleaning Build Files 
```
newton clean
```

## Build Dependencies
When building packages:
1. ODrive CAN package must be built before the motor package.

## Package Verification

After building packages, verify installation with:
```bash
ros2 pkg list | grep odrive_can
ros2 pkg list | grep odrive_motor
```

## Troubleshooting
If you encouter issue with Odrive devices, set up the udev rules:
```
sudo bash -c "curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger"
```