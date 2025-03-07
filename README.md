 # Overview
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

## Usage 
Before running anything, make sure the can interface is working/

```bash
sudo ip link set can0 up type can bitrate 250000
```

### Running Containers

Run the latest container by calling this command from the root directory. 

Note: The containers are run with interactive mode (-it flag) which means:
- You'll get an interactive terminal session
- The container runs in the foreground
- Automatic volume mounting of your workspace
```bash
nt ctn run
```
Note: the command mounts the directory your are calling it from. 
So if you call it from ~ then the whole ~/ directory is mounted on the container. 
You want to run it from the root directory. 

Once the container runs, make sure to run this command to make the nt(newton keyword accessible in the shell)
```bash
source setup.sh
```

### Building Packages

```bash
nt pkg build
```
This command will build all ros packages.

### Sourcing Packages
Once packages are built, they need to be source  with 

```bash
source src/install/setup.bash
```

### Launching Packages
When the packages are source you can run using 

```bash
ros2 launch n_motor_controller motors.launch.py
```
This will initiliaze as many nodes they are in the `config/newton.yaml` file and start the motor controller. 
### Clean Packages
Within the container you can this command remove old build files. 

```bash
nt pkg clean
```


### Stoping the container
To stop and exit the container:
- Press `Ctrl + D` in the container shell


## Package Verification

After building and sourcing packages, you can verify installation with:
```bash
ros2 pkg list | grep odrive_can
ros2 pkg list | grep odrive_motor
```