#!/bin/bash
# shellcheck disable=SC1090,SC1091
# set -e
echo "Hello Docker!"

ROS_DISTRO=humble
# setup ros2 environment
source /opt/ros/"$ROS_DISTRO"/setup.bash --
source /opt/ros/humble/setup.bash

# add sourcing to .bashrc
echo "source '/opt/ros/$ROS_DISTRO/setup.bash'" >> ~/.bashrc

ROOT=$1
# Execute the command with stdin attached
if [ $# -gt 0 ]; then
  # Print the arguments
  echo "alias nt='python3 ~/$ROOT/newton.py'" >> ~/.bashrc
echo "export WORKSPACE_ROOT=/home/workspace/$ROOT" >> ~/.bashrch
fi
exec bash
