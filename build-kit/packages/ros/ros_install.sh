#!/bin/bash
sudo DOCKER_BUILDKIT=0 docker build --network=host --tag newton -f /home/galileo/workspaces/newton-jetson-software/build-kit/packages/ros/Dockerfile /home/galileo/workspaces/newton-jetson-software/build-kit/packages/ros
