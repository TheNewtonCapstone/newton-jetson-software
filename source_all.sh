#!/usr/bin/env bash

# get root git directory, by .git directory
ROOT_DIR="$(git rev-parse --show-toplevel)"

# source ROS top-level package & general packages
source "$ROOT_DIR/install/setup.bash"
source "$ROOT_DIR/src/install/setup.bash"