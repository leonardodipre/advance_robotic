#!/bin/bash

# Absolute path to Conda installation
CONDA_PATH="/root/miniconda3"

# Initialize Conda in the script
source "${CONDA_PATH}/etc/profile.d/conda.sh"

# Activate the desired Conda environment
conda activate ros_yolo_env

# Source ROS setup within the Conda environment
source /opt/ros/noetic/setup.bash
source ~/your_ros_workspace/devel/setup.bash

# Launch the ROS node
exec rosrun arm_controllers torch_node "$@"
