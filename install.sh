#!/bin/bash
# PIVOT Planner - Quick Installation Script for ROS2 Jazzy

set -e

echo "=================================="
echo "PIVOT Planner Installation"
echo "=================================="

# Check if ROS2 is installed
if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "ERROR: ROS2 Jazzy not found at /opt/ros/jazzy/"
    echo "Please install ROS2 Jazzy first."
    exit 1
fi

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Check if workspace exists, create if not
if [ -z "$1" ]; then
    WORKSPACE="$HOME/ros2_ws"
    echo "No workspace specified, using: $WORKSPACE"
else
    WORKSPACE="$1"
    echo "Using workspace: $WORKSPACE"
fi

# Create workspace structure
mkdir -p "$WORKSPACE/src"

# Copy package
echo "Copying PIVOT planner package to $WORKSPACE/src/..."
cp -r "$(dirname "$0")" "$WORKSPACE/src/pivot_planner"

# Navigate to workspace
cd "$WORKSPACE"

echo "Building package..."
colcon build --packages-select pivot_planner --symlink-install

if [ $? -eq 0 ]; then
    echo ""
    echo "=================================="
    echo "Installation successful!"
    echo "=================================="
    echo ""
    echo "To use the package, run:"
    echo "  source $WORKSPACE/install/setup.bash"
    echo "  ros2 launch pivot_planner pivot_planner.launch.py"
    echo ""
    echo "To customize parameters, edit:"
    echo "  $WORKSPACE/install/pivot_planner/share/pivot_planner/config/pivot_params.yaml"
    echo ""
else
    echo "Build failed. Please check error messages above."
    exit 1
fi
