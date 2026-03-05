#!/bin/bash
# Script to launch camera and lane detection parameter tuner

set -e

echo "Sourcing ROS2 workspace..."
source /opt/ros/humble/setup.bash
source /home/chetan-satpute/depth_cam/install/setup.bash

echo "Starting Astra Pro Plus camera..."
ros2 launch orbbec_camera astra_pro_plus.launch.py &
CAMERA_PID=$!

sleep 5

echo ""
echo "Starting Lane Parameter Tuner..."
echo "Use the trackbars to adjust parameters until lanes are detected"
echo "Press 'q' when done to see optimal parameters"
echo ""

python3 /home/chetan-satpute/depth_cam/src/Lane-Detection/LaneDetect_Tuner.py

# Clean up
echo "Shutting down camera..."
kill $CAMERA_PID 2>/dev/null
wait $CAMERA_PID 2>/dev/null

echo "Done!"
