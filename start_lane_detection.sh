#!/bin/bash
# Script to launch Astra Pro Plus camera and lane detection

# Exit on any error
set -e

# Source ROS2 and the workspace
echo "Sourcing ROS2 workspace..."
source /opt/ros/humble/setup.bash
source /home/chetan-satpute/depth_cam/install/setup.bash

# Start the Astra Pro Plus camera in the background
echo "Starting Astra Pro Plus camera..."
ros2 launch orbbec_camera astra_pro_plus.launch.py &
CAMERA_PID=$!

# Wait a few seconds for camera to initialize
echo "Waiting for camera to initialize..."
sleep 5

# Check if camera is running
if ! ps -p $CAMERA_PID > /dev/null; then
   echo "Error: Camera failed to start!"
   exit 1
fi

echo "Camera started successfully!"
echo ""
echo "Starting lane detection node..."
echo "Press 'q' in the image window to quit"
echo "Press 's' in the image window to start/stop recording"
echo ""

# Start lane detection
python3 /home/chetan-satpute/depth_cam/src/Lane-Detection/LaneDetect_ROS2_Realtime.py

# Clean up camera process when lane detection exits
echo "Shutting down camera..."
kill $CAMERA_PID
wait $CAMERA_PID 2>/dev/null

echo "Done!"
