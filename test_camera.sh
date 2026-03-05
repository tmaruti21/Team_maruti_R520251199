#!/bin/bash
# Quick test script to check if camera is working

echo "Checking camera connection..."
source /opt/ros/humble/setup.bash
source /home/chetan-satpute/depth_cam/install/setup.bash

# Launch camera briefly and check for errors
timeout 10 ros2 launch orbbec_camera astra_pro_plus.launch.py &
CAMERA_PID=$!

echo "Waiting for camera to initialize..."
sleep 8

# Check if topics are publishing
echo ""
echo "Checking for camera topics:"
ros2 topic list | grep "/camera"

# Check if we're getting image data
echo ""
echo "Checking image data rate:"
timeout 3 ros2 topic hz /camera/color/image_raw 2>/dev/null || echo "No color image data yet"

# Cleanup
kill $CAMERA_PID 2>/dev/null
wait $CAMERA_PID 2>/dev/null

echo ""
echo "Test complete!"
