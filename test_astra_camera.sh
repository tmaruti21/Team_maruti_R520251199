#!/bin/bash

echo "======================================"
echo "Testing Astra Pro Plus Camera"
echo "======================================"

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/depth_cam/install/setup.bash

echo ""
echo "1. Checking USB connection..."
if lsusb | grep -q "2bc5"; then
    echo "✓ Astra Pro Plus detected on USB"
    lsusb | grep "2bc5"
else
    echo "✗ Camera not found!"
    echo "Please check:"
    echo "  - Camera is plugged in"
    echo "  - USB cable is good"
    echo "  - Run: lsusb to see all devices"
    exit 1
fi

echo ""
echo "2. Starting camera node..."
ros2 launch orbbec_camera astra_pro_plus.launch.py &
CAMERA_PID=$!

echo "Waiting 7 seconds for camera initialization..."
sleep 7

echo ""
echo "3. Available camera topics:"
ros2 topic list | grep camera

echo ""
echo "4. Camera info:"
timeout 2 ros2 topic echo /camera/color/camera_info --once 2>/dev/null || echo "No camera info available yet"

echo ""
echo "5. Checking image stream..."
echo "Listening to /camera/color/image_raw for 2 seconds..."
timeout 2 ros2 topic hz /camera/color/image_raw 2>/dev/null || echo "No images detected"

echo ""
echo "======================================"
echo "Test complete! Press Ctrl+C to stop camera node"
echo "======================================"

# Keep running until user stops
wait $CAMERA_PID
