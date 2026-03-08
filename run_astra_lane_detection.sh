#!/bin/bash

echo "================================================"
echo "Starting Astra Pro Plus Lane Detection System"
echo "================================================"

# Check if user is in video group (for current session)
if groups | grep -q video; then
    echo "✓ User has video group permissions"
else
    echo "⚠ Video group not active in current session"
    echo "Using 'sg video' to run with video permissions..."
    echo ""
    # Re-run this script with video group permissions
    exec sg video "$0"
fi

# Source ROS2 installation
source /opt/ros/humble/setup.bash

# Source workspace overlays
<<<<<<< HEAD
source ~/depth_cam/install/setup.bash
=======
source ~/lane_following/install/setup.bash
>>>>>>> f3c6d1fd (okay :<)

# Check if camera is connected
echo ""
echo "Checking for Astra Pro Plus camera..."
if lsusb | grep -q "2bc5"; then
    echo "✓ Astra Pro Plus camera detected!"
else
    echo "⚠ Warning: Camera not detected. Make sure it's plugged in."
    echo "Looking for USB device with vendor ID 2bc5 (Orbbec)"
fi

echo ""
echo "Starting camera node in background..."
# Start Orbbec camera node in background
ros2 launch orbbec_camera astra_pro_plus.launch.py &
CAMERA_PID=$!

# Wait for camera to initialize
echo "Waiting for camera to initialize..."
sleep 5

echo ""
echo "Checking camera topics..."
ros2 topic list | grep camera

echo ""
echo "Starting lane detection node..."
echo "Controls:"
echo "  q - Quit"
echo "  s - Start/Stop recording"
echo "================================================"
echo ""

# Make the Python script executable
<<<<<<< HEAD
chmod +x ~/depth_cam/src/Lane-Detection/LaneDetect_AstraCam.py

# Run the lane detection node
cd ~/depth_cam/src/Lane-Detection
=======
chmod +x ~/lane_following/src/lane_detection/LaneDetect_AstraCam.py

# Run the lane detection node
cd ~/lane_following/src/lane_detection
>>>>>>> f3c6d1fd (okay :<)
python3 LaneDetect_AstraCam.py

# Cleanup: kill camera node when done
echo ""
echo "Shutting down camera node..."
kill $CAMERA_PID 2>/dev/null

echo "Lane detection stopped."
