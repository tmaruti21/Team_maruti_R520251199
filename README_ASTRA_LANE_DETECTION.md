# Lane Detection from Astra Pro Plus Depth Camera

This guide helps you detect lanes from your Astra Pro Plus depth camera connected to your laptop.

## Quick Start

### Step 1: Test Your Camera

First, make sure your camera is working:

```bash
cd ~/depth_cam
./test_astra_camera.sh
```

This will:
- Check if camera is connected
- Start the camera node
- Show available topics
- Display image stream rate

**Expected output**: You should see `/camera/color/image_raw` publishing at ~30 Hz

Press `Ctrl+C` to stop the test.

### Step 2: Run Lane Detection

Once camera is working, run the lane detection:

```bash
cd ~/depth_cam
./run_astra_lane_detection.sh
```

A window will open showing:
- Real-time video from camera
- Detected lane lines (green)
- Lane area filled (yellow/cyan)

**Controls:**
- `q` - Quit
- `s` - Start/Stop recording (saves to output_astra.avi)

## Troubleshooting

### Camera Not Detected

If you see "Camera not found":

1. **Check USB connection:**
   ```bash
   lsusb | grep 2bc5
   ```
   Should show: `2bc5:.... Orbbec ...`

2. **Check permissions:**
   ```bash
   groups | grep video
   ```
   Should show `video` in your groups. If not:
   ```bash
   sudo usermod -aG video $USER
   # Log out and log back in
   ```

3. **Try different USB port:**
   - Use USB 3.0 port (blue)
   - Avoid USB hubs

### No Image Topics

If `/camera/color/image_raw` doesn't appear:

1. **Wait longer** - Camera needs 5-10 seconds to initialize

2. **Check logs:**
   ```bash
   ros2 launch orbbec_camera astra_pro_plus.launch.py
   ```
   Look for errors in output

3. **Restart camera:**
   - Unplug camera
   - Wait 5 seconds
   - Plug back in

### Lane Detection Not Working

If lanes aren't detected:

1. **Check lighting** - Need good contrast between lane and floor

2. **Adjust ROI** - Edit the region of interest in the code:
   ```python
   # In LaneDetect_AstraCam.py, line ~63
   vertices = np.array([[(0,imshape[0]),
                         (int(imshape[1]*0.45), int(imshape[0]*0.6)), 
                         (int(imshape[1]*0.55), int(imshape[0]*0.6)), 
                         (imshape[1],imshape[0])]], dtype=np.int32)
   ```

3. **Tune parameters:**
   - `minVal`, `maxVal` (line ~56): Canny edge detection thresholds
   - `threshold` (line ~71): Hough line detection threshold
   - `min_line_len` (line ~72): Minimum line length

### Black Lanes Not Detected

The current code works for **white/light lanes**. For black lanes:

1. Add image inversion after grayscale conversion:
   ```python
   grayIm = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
   grayIm = cv2.bitwise_not(grayIm)  # Add this line
   ```

2. Apply CLAHE for better contrast:
   ```python
   clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
   grayIm = clahe.apply(grayIm)
   ```

## Files

- `LaneDetect_AstraCam.py` - Main ROS2 lane detection node
- `run_astra_lane_detection.sh` - Launch script (camera + detection)
- `test_astra_camera.sh` - Camera test utility
- `README_ASTRA_LANE_DETECTION.md` - This file

## Manual Running

If scripts don't work, run manually:

### Terminal 1: Start Camera
```bash
source /opt/ros/humble/setup.bash
source ~/depth_cam/install/setup.bash
ros2 launch orbbec_camera astra_pro_plus.launch.py
```

### Terminal 2: Run Lane Detection
```bash
cd ~/depth_cam/src/Lane-Detection
python3 LaneDetect_AstraCam.py
```

## Camera Topics Available

- `/camera/color/image_raw` - RGB image (used by lane detection)
- `/camera/depth/image_raw` - Depth image
- `/camera/ir/image_raw` - Infrared image
- `/camera/color/camera_info` - Camera calibration info

## Next Steps

To add more advanced features:
- Sharp turn detection
- State machine (forward/backward/left/right)
- Motor control integration
- Depth-based obstacle avoidance

Let me know what feature you'd like to add!
