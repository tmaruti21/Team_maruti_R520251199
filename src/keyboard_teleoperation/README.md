# keyboard_teleoperation

ROS2 package for keyboard driving (`w/a/s/d`) of a differential-drive rover and bridging `/cmd_vel` to an ESP32 over serial.

## What this package does

- `keyboard_teleop` node
  - Reads keyboard keys:
    - `w`: forward
    - `s`: backward
    - `a`: left turn
    - `d`: right turn
    - `x` or `space`: stop
  - Publishes `geometry_msgs/Twist` on `/cmd_vel`.
- `esp32_cmd_bridge` node
  - Subscribes to `/cmd_vel`.
  - Converts Twist to left and right motor commands.
  - Sends serial packets to ESP32:
    - `CMD,<left_pwm>,<left_dir>,<right_pwm>,<right_dir>\n`

## Differential-drive mapping

All 3 motors on the left side should be wired to the same left driver channel.
All 3 motors on the right side should be wired to the same right driver channel.
This package sends one command per side (left/right).

## ESP32 pin mapping

The included firmware uses:

```cpp
#define PWM1 18
#define DIR1 19

#define PWM2 21
#define DIR2 22
```

Firmware path:

- `firmware/esp32_keyboard_bridge/esp32_keyboard_bridge.ino`

## Build

From workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select keyboard_teleoperation
source install/setup.bash
```

## Run

Terminal 1 (ESP32 bridge):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run keyboard_teleoperation esp32_cmd_bridge --ros-args -p serial_port:=/dev/ttyACM0 -p wheel_base_m:=0.24 -p max_linear_speed:=0.35
```

Terminal 2 (keyboard teleop):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run keyboard_teleoperation keyboard_teleop --ros-args -p linear_speed:=0.20 -p angular_speed:=1.20
```

Optional launch for bridge only:

```bash
ros2 launch keyboard_teleoperation esp32_bridge.launch.py serial_port:=/dev/ttyACM0
```

## Safety behavior

- Keyboard node has a key timeout. If no key is received, it publishes stop.
- Bridge node has command timeout. If `/cmd_vel` stops, it sends stop.
- ESP32 firmware has command timeout (`500 ms`). If serial commands stop, motors stop.
