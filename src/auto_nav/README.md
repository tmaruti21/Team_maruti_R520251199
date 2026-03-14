# auto_nav

ROS2 package that bridges autonomous navigation to hardware while keeping keyboard teleoperation usable at the same time.

## What this package does

- Runs a dedicated `cmd_vel` multiplexer.
  - Auto input topic: `/cmd_vel_auto`
  - Manual input topic: `/cmd_vel_manual`
  - Selected output topic: `/cmd_vel_out`
- Gives manual commands priority for a short timeout window.
- Sends selected velocity commands to ESP32 using serial protocol:
  - `CMD,<left_pwm>,<left_dir>,<right_pwm>,<right_dir>\n`

## Why keyboard teleop is not hampered

- `custom_follow infer_node` is remapped from `/cmd_vel` to `/cmd_vel_auto`.
- `keyboard_teleoperation keyboard_teleop` can publish to `/cmd_vel_manual`.
- Mux chooses manual when keyboard input is active, otherwise auto.

## Build

```bash
cd /home/chetan-satpute/lane_following
source /opt/ros/humble/setup.bash
colcon build --packages-select auto_nav
source install/setup.bash
```

## Run full auto + bridge pipeline

```bash
cd /home/chetan-satpute/lane_following
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch auto_nav auto_nav_bringup.launch.py serial_port:=/dev/ttyACM0
```

## Optional keyboard override (second terminal)

```bash
cd /home/chetan-satpute/lane_following
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run keyboard_teleoperation keyboard_teleop --ros-args -p cmd_topic:=/cmd_vel_manual
```

## Notes

- If `/dev/ttyACM0` is wrong, find the correct port first:

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

- `manual_timeout_s` and `auto_timeout_s` in launch control source switching behavior.
