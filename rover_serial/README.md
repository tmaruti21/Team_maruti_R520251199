# rover_serial

Serial bridge for ESP32 rover:
- Publishes /imu/data_raw (sensor_msgs/Imu)
- Publishes /imu/temperature (sensor_msgs/Temperature)
- Publishes /wheel/odom (nav_msgs/Odometry)
- Optionally publishes TF odom->base_link

## Serial formats supported

A) Single-line (recommended):
ts_ms,ax,ay,az,gx,gy,gz,temp,left_ticks,right_ticks

B) Two-line types:
IMU: ts_ms,ax,ay,az,gx,gy,gz,temp
ENC: ts_ms,left_ticks,right_ticks

Units expected:
- accel m/s^2
- gyro rad/s
- temp C