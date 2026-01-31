# Project 3 — Dead Reckoning (proj3_M76150518)

## What this project does
This project estimates the TurtleBot3 trajectory using two independent dead-reckoning approaches from the provided ROS2 bag (`proj3`):

1) **CmdVel dead reckoning**: integrate `/cmd_vel` in SE(2) to estimate (x, y, yaw).
2) **IMU integration**: integrate IMU gyro `ωz` for yaw and double integrate IMU acceleration (after rotating body → world) to estimate (x, y).

Both trajectories are published for visualization in RViz2 (`nav_msgs/Path`) and for plotting/debugging (`nav_msgs/Odometry`).

---

## Bag contents (confirmed)
This bag contains:
- `/cmd_vel` — `geometry_msgs/msg/TwistStamped`
- `/imu` — `sensor_msgs/msg/Imu`

Check:
```bash
cd ~/proj3_ws
ros2 bag info proj3

