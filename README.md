# Project 3 — Dead Reckoning (proj3)

## Overview (What this project does)
This project performs **state estimation by dead reckoning** using a provided ROS2 bag recorded from a TurtleBot3 Burger. The goal is to estimate the robot’s 2D trajectory (x, y, yaw) using **two independent methods** and compare how each method drifts over time.

**Method 1 — Command Dead Reckoning (`/cmd_vel`)**
- Uses commanded forward velocity `v = linear.x` and yaw rate `ω = angular.z`.
- Integrates these commands in SE(2) to estimate pose:
  - θ(t+Δt) = θ(t) + ω·Δt  
  - x(t+Δt) = x(t) + v·cos(θ)·Δt  
  - y(t+Δt) = y(t) + v·sin(θ)·Δt  

**Method 2 — IMU Integration (`/imu`)**
- Uses IMU gyroscope and accelerometer:
  - Integrates `angular_velocity.z` to estimate yaw:
    - θ(t+Δt) = θ(t) + ωz·Δt
  - Rotates accelerations from **body frame → world frame** using yaw:
    - ax_world = ax_body·cos(θ) − ay_body·sin(θ)
    - ay_world = ax_body·sin(θ) + ay_body·cos(θ)
  - Double integrates acceleration to estimate position:
    - v(t+Δt) = v(t) + a·Δt
    - p(t+Δt) = p(t) + v·Δt

Both trajectories are published as `nav_msgs/Path` for RViz2 visualization and `nav_msgs/Odometry` for plotting/debugging.

---

## Bag File Details (proj3)
This project uses the provided bag `proj3` located at:
- `~/proj3_ws/proj3`

Bag format:
- Storage: **MCAP**
- ROS distro: **Jazzy**
- Duration: ~49 seconds

Topics in the bag:
- `/cmd_vel` — `geometry_msgs/msg/TwistStamped`
- `/imu` — `sensor_msgs/msg/Imu`

Verify using:
```bash
cd ~/proj3_ws
ros2 bag info proj3
