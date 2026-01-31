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


---

## Dead reckoning math used

### 1) CmdVel integration (SE(2))
For each `/cmd_vel` message:
- `v = msg.twist.linear.x`
- `ω = msg.twist.angular.z`
- compute `Δt` from message timestamps

Update:
- `θ(t+Δt) = θ(t) + ω·Δt`
- `x(t+Δt) = x(t) + v·cos(θ)·Δt`
- `y(t+Δt) = y(t) + v·sin(θ)·Δt`

### 2) IMU integration
Yaw:
- `θ(t+Δt) = θ(t) + ωz·Δt` where `ωz = angular_velocity.z`

Rotate body-frame accel to world-frame (2D):
- `ax_world = ax_body·cos(θ) − ay_body·sin(θ)`
- `ay_world = ax_body·sin(θ) + ay_body·cos(θ)`

Integrate:
- velocity: `v(t+Δt) = v(t) + a·Δt`
- position: `p(t+Δt) = p(t) + v·Δt`

---

## Screenshot (RViz output)
Your RViz screenshot is stored in this repo at:
- `images/rviz_paths.png`

RViz paths shown:
- **Cyan** = `/dead_reckoning/path` (CmdVel)
- **Green** = `/imu_integration/path` (IMU)

![RViz trajectories](images/rviz_paths.png)

---

## Required analysis questions (answers)

### 1) How do the two trajectory estimates compare? Do they agree initially? When do they diverge?
They generally agree at the beginning because integration has not accumulated much error yet. As time increases, both estimates drift and diverge. The CmdVel estimate follows the commanded motion pattern, while the IMU estimate can diverge more due to yaw drift and double integration of acceleration.

### 2) Which method exhibits more drift? Why?
The IMU-based position estimate typically shows more drift because position comes from **double integration** (accel → velocity → position). Small accelerometer bias/noise becomes a growing velocity error and then a large position error. CmdVel dead reckoning also drifts because commanded motion differs from actual motion, but often more slowly.

### 3) What are the sources of error in each approach?
CmdVel errors:
- commanded velocity ≠ actual velocity (slip, friction, saturation, dynamics)
- Δt variation and discretization
- planar SE(2) assumption

IMU errors:
- gyro bias/noise → yaw drift
- accelerometer bias/noise → large drift after double integration
- gravity leakage into x/y if slight tilt or imperfect orientation
- sensor alignment/timing issues

### 4) How might you combine these two estimates for better accuracy?
Use sensor fusion such as a complementary filter or EKF:
- predict using cmd_vel/wheel odometry
- correct orientation/short-term motion with IMU
- correct long-term drift using external reference (SLAM, AprilTags, vision odometry, GPS)

### 5) What assumptions does each method make?
CmdVel assumes robot tracks commanded (v, ω), minimal slip, planar motion.
IMU assumes yaw estimate is good enough to rotate acceleration to world frame and that gravity/bias are handled; double integration makes drift grow quickly.

