# Project 3 — Dead Reckoning (proj3_M76150518)

## What this project does
This project estimates a TurtleBot3 Burger trajectory using **two independent dead-reckoning approaches** from the provided ROS2 bag (`proj3`):

1) **CmdVel dead reckoning**: integrate `/cmd_vel` in SE(2) to estimate (x, y, yaw).  
2) **IMU integration**: integrate IMU gyro `ωz` for yaw and **double integrate** IMU acceleration (after rotating body → world) to estimate (x, y).

Both trajectories are published for visualization in RViz2 (`nav_msgs/Path`) and for plotting/debugging (`nav_msgs/Odometry`).

---

## Bag contents (confirmed)
This bag contains:
- `/cmd_vel` — `geometry_msgs/msg/TwistStamped`
- `/imu` — `sensor_msgs/msg/Imu`


Verify:
```bash
cd ~/proj3_ws
ros2 bag info proj3


##Dead reckoning math used
###1) CmdVel integration (SE(2))

For each /cmd_vel message:

v = msg.twist.linear.x

ω = msg.twist.angular.z

compute Δt from message timestamps

Update:

θ(t+Δt) = θ(t) + ω·Δt

x(t+Δt) = x(t) + v·cos(θ)·Δt

y(t+Δt) = y(t) + v·sin(θ)·Δt

###2) IMU integration

Yaw (ground robot assumes rotation mainly about z):

θ(t+Δt) = θ(t) + ωz·Δt where ωz = angular_velocity.z

Rotate body-frame accel to world-frame (2D):

ax_world = ax_body·cos(θ) − ay_body·sin(θ)

ay_world = ax_body·sin(θ) + ay_body·cos(θ)

Integrate:

velocity: v(t+Δt) = v(t) + a·Δt

position: p(t+Δt) = p(t) + v·Δt

Note: the IMU-based position estimate typically drifts quickly due to double integration (accel → velocity → position) and sensor bias/noise.
