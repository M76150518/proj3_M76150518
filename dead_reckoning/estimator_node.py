#!/usr/bin/env python3
"""
Project 3: Dead Reckoning

You are given a ROS2 bag (proj3) with:
- /cmd_vel : geometry_msgs/msg/TwistStamped
- /imu     : sensor_msgs/msg/Imu

Task:
1) Dead reckoning from /cmd_vel (SE(2) integration)
2) IMU integration:
   - yaw from integrating gyro wz
   - position from double integrating accel (ax, ay) after rotating body->world using yaw
Publish for visualization:
- /dead_reckoning/path   (nav_msgs/Path)
- /dead_reckoning/odom   (nav_msgs/Odometry)
- /imu_integration/path  (nav_msgs/Path)
- /imu_integration/odom  (nav_msgs/Odometry)
"""

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Quaternion
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu


def stamp_to_sec(stamp) -> float:
    """builtin_interfaces/msg/Time -> float seconds"""
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def yaw_to_quat(yaw: float) -> Quaternion:
    """Convert planar yaw to quaternion (z,w used)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


@dataclass
class SE2State:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class IMUState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0


class DeadReckoningEstimator(Node):
    def __init__(self):
        super().__init__('dead_reckoning_estimator')

        # -------------------------
        # Parameters (best practice)
        # -------------------------
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')

        # IMU handling knobs
        self.declare_parameter('use_imu_bias_calibration', True)
        self.declare_parameter('bias_calib_samples', 100)     # use first N imu msgs to estimate accel bias
        self.declare_parameter('accel_lowpass_alpha', 0.15)   # 0..1 (higher = less smoothing)
        self.declare_parameter('vel_damping', 0.0)            # keep 0.0 for “pure drift”; optional small damping



        self.fixed_frame = str(self.get_parameter('fixed_frame').value)
        self.child_frame = str(self.get_parameter('child_frame').value)

        self.use_bias_calib = bool(self.get_parameter('use_imu_bias_calibration').value)
        self.bias_calib_samples = int(self.get_parameter('bias_calib_samples').value)
        self.alpha = float(self.get_parameter('accel_lowpass_alpha').value)
        self.vel_damping = float(self.get_parameter('vel_damping').value)

        # -------------------------
        # QoS
        # -------------------------
        # Bag playback is often BEST_EFFORT: use sensor_data QoS for subscribers.
        sub_qos = qos_profile_sensor_data

        # Path publishers: transient local so RViz can start late and still get the full path.
        self.path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Odom publishers: reliable, volatile
        self.odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # -------------------------
        # State
        # -------------------------
        self.cmd_state = SE2State()
        self.imu_state = IMUState()

        self.last_cmd_t: Optional[float] = None
        self.last_imu_t: Optional[float] = None

        # IMU bias + filtering for ax, ay
        self.bias_count = 0
        self.ax_bias_sum = 0.0
        self.ay_bias_sum = 0.0
        self.ax_f = 0.0
        self.ay_f = 0.0

        # -------------------------
        # Messages for Path
        # -------------------------
        self.cmd_path = Path()
        self.cmd_path.header.frame_id = self.fixed_frame

        self.imu_path = Path()
        self.imu_path.header.frame_id = self.fixed_frame

        # -------------------------
        # Publishers
        # -------------------------
        self.cmd_path_pub = self.create_publisher(Path, '/dead_reckoning/path', self.path_qos)
        self.cmd_odom_pub = self.create_publisher(Odometry, '/dead_reckoning/odom', self.odom_qos)

        self.imu_path_pub = self.create_publisher(Path, '/imu_integration/path', self.path_qos)
        self.imu_odom_pub = self.create_publisher(Odometry, '/imu_integration/odom', self.odom_qos)

        # -------------------------
        # Subscribers
        # -------------------------
        # IMPORTANT: your bag uses TwistStamped
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd_vel, sub_qos)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.on_imu, sub_qos)

        self.get_logger().info("DeadReckoningEstimator started. Play bag with: ros2 bag play proj3 --clock")

    # ----------------------------------------
    # /cmd_vel dead reckoning (TwistStamped)
    # ----------------------------------------
    def on_cmd_vel(self, msg: TwistStamped):
        # Use header stamp for accurate dt from bag
        t = stamp_to_sec(msg.header.stamp)

        if self.last_cmd_t is None:
            self.last_cmd_t = t
            self.publish_cmd_outputs()
            return

        dt = t - self.last_cmd_t
        if dt <= 0.0 or dt > 1.0:
            # ignore big jumps or backwards time
            self.last_cmd_t = t
            return

        v = float(msg.twist.linear.x)
        w = float(msg.twist.angular.z)

        # Discrete integration in SE(2)
        self.cmd_state.yaw += w * dt
        self.cmd_state.x += v * math.cos(self.cmd_state.yaw) * dt
        self.cmd_state.y += v * math.sin(self.cmd_state.yaw) * dt

        self.last_cmd_t = t
        self.publish_cmd_outputs()

    def publish_cmd_outputs(self):
        stamp_msg = self.get_clock().now().to_msg()

        ps = PoseStamped()
        ps.header.frame_id = self.fixed_frame
        ps.header.stamp = stamp_msg
        ps.pose.position.x = self.cmd_state.x
        ps.pose.position.y = self.cmd_state.y
        ps.pose.position.z = 0.0
        ps.pose.orientation = yaw_to_quat(self.cmd_state.yaw)

        self.cmd_path.header.stamp = stamp_msg
        self.cmd_path.poses.append(ps)
        self.cmd_path_pub.publish(self.cmd_path)

        odom = Odometry()
        odom.header.frame_id = self.fixed_frame
        odom.child_frame_id = self.child_frame
        odom.header.stamp = stamp_msg
        odom.pose.pose = ps.pose
        self.cmd_odom_pub.publish(odom)

    # ----------------------------------------
    # /imu integration (Imu)
    # ----------------------------------------
    def on_imu(self, msg: Imu):
        # Prefer header timestamp when available
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            # fallback if stamp not set (rare)
            t = float(self.get_clock().now().nanoseconds) * 1e-9
        else:
            t = stamp_to_sec(msg.header.stamp)

        if self.last_imu_t is None:
            self.last_imu_t = t
            self.publish_imu_outputs()
            return

        dt = t - self.last_imu_t
        if dt <= 0.0 or dt > 1.0:
            self.last_imu_t = t
            return

        # 1) yaw from gyro (ground robot rotates mainly about z)
        wz = float(msg.angular_velocity.z)
        self.imu_state.yaw += wz * dt

        # 2) accel in body frame (x,y plane)
        ax_b = float(msg.linear_acceleration.x)
        ay_b = float(msg.linear_acceleration.y)

        # 2a) estimate accel bias from first N samples
        if self.use_bias_calib and self.bias_count < self.bias_calib_samples:
            self.ax_bias_sum += ax_b
            self.ay_bias_sum += ay_b
            self.bias_count += 1

            if self.bias_count == self.bias_calib_samples:
                ax_bias = self.ax_bias_sum / float(self.bias_calib_samples)
                ay_bias = self.ay_bias_sum / float(self.bias_calib_samples)
                self.get_logger().info(
                    f"Accel bias calibrated (first {self.bias_calib_samples} samples): "
                    f"ax_bias={ax_bias:.4f}, ay_bias={ay_bias:.4f}"
                )

        # Compute current bias estimate (safe even before finishing calibration)
        ax_bias = (self.ax_bias_sum / float(self.bias_count)) if (self.use_bias_calib and self.bias_count > 0) else 0.0
        ay_bias = (self.ay_bias_sum / float(self.bias_count)) if (self.use_bias_calib and self.bias_count > 0) else 0.0

        ax_b -= ax_bias
        ay_b -= ay_bias

        # 2b) low-pass filter accel to reduce high-frequency noise
        self.ax_f = (1.0 - self.alpha) * self.ax_f + self.alpha * ax_b
        self.ay_f = (1.0 - self.alpha) * self.ay_f + self.alpha * ay_b

        # 3) rotate accel body->world using yaw
        c = math.cos(self.imu_state.yaw)
        s = math.sin(self.imu_state.yaw)
        ax_w = self.ax_f * c - self.ay_f * s
        ay_w = self.ax_f * s + self.ay_f * c

        # 4) integrate accel -> velocity
        self.imu_state.vx += ax_w * dt
        self.imu_state.vy += ay_w * dt

        # optional mild damping (leave default = 0.0 for drift demo)
        if self.vel_damping > 0.0:
            damp = max(0.0, 1.0 - self.vel_damping)
            self.imu_state.vx *= damp
            self.imu_state.vy *= damp

        # 5) integrate velocity -> position
        self.imu_state.x += self.imu_state.vx * dt
        self.imu_state.y += self.imu_state.vy * dt

        self.last_imu_t = t
        self.publish_imu_outputs()

    def publish_imu_outputs(self):
        stamp_msg = self.get_clock().now().to_msg()

        ps = PoseStamped()
        ps.header.frame_id = self.fixed_frame
        ps.header.stamp = stamp_msg
        ps.pose.position.x = self.imu_state.x
        ps.pose.position.y = self.imu_state.y
        ps.pose.position.z = 0.0
        ps.pose.orientation = yaw_to_quat(self.imu_state.yaw)

        self.imu_path.header.stamp = stamp_msg
        self.imu_path.poses.append(ps)
        self.imu_path_pub.publish(self.imu_path)

        odom = Odometry()
        odom.header.frame_id = self.fixed_frame
        odom.child_frame_id = self.child_frame
        odom.header.stamp = stamp_msg
        odom.pose.pose = ps.pose
        odom.twist.twist.linear.x = float(self.imu_state.vx)
        odom.twist.twist.linear.y = float(self.imu_state.vy)
        self.imu_odom_pub.publish(odom)


def main():
    rclpy.init()
    node = DeadReckoningEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

