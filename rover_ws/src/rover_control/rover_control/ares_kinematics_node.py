#!/usr/bin/env python3
"""ARES-G four-wheel-steering (4WS) kinematics node.

Translates a Nav2 / teleop ``/cmd_vel`` Twist into the low-level actuator
commands for the genuine ARES-G steering architecture (L0.3 3.1.2):

  * Two front *bogies* steer as rigid units about a shared kingpin.
  * Two rear wheels steer individually.
  * Six wheels are driven.

Two regimes, matching the doc (four-station steering with a skid-steer fallback):

  ACKERMANN (|v| above a threshold)
      Opposite-phase 4WS: front kingpins +delta, rear kingpins -delta, giving a
      tight coordinated turn.  delta = atan( wz * L / (2 v) ).

  SKID (near-zero forward speed, non-zero yaw)
      Kingpins centred; left/right wheel speeds differ to spin in place.

It also integrates wheel odometry from the measured ``/joint_states`` and
publishes ``/odom`` (+ optional odom->base_footprint TF), so the localization
EKF has a wheel-odometry source exactly as the real vehicle would.

Command topics (Float64MultiArray, index order fixed by ares_controllers.yaml):
    /wheel_velocity_controller/commands   [FL, FR, ML, MR, RL, RR]  (rad/s)
    /steering_position_controller/commands [L_bogie, R_bogie, L_rear, R_rear] (rad)
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster


WHEEL_ORDER = ['front_left_wheel_joint', 'front_right_wheel_joint',
               'mid_left_wheel_joint', 'mid_right_wheel_joint',
               'rear_left_wheel_joint', 'rear_right_wheel_joint']
STEER_ORDER = ['left_bogie_steer_joint', 'right_bogie_steer_joint',
               'left_rear_steer_joint', 'right_rear_steer_joint']


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class AresKinematics(Node):
    def __init__(self):
        super().__init__('ares_kinematics_node')

        # --- Geometry (ARES-G L0.3 3.2.4 / 3.1.6) ---
        self.declare_parameter('wheelbase', 0.42)
        self.declare_parameter('track', 0.28)
        self.declare_parameter('wheel_radius', 0.11)
        self.declare_parameter('max_steer', 0.6)        # rad
        self.declare_parameter('min_ackermann_speed', 0.06)  # m/s
        self.declare_parameter('cmd_timeout', 0.5)      # s, stop if stale
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('rate', 50.0)

        self.L = self.get_parameter('wheelbase').value
        self.T = self.get_parameter('track').value
        self.r = self.get_parameter('wheel_radius').value
        self.max_steer = self.get_parameter('max_steer').value
        self.min_ack = self.get_parameter('min_ackermann_speed').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        rate = self.get_parameter('rate').value

        # --- State ---
        self._cmd = Twist()
        self._cmd_stamp = self.get_clock().now()
        self._js = {}                       # joint name -> (pos, vel)
        self._x = self._y = self._th = 0.0
        self._last = self.get_clock().now()

        # --- I/O ---
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        self.steer_pub = self.create_publisher(
            Float64MultiArray, '/steering_position_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_bc = TransformBroadcaster(self)

        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(JointState, '/joint_states', self._on_js, 20)

        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'ARES 4WS kinematics up: L={self.L} T={self.T} r={self.r}')

    # ------------------------------------------------------------------
    def _on_cmd(self, msg):
        self._cmd = msg
        self._cmd_stamp = self.get_clock().now()

    def _on_js(self, msg):
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 0.0
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            self._js[name] = (pos, vel)

    # ------------------------------------------------------------------
    def _compute_commands(self, vx, wz):
        """Return (wheel_speeds[6], steer_angles[4]) for a desired body twist."""
        half_L = self.L * 0.5

        # Stationary
        if abs(vx) < 1e-4 and abs(wz) < 1e-4:
            return [0.0] * 6, [0.0] * 4

        # Skid fallback: little/no forward speed but a yaw command -> spin in place
        if abs(vx) < self.min_ackermann_speed:
            v_left = vx - wz * (self.T * 0.5)
            v_right = vx + wz * (self.T * 0.5)
            wl, wr = v_left / self.r, v_right / self.r
            # [FL, FR, ML, MR, RL, RR]
            wheels = [wl, wr, wl, wr, wl, wr]
            return wheels, [0.0, 0.0, 0.0, 0.0]

        # Ackermann opposite-phase 4WS
        delta = math.atan2(wz * half_L, abs(vx))
        delta = max(-self.max_steer, min(self.max_steer, delta))
        # All wheels roll along their heading at v / cos(delta)
        w = (vx / max(math.cos(delta), 1e-3)) / self.r
        wheels = [w, w, w, w, w, w]
        # front bogies +delta, rear wheels -delta
        steers = [delta, delta, -delta, -delta]
        return wheels, steers

    # ------------------------------------------------------------------
    def _measured_twist(self):
        """Estimate body (vx, wz) from measured joint states (wheel odometry)."""
        def wv(name):
            return self._js.get(name, (0.0, 0.0))[1]

        def sp(name):
            return self._js.get(name, (0.0, 0.0))[0]

        # measured mean steering (front bogie) magnitude
        dbl = sp('left_bogie_steer_joint')
        dbr = sp('right_bogie_steer_joint')
        delta = 0.5 * (dbl + dbr)

        speeds = [wv(n) * self.r for n in WHEEL_ORDER]  # m/s at each wheel
        v_mean = sum(speeds) / len(speeds)
        vx = v_mean * math.cos(delta)

        if abs(delta) > 0.02:
            # Ackermann geometry: wz = 2 v tan(delta) / L
            wz = 2.0 * vx * math.tan(delta) / self.L
        else:
            # Skid: yaw from left/right speed difference
            v_left = (speeds[0] + speeds[2] + speeds[4]) / 3.0
            v_right = (speeds[1] + speeds[3] + speeds[5]) / 3.0
            wz = (v_right - v_left) / self.T
        return vx, wz

    # ------------------------------------------------------------------
    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self._last).nanoseconds * 1e-9
        self._last = now
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / 50.0

        # Command (zero out if the cmd_vel is stale — safety)
        age = (now - self._cmd_stamp).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            vx, wz = 0.0, 0.0
        else:
            vx, wz = self._cmd.linear.x, self._cmd.angular.z

        wheels, steers = self._compute_commands(vx, wz)
        self.wheel_pub.publish(Float64MultiArray(data=wheels))
        self.steer_pub.publish(Float64MultiArray(data=steers))

        # Odometry from measured wheel state
        mvx, mwz = self._measured_twist()
        self._th += mwz * dt
        self._x += mvx * math.cos(self._th) * dt
        self._y += mvx * math.sin(self._th) * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        qx, qy, qz, qw = yaw_to_quat(self._th)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = mvx
        odom.twist.twist.angular.z = mwz
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.05
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_bc.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = AresKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
