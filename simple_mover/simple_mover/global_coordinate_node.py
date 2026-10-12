#!/usr/bin/env python3
"""
Global Coordinate Estimator Node
==================================
Subscribes to /cmd_vel (Twist) and numerically integrates the differential-drive
kinematic model to estimate the robot's global pose (X, Y, θ) and velocity
components in the world frame.

Equations used (from images):
  ẋ     = V_B · cos(θ)
  ẏ     = V_B · sin(θ)
  θ̇     = ω

where:
  V_B  = (V_L + V_R) / 2          (body linear velocity)
  ω    = (V_R - V_L) / s          (angular velocity)
  V_L  = v_b - ω·s/2              (left wheel velocity)
  V_R  = v_b + ω·s/2              (right wheel velocity)
  s    = wheel separation (track width)

The node also drives the robot along a rectangle path (same as the commented
program in mover_node.py) and prints a live terminal table every 0.1 s.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


# ──────────────────────────────────────────────────────────────
#  Rectangle path definition (mirrors the commented mover_node)
# ──────────────────────────────────────────────────────────────
# Each entry: (end_time, linear_x, angular_z, label)
#   Times are cumulative from node start [seconds]
RECTANGLE_PATH = [
    # Phase                         t_end   v_b    ω     label
    ("Forward Long 1",               5.00,  0.5,  0.0),
    ("Turn 90° — 1",                 8.14,  0.0,  0.5),
    ("Forward Short 1",             10.14,  0.5,  0.0),
    ("Turn 90° — 2",                13.28,  0.0,  0.5),
    ("Forward Long 2",              17.28,  0.5,  0.0),
    ("Turn 90° — 3",                20.42,  0.0,  0.5),
    ("Forward Short 2",             22.42,  0.5,  0.0),
    ("Turn 90° (Return Orient.)",   25.56,  0.0,  0.5),
    ("DONE",                       999.99,  0.0,  0.0),
]

# ──────────────────────────────────────────────────────────────
#  Helper: pretty terminal table row
# ──────────────────────────────────────────────────────────────
HEADER = (
    f"{'Time':>7} | {'Phase':<28} | "
    f"{'X':>8} {'Y':>8} {'θ(°)':>7} | "
    f"{'Vb':>6} {'ω':>6} | "
    f"{'VL':>6} {'VR':>6} | "
    f"{'ẋ':>7} {'ẏ':>7}"
)
SEP = "-" * len(HEADER)


class GlobalCoordinateNode(Node):
    def __init__(self):
        super().__init__('global_coordinate_node')

        # ── Robot physical parameters ──────────────────────────
        self.declare_parameter('wheel_radius',    0.1)
        self.declare_parameter('wheel_separation', 0.45)

        self.r = self.get_parameter('wheel_radius').value
        self.s = self.get_parameter('wheel_separation').value

        # ── Global state (integrated pose) ─────────────────────
        self.x     = 0.0   # m
        self.y     = 0.0   # m
        self.theta = 0.0   # rad

        # ── Latest command received from /cmd_vel ───────────────
        self.v_b   = 0.0   # body linear velocity  [m/s]
        self.omega = 0.0   # body angular velocity [rad/s]

        # ── Derived velocities ──────────────────────────────────
        self.V_L   = 0.0   # left  wheel velocity  [m/s]
        self.V_R   = 0.0   # right wheel velocity  [m/s]
        self.x_dot = 0.0   # ẋ in global frame     [m/s]
        self.y_dot = 0.0   # ẏ in global frame     [m/s]

        # ── Timing ─────────────────────────────────────────────
        self.start_time  = time.time()
        self.last_update = time.time()

        # ── Rectangle path state ────────────────────────────────
        self.phase_idx   = 0       # current phase index
        self.path_done   = False

        # ── ROS interfaces ──────────────────────────────────────
        # Subscribe to /cmd_vel to track whatever is published
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publish rectangle path commands on /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer: 10 Hz for path commands + integration + display
        self.create_timer(0.1, self.timer_callback)

        # Print header once
        print()
        print(SEP)
        print(HEADER)
        print(SEP)

        self.get_logger().info(
            f'GlobalCoordinate Node started | '
            f'r={self.r} m, s={self.s} m'
        )
        self.get_logger().info(
            'Robot will follow the rectangle path automatically. '
            'Press Ctrl+C to stop.'
        )

    # ──────────────────────────────────────────────────────────
    #  /cmd_vel subscriber — update velocity snapshot
    # ──────────────────────────────────────────────────────────
    def cmd_vel_callback(self, msg: Twist):
        """Called every time a Twist is published on /cmd_vel."""
        self.v_b   = msg.linear.x
        self.omega = msg.angular.z

    # ──────────────────────────────────────────────────────────
    #  Main timer callback (10 Hz)
    # ──────────────────────────────────────────────────────────
    def timer_callback(self):
        now       = time.time()
        elapsed   = now - self.start_time
        dt        = now - self.last_update
        self.last_update = now

        phase_label = self._publish_path_command(elapsed)
        self._integrate_pose(dt)
        self._compute_derived()
        self._print_row(elapsed, phase_label)

    # ──────────────────────────────────────────────────────────
    #  Rectangle path commander
    # ──────────────────────────────────────────────────────────
    def _publish_path_command(self, elapsed: float) -> str:
        if self.path_done:
            return "DONE — robot stopped"

        label, t_end, lin, ang = RECTANGLE_PATH[self.phase_idx]

        # Advance to next phase if time has passed
        while elapsed >= t_end and self.phase_idx < len(RECTANGLE_PATH) - 1:
            self.phase_idx += 1
            label, t_end, lin, ang = RECTANGLE_PATH[self.phase_idx]

        if label == "DONE":
            self.path_done = True

        cmd = Twist()
        cmd.linear.x  = lin
        cmd.angular.z = ang
        self.cmd_pub.publish(cmd)

        return label

    # ──────────────────────────────────────────────────────────
    #  Pose integrator  (Euler forward step)
    # ──────────────────────────────────────────────────────────
    def _integrate_pose(self, dt: float):
        """
        Numerically integrate:
            ẋ = V_B · cos(θ)
            ẏ = V_B · sin(θ)
            θ̇ = ω
        using simple Euler forward method with time-step dt.
        """
        # Midpoint theta for slightly better accuracy on turning segments
        theta_mid = self.theta + 0.5 * self.omega * dt

        self.x     += self.v_b * math.cos(theta_mid) * dt
        self.y     += self.v_b * math.sin(theta_mid) * dt
        self.theta += self.omega * dt

        # Keep theta in [-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    # ──────────────────────────────────────────────────────────
    #  Derived-velocity computation
    # ──────────────────────────────────────────────────────────
    def _compute_derived(self):
        """
        From V_B and ω compute:
            V_L  = V_B - ω·s/2
            V_R  = V_B + ω·s/2
            ẋ    = V_B · cos(θ)
            ẏ    = V_B · sin(θ)
        """
        self.V_L   = self.v_b - self.omega * self.s / 2.0
        self.V_R   = self.v_b + self.omega * self.s / 2.0
        self.x_dot = self.v_b * math.cos(self.theta)
        self.y_dot = self.v_b * math.sin(self.theta)

    # ──────────────────────────────────────────────────────────
    #  Terminal display
    # ──────────────────────────────────────────────────────────
    def _print_row(self, elapsed: float, phase_label: str):
        """Print one row of the live coordinate table."""
        theta_deg = math.degrees(self.theta)
        row = (
            f"{elapsed:>7.2f} | {phase_label:<28} | "
            f"{self.x:>8.4f} {self.y:>8.4f} {theta_deg:>7.2f} | "
            f"{self.v_b:>6.3f} {self.omega:>6.3f} | "
            f"{self.V_L:>6.3f} {self.V_R:>6.3f} | "
            f"{self.x_dot:>7.4f} {self.y_dot:>7.4f}"
        )
        print(row)

        # Print separator + header again every ~5 s for readability
        if abs(elapsed % 5.0) < 0.15:
            print(SEP)
            print(HEADER)
            print(SEP)

        # Announce when path completes
        if self.path_done:
            print()
            print("=" * len(HEADER))
            print(
                f"  RECTANGLE PATH COMPLETE\n"
                f"  Final position : X = {self.x:.4f} m | Y = {self.y:.4f} m\n"
                f"  Final heading  : θ = {math.degrees(self.theta):.2f}°\n"
                f"  (Ideal return to origin; deviation shows accumulated error)"
            )
            print("=" * len(HEADER))
            self.path_done = False   # silence repeated print


# ──────────────────────────────────────────────────────────────
#  Entry point
# ──────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = GlobalCoordinateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot on exit
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()