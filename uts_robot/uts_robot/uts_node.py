#!/usr/bin/env python3
"""
UTS Robotika - node penggerak robot "robin" melewati track UTS (kiri / kanan).

Node ini HANYA memakai dua topic yang diizinkan:
    /wheel/speed  std_msgs/Float64MultiArray  data[0] = phi_L, data[1] = phi_R  [rad/s]
    /cmd_vel      geometry_msgs/Twist         linear.x = VB [m/s], angular.z = omega [rad/s]

Posisi robot TIDAK dibaca dari Gazebo, tetapi diestimasi sendiri (odometri):
    1. Forward kinematics : VB = r/2 (phi_L + phi_R)      omega = r/s (-phi_L + phi_R)
    2. Global coordinate  : x_dot = VB cos(theta)   y_dot = VB sin(theta)   theta_dot = omega
    3. Euler              : x_k = x_(k-1) + x_dot * dt   (begitu juga y dan theta)
Lalu robot diarahkan ke daftar waypoint dengan go-to-goal:
    4. d = sqrt(dx^2 + dy^2)   theta_d = atan2(dy, dx)   e_theta = atan2(sin(.), cos(.))
       VB = Kv * d             omega = Ktheta * e_theta
Saat waypoint terakhir (kotak finish) tercapai, robot berhenti dan waktu tempuh dicetak.

Mode (parameter 'mode'):
    track : jalan melewati track (default)
    lurus : kalibrasi r  -> maju lurus sejauh calib_distance (estimasi), lalu berhenti
    putar : kalibrasi s  -> putar di tempat sebanyak calib_turns putaran (estimasi)
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


# ---------------------------------------------------------------------------
# 1. WAYPOINT TRACK
#    Koordinat dunia (meter). Robot mulai di (0, 0) menghadap sumbu +x.
#    Titik-titik ini berada di tengah lorong track (lebar lorong 0,6 m),
#    diambil dari file map robin_bringup/worlds/map_uts.sdf (branch uts).
# ---------------------------------------------------------------------------
TRACK_KIRI = [
    (0.60, 2.38),   # 1  di depan pintu masuk track
    (2.42, 2.38),   # 2  sudut: naik
    (2.42, 3.04),   # 3  sudut: ke kanan
    (3.66, 3.04),   # 4  sudut: turun
    (3.66, 0.79),   # 5  sudut: lorong panjang bawah
    (7.91, 0.79),   # 6  sudut: naik di sisi kanan
    (7.91, 5.86),   # 7  awal busur (setengah lingkaran)
    (7.68, 6.62),   # 8  busur 30 derajat
    (7.11, 7.16),   # 9  busur 60 derajat
    (6.36, 7.35),   # 10 busur 90 derajat (puncak)
    (5.61, 7.15),   # 11 busur 120 derajat
    (5.06, 6.61),   # 12 busur 150 derajat
    (4.84, 5.86),   # 13 akhir busur
    (4.84, 4.27),   # 14 sudut: ke kiri
    (3.94, 4.27),   # 15 sudut: naik
    (3.94, 6.78),   # 16 sudut: masuk ruangan
    (3.00, 6.78),   # 17 di dalam ruangan
    (1.50, 7.54),   # 18 menuju celah atas
    (0.28, 7.54),   # 19 sudut: turun ke lorong terakhir
    (0.28, 4.80),   # 20 FINISH (ujung lorong terakhir)
]

# Track kanan adalah cerminan track kiri terhadap garis y = -0,0594 m.
TRACK_KANAN = [(x, round(-0.1188 - y, 2)) for (x, y) in TRACK_KIRI]


def normalize_angle(a):
    """Normalisasi sudut ke rentang -pi .. pi (rumus e_theta di materi)."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(value, low, high):
    return max(low, min(high, value))


class UtsNode(Node):

    def __init__(self):
        super().__init__('uts_node')

        # -------------------------------------------------------------------
        # 2. PARAMETER (bisa diubah saat run: --ros-args -p nama:=nilai)
        # -------------------------------------------------------------------
        self.declare_parameter('mode', 'track')           # 'track', 'lurus', 'putar'
        self.declare_parameter('track', 'kiri')           # 'kiri' atau 'kanan'
        self.declare_parameter('wheel_radius', 0.03)      # r [m], roda fisik robot
        self.declare_parameter('wheel_separation', 0.17)  # s [m], jarak roda fisik
        self.declare_parameter('v_max', 1.0)              # batas perintah linear.x
        self.declare_parameter('w_max', 2.0)              # batas perintah angular.z
        self.declare_parameter('acc_lin', 1.0)            # batas perubahan linear.x per detik
        self.declare_parameter('acc_ang', 3.0)            # batas perubahan angular.z per detik
        self.declare_parameter('k_v', 3.0)                # Kv
        self.declare_parameter('k_theta', 2.0)            # Ktheta
        self.declare_parameter('turn_in_place', 0.6)      # |e_theta| > ini -> putar dulu [rad]
        self.declare_parameter('wp_tolerance', 0.08)      # epsilon_p waypoint biasa [m]
        self.declare_parameter('goal_tolerance', 0.05)    # epsilon_p kotak finish [m]
        self.declare_parameter('calib_distance', 6.0)     # mode lurus [m]
        self.declare_parameter('calib_turns', 4.0)        # mode putar [putaran]

        self.mode = str(self.get_parameter('mode').value).lower()
        track = str(self.get_parameter('track').value).lower()
        self.r = float(self.get_parameter('wheel_radius').value)
        self.s = float(self.get_parameter('wheel_separation').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.acc_lin = float(self.get_parameter('acc_lin').value)
        self.acc_ang = float(self.get_parameter('acc_ang').value)
        self.k_v = float(self.get_parameter('k_v').value)
        self.k_theta = float(self.get_parameter('k_theta').value)
        self.turn_in_place = float(self.get_parameter('turn_in_place').value)
        self.wp_tol = float(self.get_parameter('wp_tolerance').value)
        self.goal_tol = float(self.get_parameter('goal_tolerance').value)
        self.calib_distance = float(self.get_parameter('calib_distance').value)
        self.calib_turns = float(self.get_parameter('calib_turns').value)

        if self.mode == 'lurus':
            self.waypoints = [(self.calib_distance, 0.0)]
            self.goal_tol = 0.01
        elif self.mode == 'putar':
            self.waypoints = []
        else:
            self.mode = 'track'
            if track != 'kanan':
                track = 'kiri'
            self.waypoints = TRACK_KANAN if track == 'kanan' else TRACK_KIRI
        self.track = track

        # State odometri (posisi awal = posisi spawn robot)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.total_rotation = 0.0        # sudut kumulatif (tidak dinormalisasi)
        self.last_odom_time = None
        self.got_wheel_data = False
        self.reset_samples()

        # State navigasi
        self.wp_index = 0
        self.started = False
        self.finished = False
        self.start_time = None
        self.last_log_time = None
        self.cmd_v = 0.0                 # perintah terakhir (untuk batas percepatan)
        self.cmd_w = 0.0
        self.dt_ctrl = 0.05              # periode kontrol 20 Hz

        # -------------------------------------------------------------------
        # 3. TOPIC
        # -------------------------------------------------------------------
        self.wheel_sub = self.create_subscription(
            Float64MultiArray, '/wheel/speed', self.wheel_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(self.dt_ctrl, self.control_loop)

        sim_time = self.get_parameter('use_sim_time').value
        self.get_logger().info(
            f'Mode {self.mode.upper()} | track {self.track.upper()} | '
            f'r={self.r} m, s={self.s} m | use_sim_time={sim_time} | '
            f'menunggu data /wheel/speed ...')

    # -----------------------------------------------------------------------
    # 4. ODOMETRI: dipanggil setiap ada data kecepatan roda
    # -----------------------------------------------------------------------
    def wheel_callback(self, msg):
        if len(msg.data) < 2:
            return
        # Kumpulkan data roda; dirata-rata per selang waktu dt
        # (penting kalau jam simulasi ter-update lebih jarang dari data roda)
        self.sum_l += float(msg.data[0])  # kecepatan sudut roda kiri  [rad/s]
        self.sum_r += float(msg.data[1])  # kecepatan sudut roda kanan [rad/s]
        self.n_samples += 1
        self.got_wheel_data = True

        now = self.get_clock().now()
        if self.last_odom_time is None:
            self.last_odom_time = now
            self.reset_samples()
            return
        dt = (now - self.last_odom_time).nanoseconds * 1e-9    # delta t_k = t_k - t_(k-1)
        if dt <= 0.0:
            return                        # jam belum maju, tunggu data berikutnya
        phi_l = self.sum_l / self.n_samples
        phi_r = self.sum_r / self.n_samples
        self.reset_samples()
        self.last_odom_time = now
        if dt > 0.5:                      # abaikan jeda yang tidak wajar
            return

        # Forward kinematics (local coordinate)
        v_b = self.r / 2.0 * (phi_l + phi_r)
        omega = self.r / self.s * (-phi_l + phi_r)

        # Global coordinate (theta di tengah langkah, sedikit lebih akurat dari Euler biasa)
        theta_mid = self.theta + omega * dt / 2.0
        x_dot = v_b * math.cos(theta_mid)
        y_dot = v_b * math.sin(theta_mid)

        # Position estimation: x_k = x_(k-1) + x_dot * dt
        self.x += x_dot * dt
        self.y += y_dot * dt
        self.theta = normalize_angle(self.theta + omega * dt)
        self.total_rotation += omega * dt

    def reset_samples(self):
        self.sum_l = 0.0
        self.sum_r = 0.0
        self.n_samples = 0

    # -----------------------------------------------------------------------
    # 5. KONTROL GO-TO-GOAL: dipanggil 20 kali per detik
    # -----------------------------------------------------------------------
    def control_loop(self):
        if self.finished:
            self.publish_cmd(0.0, 0.0, smooth=False)
            return

        if not self.started:
            if not self.got_wheel_data:
                return
            if self.get_clock().now().nanoseconds == 0:
                return                    # use_sim_time: tunggu /clock pertama
            self.started = True
            self.start_time = self.get_clock().now()
            self.get_logger().info('START! Robot mulai bergerak.')

        if self.mode == 'putar':
            self.rotate_loop()
            return

        goal_x, goal_y = self.waypoints[self.wp_index]
        dx = goal_x - self.x
        dy = goal_y - self.y
        d = math.hypot(dx, dy)                          # jarak ke waypoint

        is_last = self.wp_index == len(self.waypoints) - 1
        tolerance = self.goal_tol if is_last else self.wp_tol

        if d <= tolerance:                              # d <= epsilon_p
            if is_last:
                self.finish()
                return
            self.wp_index += 1
            self.get_logger().info(
                f'Waypoint {self.wp_index}/{len(self.waypoints)} tercapai '
                f'(x={self.x:.2f}, y={self.y:.2f})')
            goal_x, goal_y = self.waypoints[self.wp_index]
            dx = goal_x - self.x
            dy = goal_y - self.y
            d = math.hypot(dx, dy)

        theta_d = math.atan2(dy, dx)                    # sudut arah waypoint
        e_theta = normalize_angle(theta_d - self.theta)  # error sudut

        omega = clamp(self.k_theta * e_theta, -self.w_max, self.w_max)
        if abs(e_theta) > self.turn_in_place:
            v_b = 0.0                                   # putar di tempat dulu
        else:
            v_b = min(self.k_v * d, self.v_max) * math.cos(e_theta)

        self.publish_cmd(v_b, omega)
        self.log_status(d)

    def rotate_loop(self):
        target = 2.0 * math.pi * self.calib_turns
        remaining = target - self.total_rotation
        if remaining <= 0.002:
            self.finish()
            return
        omega = clamp(self.k_theta * remaining, 0.0, self.w_max)
        self.publish_cmd(0.0, max(omega, 0.05))
        self.log_status(remaining)

    def finish(self):
        self.finished = True
        self.publish_cmd(0.0, 0.0, smooth=False)
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        log = self.get_logger()
        log.info('=' * 55)
        if self.mode == 'lurus':
            log.info(f'KALIBRASI LURUS selesai: estimasi jarak = {self.x:.3f} m')
            log.info('Baca posisi x robot di Gazebo, lalu hitung:')
            log.info(f'  r_baru = {self.r} x (x_gazebo / {self.x:.3f})')
        elif self.mode == 'putar':
            turns = self.total_rotation / (2.0 * math.pi)
            log.info(f'KALIBRASI PUTAR selesai: estimasi = {turns:.3f} putaran')
            log.info('Lihat arah hadap robot di Gazebo (awalnya ke +x):')
            log.info('  lebih dari target (kelebihan putar) -> s diperkecil')
            log.info('  kurang dari target (kurang putar)   -> s diperbesar')
            log.info(f'  s_baru = {self.s} x {360 * turns:.0f} / sudut_asli_derajat')
        else:
            log.info(f'FINISH! Track {self.track.upper()}')
        log.info(f'Waktu tempuh : {elapsed:.2f} detik')
        log.info(
            f'Posisi akhir (estimasi): x={self.x:.2f} m, y={self.y:.2f} m, '
            f'theta={math.degrees(self.theta):.1f} deg')
        log.info('=' * 55)

    def publish_cmd(self, v_b, omega, smooth=True):
        if smooth:   # batasi percepatan supaya roda tidak slip
            dv = self.acc_lin * self.dt_ctrl
            dw = self.acc_ang * self.dt_ctrl
            v_b = clamp(v_b, self.cmd_v - dv, self.cmd_v + dv)
            omega = clamp(omega, self.cmd_w - dw, self.cmd_w + dw)
        self.cmd_v, self.cmd_w = v_b, omega
        msg = Twist()
        msg.linear.x = float(v_b)
        msg.angular.z = float(omega)
        self.cmd_pub.publish(msg)

    def log_status(self, d):
        now = self.get_clock().now()
        if self.last_log_time is not None and \
                (now - self.last_log_time).nanoseconds < 1e9:
            return
        self.last_log_time = now
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        wp = f'WP {self.wp_index + 1}/{len(self.waypoints)}' if self.waypoints else 'PUTAR'
        self.get_logger().info(
            f't={elapsed:6.1f}s | {wp} | x={self.x:5.2f} y={self.y:5.2f} '
            f'theta={math.degrees(self.theta):6.1f} deg | sisa={d:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = UtsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_cmd(0.0, 0.0, smooth=False)   # pastikan robot berhenti
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
