# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import time

# class MoverNode(Node):
#     def __init__(self):
#         super().__init__('mover_node')
        
#         self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.start_time = time.time()

#     def timer_callback(self):
#         msg = Twist()
#         elapsed_time = time.time() - self.start_time

#         if elapsed_time < 5.0:
#             msg.linear.x = 0.5
#             msg.angular.z = 0.0
#             self.get_logger().info('Forward: Long Step 1')
            
#         elif elapsed_time < 8.14:  # 5.0 + 3.14
#             msg.linear.x = 0.0
#             msg.angular.z = 0.5
#             self.get_logger().info('Turn 90 Degree - 1')
            
#         elif elapsed_time < 10.14:  # 8.14 + 2.0
#             msg.linear.x = 0.5
#             msg.angular.z = 0.0
#             self.get_logger().info('Forward: Short Step 1')
            
#         elif elapsed_time < 13.28: # 10.14 + 3.14
#             msg.linear.x = 0.0
#             msg.angular.z = 0.5
#             self.get_logger().info('Turn 90 Degree - 2')
            
#         elif elapsed_time < 17.28: # 13.28 + 4.0
#             msg.linear.x = 0.5
#             msg.angular.z = 0.0
#             self.get_logger().info('Forward: Long Step 2')
            
#         elif elapsed_time < 20.42: # 17.28 + 3.14
#             msg.linear.x = 0.0
#             msg.angular.z = 0.5
#             self.get_logger().info('Turn 90 Degree - 3')
            
#         elif elapsed_time < 22.42: # 20.42 + 2.0
#             msg.linear.x = 0.5
#             msg.angular.z = 0.0
#             self.get_logger().info('Forward: Short Step 2')
            
#         elif elapsed_time < 25.56: # 22.42 + 3.14
#             msg.linear.x = 0.0
#             msg.angular.z = 0.5
#             self.get_logger().info('Turn 90 Degree (First Orientent)')
            
#         # Selesai
#         else:
#             msg.linear.x = 0.0
#             msg.angular.z = 0.0
#             self.get_logger().info('Lintasan Persegi Panjang Selesai.')
#             self.publisher_.publish(msg)
            
#             self.timer.cancel()
#             rclpy.shutdown()
#             return

#         self.publisher_.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = MoverNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if rclpy.ok():
#             node.destroy_node()
#             rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math


class GoToPointNode(Node):
    def __init__(self):
        super().__init__('go_to_point_node')

        # === Parameter ===
        self.declare_parameter('K_linear', 1.0)       # gain jarak
        self.declare_parameter('K_angular', 2.0)      # gain sudut
        self.declare_parameter('V_MAX', 0.5)          # kecepatan linier max (m/s)
        self.declare_parameter('W_MAX', 1.5)          # kecepatan sudut max (rad/s)
        self.declare_parameter('DIST_THRESHOLD', 0.05)  # toleransi jarak (m)
        self.declare_parameter('ANGLE_THRESHOLD', 0.1)  # toleransi sudut (rad)
        self.declare_parameter('ROTATE_FIRST_ANGLE', 0.5)  # putar dulu kalau error > 30°

        self.K_v = self.get_parameter('K_linear').value
        self.K_w = self.get_parameter('K_angular').value
        self.V_MAX = self.get_parameter('V_MAX').value
        self.W_MAX = self.get_parameter('W_MAX').value
        self.DIST_THRESHOLD = self.get_parameter('DIST_THRESHOLD').value
        self.ANGLE_THRESHOLD = self.get_parameter('ANGLE_THRESHOLD').value
        self.ROTATE_FIRST_ANGLE = self.get_parameter('ROTATE_FIRST_ANGLE').value

        # === State ===
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_received = False

        self.target_x = None
        self.target_y = None
        self.goal_reached = False

        # === Subscribers ===
        self.create_subscription(Odometry, '/odom_custom', self.odom_callback, 10)
        self.create_subscription(Point, '/goal_point', self.goal_callback, 10)

        # === Publisher ===
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # === Timer kontrol 20 Hz ===
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'Go-to-Point Node aktif | '
            f'K_v={self.K_v}, K_w={self.K_w}, '
            f'V_MAX={self.V_MAX}, W_MAX={self.W_MAX}'
        )
        self.get_logger().info(
            'Kirim target via: ros2 topic pub /goal_point geometry_msgs/msg/Point '
            '"{x: 1.0, y: 0.5}" --once'
        )

    # ============ CALLBACK ODOMETRI ============
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Quaternion → yaw (theta)
        q = msg.pose.pose.orientation
        self.theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.odom_received = True

    # ============ CALLBACK GOAL ============
    def goal_callback(self, msg: Point):
        self.target_x = msg.x
        self.target_y = msg.y
        self.goal_reached = False
        self.get_logger().info(
            f'🎯 Target baru: ({self.target_x:.2f}, {self.target_y:.2f})'
        )

    # ============ CONTROL LOOP ============
    def control_loop(self):
        # Belum dapat odom / goal?
        if not self.odom_received or self.target_x is None:
            return

        # Sudah sampai?
        if self.goal_reached:
            return

        # Hitung vektor ke target
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        distance = math.sqrt(dx * dx + dy * dy)

        # Hitung error heading
        angle_to_target = math.atan2(dy, dx)
        angle_error = angle_to_target - self.theta

        # Normalisasi ke [-π, π]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # === Check apakah sudah sampai ===
        if distance < self.DIST_THRESHOLD:
            self.stop_robot()
            self.goal_reached = True
            self.get_logger().info(
                f'✅ Goal tercapai! Posisi akhir: '
                f'({self.x:.3f}, {self.y:.3f}) | '
                f'Error: {distance*100:.2f} cm'
            )
            return

        # === Hitung kecepatan ===
        v = self.K_v * distance
        w = self.K_w * angle_error

        # Strategi "rotate first" — kalau error heading besar, putar dulu
        if abs(angle_error) > self.ROTATE_FIRST_ANGLE:
            v = 0.0
        else:
            # Kalau heading hampir benar, boleh maju
            # Kurangi kecepatan kalau heading masih agak miring
            v = self.K_v * distance * math.cos(angle_error)

        # Clamp
        v = max(min(v, self.V_MAX), -self.V_MAX)
        w = max(min(w, self.W_MAX), -self.W_MAX)

        # Publish
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        # Log periodik (throttled)
        self.get_logger().info(
            f'→ target ({self.target_x:.2f}, {self.target_y:.2f}) | '
            f'pos ({self.x:.2f}, {self.y:.2f}) | '
            f'd={distance:.3f} m | θ_err={math.degrees(angle_error):.1f}° | '
            f'v={v:.2f} ω={w:.2f}',
            throttle_duration_sec=0.5
        )

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoToPointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()