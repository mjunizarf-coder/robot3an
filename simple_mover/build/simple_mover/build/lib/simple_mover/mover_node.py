import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoverNode(Node):
    def __init__(self):
        super().__init__('mover_node')
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()

    def timer_callback(self):
        msg = Twist()
        elapsed_time = time.time() - self.start_time

        if elapsed_time < 5.0:
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.get_logger().info('Forward: Long Step 1')
            
        elif elapsed_time < 8.14:  # 5.0 + 3.14
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.get_logger().info('Turn 90 Degree - 1')
            
        elif elapsed_time < 10.14:  # 8.14 + 2.0
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.get_logger().info('Forward: Short Step 1')
            
        elif elapsed_time < 13.28: # 10.14 + 3.14
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.get_logger().info('Turn 90 Degree - 2')
            
        elif elapsed_time < 17.28: # 13.28 + 4.0
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.get_logger().info('Forward: Long Step 2')
            
        elif elapsed_time < 20.42: # 17.28 + 3.14
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.get_logger().info('Turn 90 Degree - 3')
            
        elif elapsed_time < 22.42: # 20.42 + 2.0
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.get_logger().info('Forward: Short Step 2')
            
        elif elapsed_time < 25.56: # 22.42 + 3.14
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.get_logger().info('Turn 90 Degree (First Orientent)')
            
        # Selesai
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('Lintasan Persegi Panjang Selesai.')
            self.publisher_.publish(msg)
            
            self.timer.cancel()
            rclpy.shutdown()
            return

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()