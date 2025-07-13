#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class BallastControl(Node):
    def __init__(self):
        super().__init__('ballast_control')
        self.pub = self.create_publisher(Float64MultiArray, '/ballast_command', 10)
        
        # Example: Pitch down (more water in front)
        self.set_ballast([0.7, 0.3])  # 70% front, 30% rear
    
    def set_ballast(self, values):
        msg = Float64MultiArray()
        msg.data = values
        self.pub.publish(msg)
        self.get_logger().info(f"Ballast set: Front={values[0]:.1f}, Rear={values[1]:.1f}")

def main():
    rclpy.init()
    node = BallastControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
