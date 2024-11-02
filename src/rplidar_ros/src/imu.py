#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class IMUPublisherNode(Node):
    def __init__(self):
        super().__init__("IMU_Publisher")
        self.port = serial.Serial(port='/dev/ttyACM0', baudrate=9600)
        self.publisher_ = self.create_publisher(String, 'imu_data', 10)
        self.timer = self.create_timer(0.01, self.callback)
        self.get_logger().info('Starting IMU publishing node')
        
    def callback(self):
        (x, y, z) = self.port.readline().decode().strip().split('\t')[2:]
        msg = String()
        msg.data = f'{x},{y},{z}'
        self.get_logger().info(msg.data)
        self.publisher_.publish(msg)

        
        
def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisherNode()
    
    rclpy.spin(node)
    node.destroy_node()
    
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()