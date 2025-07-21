#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String # khai bao 

class Ros_Subscriber(Node): # them (Node) de ke thua cai Node, binh thuong chi co class Ros_Publisher:
    def __init__(self):
        super().__init__("subscriber_node") 
        self.get_logger().info("subscriber_node initialized")

        self.subscriber = self.create_subscription(String, 'chatter', self.listener_callback, 10) # cho Int16 la message type, count la topic name 

    
    def listener_callback(self, msg):
        print("msg: ", msg)

def main():
    rclpy.init()
    node = Ros_Subscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()