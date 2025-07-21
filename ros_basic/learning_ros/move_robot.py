#!/usr/bin/env python3
import rclpy
import math
import signal
import numpy as np
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped # khai bao 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class move_robot_pub(Node): # them (Node) de ke thua cai Node, binh thuong chi co class Ros_Publisher:
    def __init__(self):
        super().__init__("publisher_node") 
        print("publisher_node initialized ")
        self.get_logger().info("publisher_node initialized")
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10) # cho Int16 la message type, count la topic name 
        self.subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.plc_monitor_timer = self.create_timer(0.1, self.timer_callback) # cứ 0.1 giây thì gọi hàm 1 lần
        self.distance_odom = 0.0  # quãng đường đã đi
        self.distance_lidar = 0.0
        self.speed = 0.5     # giả định tốc độ là 0.2 m/s
        self.current_pos = None
        self.initialized = None
        
    def timer_callback(self):
        msg = AckermannDriveStamped() # khoi tao 1 cai msg co kieu du lieu la Int16
        if self.distance_odom is  None or self.distance_lidar is None:
            return
        if self.distance_odom > 10:
            msg.drive.speed = 0.0
        elif self.distance_lidar <= 1.25:
            print("STOP")
            msg.drive.speed = 0.0
        else:
            msg.drive.speed = 1.0
            msg.drive.steering_angle = 0.0

        # if self.distance_lidar <= 0.1:
        #     self.speed = 0.0
        #     self.get_logger().info("Obstacle detected. Stopping.")
        # if self.distance_odom >= 2.0:
        #     self.speed = 0.0
        #     self.get_logger().info("Reached 2 meters. Stopping.")
        # else:
        #     msg.drive.speed =  self.speed 
        #     msg.drive.steering_angle = 0.0
        
        self.publisher_.publish(msg)

    def odom_callback(self, msg):
       # print("msg: ", msg.pose.pose.position)
        self.current_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        # print(self.current_pos)
        if self.initialized is None:
            self.initialized = self.current_pos

        dx = self.current_pos[0] - self.initialized[0]
        dy = self.current_pos[1] - self.initialized[1]
        self.distance_odom  = math.sqrt(dx**2 + dy**2)

    def scan_callback(self, scan_msg:LaserScan):
        angle_deg = 0
        for idx, r in enumerate(scan_msg.ranges):
            if(np.isnan(r) or r > scan_msg.range_max or r < scan_msg.range_min):
                continue
            angle_rad = math.radians(angle_deg)
            index = int((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)
            self.distance_lidar = scan_msg.ranges[index]
            print(self.distance_lidar)
            
    def stop_robot(self):
        self.running = False
        stop_msg = AckermannDriveStamped()
        stop_msg.drive.speed = 0.0
        stop_msg.drive.steering_angle = 0.0
        self.publisher_.publish(stop_msg)
        self.get_logger().info("Robot stopped due to Ctrl+C")
# class Ros_Subscriber(Node): # them (Node) de ke thua cai Node, binh thuong chi co class Ros_Publisher:
#     def __init__(self):
#         super().__init__("subscriber_node") 
#         self.get_logger().info("subscriber_node initialized")

#         self.subscriber = self.create_subscription(AckermannDriveStamped, 'move', self.listener_callback, 10) # cho Int16 la message type, count la topic name 

    
#     def listener_callback(self, msg):
#         print("msg: ", msg)

def main():
    rclpy.init()
    node = move_robot_pub()
    def signal_handler(sig, frame):
        node.get_logger().info("SIGINT received! Stopping robot...")
        node.stop_robot()
        rclpy.shutdown()
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.spin(node)
    node.destroy_node()
    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:

    #     pass
    # node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
