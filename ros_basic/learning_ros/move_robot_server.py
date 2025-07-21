#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math
import time
from rosbasic_msgs.srv import BaiTap

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__("simple_service_server")
        self.group = ReentrantCallbackGroup()

        # Khai báo Service, Publisher, Subscriber với callback group
        self.service_ = self.create_service(BaiTap, "BaiTap", self.serviceCallback, callback_group=self.group)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, "/drive", 10, callback_group=self.group)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.lisener, 10, callback_group=self.group)
        self.sub_lidar = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10, callback_group=self.group)

        self.current_pos = None
        self.initialized = None
        self.distance_lidar = None
        self.distance_odom = None
        self.status= True

        self.get_logger().info("Service BaiTap Ready")

    def serviceCallback(self, request, response):
        self.get_logger().info(" Nhận request, bắt đầu di chuyển")
        max_distance = request.distance_odom  # ví dụ: 10.0
        stop_distance = request.distance_lidar # ví dụ: 4.0
        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = 0.0

        while self.current_pos is None:
            self.get_logger().info("Chưa có dữ liệu odom...")
            # time.sleep(0.1)

        start_pos = self.current_pos.copy()

        while self.status:
            dx = self.current_pos[0] - start_pos[0]
            dy = self.current_pos[1] - start_pos[1]
            traveled = math.sqrt(dx**2 + dy**2)

            if traveled >= max_distance:
                self.get_logger().info("Đã đi đủ 20m")
                break

            if self.distance_lidar is None:
                self.get_logger().info(" Đợi giá trị lidar...")
                # time.sleep(0.1)
                continue

            if self.distance_lidar < stop_distance:
                cmd.drive.speed = 0.0
                self.get_logger().info(" Vật cản phát hiện - dừng")
            else:
                cmd.drive.speed = 1.0
                self.get_logger().info(" Đường thông - tiếp tục")

            self.publisher_.publish(cmd)
            #time.sleep(0.1)

        cmd.drive.speed = 0.0
        self.publisher_.publish(cmd)
        self.get_logger().info(" Dừng hoàn toàn")

        response.result = True
        response.message = " Đã hoàn thành di chuyển 20m"
        return response

    def scan_callback(self, scan_msg):
        angle_deg = 0
        angle_rad = math.radians(angle_deg)
        index = int((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)
        if 0 <= index < len(scan_msg.ranges):
            distance = scan_msg.ranges[index]
            if not math.isnan(distance):
                self.distance_lidar = distance
                # print("laser:", distance)

    def lisener(self, msg):
        self.current_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        if self.initialized is None:
            self.initialized = self.current_pos.copy()
        self.distance_odom = math.sqrt((self.current_pos[0] - self.initialized[0])**2 + (self.current_pos[1] - self.initialized[1])**2)
        # print("distance:  ", self.distance_odom)
def main(args=None):
    rclpy.init(args=args)
    node = SimpleServiceServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3
# import rclpy
# import math
# import numpy as np
# from rclpy.node import Node
# from ackermann_msgs.msg import AckermannDriveStamped
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import LaserScan
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rosbasic_msgs.srv import BaiTap
# import time
# from rclpy.executors import MultiThreadedExecutor
# class SimpleServiceServer(Node):
#     def __init__(self):
#         super().__init__("simple_service_server")
#         self.group = ReentrantCallbackGroup()
#         self.service_ = self.create_service(BaiTap, "bai_tap", self.serviceCallback, callback_group=self.group)
#         self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10) # cho Int16 la message type, count la topic name 
#         self.subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10, callback_group=self.group)
#         self.subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10, callback_group=self.group)
#         self.get_logger().info("Service bai_tap Ready")
#         self.initialized = None
#         self.current_pos = None
#         self.distance_odom = 0.0
#         self.distance_lidar = 0.0
#         self.status = True

#     def serviceCallback(self, req, res):
#         max_distance = req.max_distance
#         stop_distance = req.distance_lidar
#         speed = req.speed

#         self.status = True
#         self.initialized = None  # reset lại để tính lại từ đầu

#         while rclpy.ok() and self.status:
#             rclpy.spin_once(self, timeout_sec=0.1)

#             if self.distance_lidar is None or self.current_pos is None:
#                 continue

#             drive_msg = AckermannDriveStamped()
#             drive_msg.drive.speed = 0.0

#             if self.distance_odom >= max_distance:
#                 self.status = False
#                 res.message = "Reached max distance"
#                 drive_msg.drive.speed = 0.0
#                 self.publisher_.publish(drive_msg)
#                 res.ok = True
#                 res.distance_travelled = self.distance_odom
#                 break

#             elif self.distance_lidar <= stop_distance:
#                 self.get_logger().info("Obstacle detected. Stopping.")
#                 drive_msg.drive.speed = 0.0
#             else:
#                 drive_msg.drive.speed = speed

#             self.publisher_.publish(drive_msg)

#         return res

#     def odom_callback(self, msg):
#        # print("msg: ", msg.pose.pose.position)
#         self.current_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
#         # print(self.current_pos)
#         if self.initialized is None:
#             self.initialized = self.current_pos

#         dx = self.current_pos[0] - self.initialized[0]
#         dy = self.current_pos[1] - self.initialized[1]
#         self.distance_odom  = math.sqrt(dx**2 + dy**2)

#     def scan_callback(self, scan_msg:LaserScan):
#         angle_deg = 0
#         for idx, r in enumerate(scan_msg.ranges):
#             if(np.isnan(r) or r > scan_msg.range_max or r < scan_msg.range_min):
#                 continue
#         angle_rad = math.radians(angle_deg)
#         index = int((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)
#         self.distance_lidar = scan_msg.ranges[index]
#         print("laser: ",self.distance_lidar)   

# def main(args=None):
#     rclpy.init(args=args)
#     try:
#         simple_service_server = SimpleServiceServer()
#         executor = MultiThreadedExecutor(num_threads=3)
#         executor.add_node(simple_service_server)
#         try:
#             executor.spin()
#         finally:
#             executor.shutdown()
#             simple_service_server.destroy_node()
#     finally:
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()




# #!/usr/bin/env python3
# import rclpy
# import math
# import numpy as np
# from rclpy.node import Node
# from ackermann_msgs.msg import AckermannDriveStamped
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import LaserScan
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rosbasic_msgs.srv import BaiTap
# import time
# from rclpy.executors import MultiThreadedExecutor
# class SimpleServiceServer(Node):
#     def __init__(self):
#         super().__init__("simple_service_server")
#         self.group = ReentrantCallbackGroup()
#         self.service_ = self.create_service(BaiTap, "bai_tap", self.serviceCallback, callback_group=self.group)
#         self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10) # cho Int16 la message type, count la topic name 
#         self.subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10, callback_group=self.group)
#         self.subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10, callback_group=self.group)
#         self.get_logger().info("Service bai_tap Ready")
#         self.initialized = None
#         self.current_pos = None
#         self.distance_odom = 0.0
#         self.distance_lidar = 0.0
#         self.status = True

#     def serviceCallback(self, req, res):
#         max_distance = req.max_distance
#         distance_lidar = req.distance_lidar
#         speed = req.speed

#         while self.status:
#             time.sleep(0.1)
#             # print("laser_test: ",self.distance_lidar)  
#             if self.distance_odom is None or self.distance_lidar is None:
#                 return
#             if self.distance_odom > max_distance:
#                 speed = 0.0
#                 self.status = False
#                 res.message = "finished"
#                 res.distance_travelled = max_distance
#             elif self.distance_lidar <= distance_lidar:
#                 print("STOP")
#                 speed = 0.0
#             else:
#                 speed = 1.0

#         res.ok = True
#         res.distance_travelled = 1.5

#         return res
#     def odom_callback(self, msg):
#        # print("msg: ", msg.pose.pose.position)
#         self.current_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
#         # print(self.current_pos)
#         if self.initialized is None:
#             self.initialized = self.current_pos

#         dx = self.current_pos[0] - self.initialized[0]
#         dy = self.current_pos[1] - self.initialized[1]
#         self.distance_odom  = math.sqrt(dx**2 + dy**2)

#     def scan_callback(self, scan_msg:LaserScan):
#         angle_deg = 0
#         for idx, r in enumerate(scan_msg.ranges):
#             if(np.isnan(r) or r > scan_msg.range_max or r < scan_msg.range_min):
#                 continue
#         angle_rad = math.radians(angle_deg)
#         index = int((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)
#         self.distance_lidar = scan_msg.ranges[index]
#         print("laser: ",self.distance_lidar)   

# def main(args=None):
#     rclpy.init(args=args)
#     try:
#         simple_service_server = SimpleServiceServer()
#         executor = MultiThreadedExecutor(num_threads=3)
#         executor.add_node(simple_service_server)
#         try:
#             executor.spin()
#         finally:
#             executor.shutdown()
#             simple_service_server.destroy_node()
#     finally:
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



