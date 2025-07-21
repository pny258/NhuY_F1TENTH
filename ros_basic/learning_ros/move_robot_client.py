#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosbasic_msgs.srv import BaiTap


class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__("simple_service_client")
        self.declare_parameter("distance_odom", 0.0)
        self.declare_parameter("distance_lidar", 0.0)
        
        self.age = self.get_parameter("distance_odom").value
        self.name = self.get_parameter("distance_lidar").value
        self.client_ = self.create_client(BaiTap, "BaiTap")

        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("⏳ Waiting for service...")

        self.req_ = BaiTap.Request()
        
        # self.req_.distance_odom = 5.0
        # self.req_.distance_lidar = 2.0

        self.future = self.client_.call_async(self.req_)
        self.future.add_done_callback(self.responseCallback)

    def responseCallback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"✅ Service responded: {response.message}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed: {e}")


def main():
    rclpy.init()
    client_node = SimpleServiceClient()
    rclpy.spin(client_node)
    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from rosbasic_msgs.srv import BaiTap
# import sys 



# class SimpleServiceClient(Node):
#     def __init__(self):
#         super().__init__("simple_service_client")
#         self.client_ = self.create_client(BaiTap, "BaiTap")
        


#         while not self.client_.wait_for_service(timeout_sec=1.0):  # wait for service function, 

#             self.get_logger().info("Service not available, waiting again...")
#         # out loop if wait_for_service = true
        
#         self.req_ = BaiTap.Request() # message type
#         self.req_.max_distance = 5.0
#         self.req_.distance_lidar = 2.0

#         self.future_ = self.client_.call_async(self.req_) 

#         self.future_.add_done_callback(self.serviceCallback)
   
#     def serviceCallback(self, req, _):
#         response = BaiTap.Response()
#         response.ok = True
#         response.message = "🚗 Robot started"
#         return response



# def main():
#     rclpy.init()
   
#     simple_service_client = SimpleServiceClient() # argv[1] only name of the script
#     rclpy.spin(simple_service_client)
#     simple_service_client.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':

#     main()