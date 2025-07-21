#!/usr/bin/env python3
from enum import Enum
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool 
from visualization_msgs.msg import Marker, MarkerArray
import math

class State(Enum):
    FREE = 0
    WARNING = 1
    DANGER = 2

class SafetyStop(Node):
    def __init__(self):
        super().__init__("safety_stop_node")

        # Declare and get parameters
        self.declare_parameter("danger_distance", 0.5)
        self.declare_parameter("warning_distance", 0.8)
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("safety_stop_topic", "safety_stop")

        self.danger_distance = self.get_parameter("danger_distance").get_parameter_value().double_value
        self.warning_distance = self.get_parameter("warning_distance").get_parameter_value().double_value
        self.scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.safety_stop_topic = self.get_parameter("safety_stop_topic").get_parameter_value().string_value

        # Initial states
        self.state = State.FREE
        self.prev_state = State.FREE
        self.is_first_msg = True

        # ROS2 communication
        self.laser_sub = self.create_subscription(LaserScan, self.scan_topic, self.laser_callback, 10)
        self.safety_stop_pub = self.create_publisher(Bool, self.safety_stop_topic, 10)
        self.zones_pub = self.create_publisher(MarkerArray, "zones_Y", 10)

        # Initialize zones
        self.zones = MarkerArray()

        # WARNING marker (ẩn)
        warning_zone = Marker()
        warning_zone.id = 0
        warning_zone.type = Marker.CYLINDER
        warning_zone.action = Marker.ADD
        warning_zone.scale.z = 0.001
        warning_zone.scale.x = self.warning_distance * 2
        warning_zone.scale.y = self.warning_distance * 2
        warning_zone.color.r = 1.0
        warning_zone.color.g = 0.984
        warning_zone.color.b = 0.0
        warning_zone.color.a = 0.0  # Sẽ bị ẩn

        # DANGER marker
        danger_zone = Marker()
        danger_zone.id = 1
        danger_zone.type = Marker.CYLINDER
        danger_zone.action = Marker.ADD
        danger_zone.scale.z = 0.001
        danger_zone.scale.x = self.danger_distance * 2
        danger_zone.scale.y = self.danger_distance * 2
        danger_zone.color.r = 1.0
        danger_zone.color.g = 0.0
        danger_zone.color.b = 0.0
        danger_zone.color.a = 0.0  # sẽ bị ẩn lúc đầu, sau khi kcach gần thì mới hiện lên màu đỏ
        danger_zone.pose.position.z = 0.01
        ##
        #TEXT marker ("EIU")
        text_marker = Marker()
        text_marker.id = 2
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.scale.z = 1.0 #chiều cao chữ
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0  # sẽ bị ẩn lúc đầu, sau khi kcach gần thì mới hiện lên màu đỏ
        text_marker.pose.position.z = 1.0
        text_marker.pose.orientation.w = 1.0
        text_marker.text = "Eastern International University"
        ##
        self.zones.markers = [warning_zone, danger_zone, text_marker]

    def laser_callback(self, msg: LaserScan):
        self.state = State.FREE

        # Check laser scan values to determine state
        for range_value in msg.ranges:
            if not math.isinf(range_value) and range_value <= self.warning_distance:
                self.state = State.WARNING
                if range_value < self.danger_distance:
                    self.state = State.DANGER
                    break

        # Publish safety stop signal
        is_safety_stop = Bool()
        is_safety_stop.data = (self.state == State.DANGER)
        self.safety_stop_pub.publish(is_safety_stop)

        # Set header on first message
        if self.is_first_msg:
            for zone in self.zones.markers:
                zone.header.frame_id = msg.header.frame_id
            self.is_first_msg = False

        # Control marker visibility
        if self.state == State.DANGER:
            self.zones.markers[0].color.a = 0.0  # Hide WARNING zone
            self.zones.markers[1].color.a = 1.0  # Show DANGER zone
        else:
            self.zones.markers[0].color.a = 0.0
            self.zones.markers[1].color.a = 0.0  # Hide all

        # Publish markers
        self.zones_pub.publish(self.zones)


def main():
    rclpy.init()
    safety_stop_node = SafetyStop()
    rclpy.spin(safety_stop_node)
    safety_stop_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
