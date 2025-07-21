from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    path = os.path.join(get_package_share_directory("ros_basic"), "config", "robot.yaml")

       
#get_package_share_directory("ros_basic")+"/config/robot.yaml"

    client_robot_node = Node(
        package="ros_basic",
        executable="move_robot_client.py",
        name="simple_service_client",
        output="screen",
        parameters=[
            path
        ],
    )

    server_robot_node = Node(
        package="ros_basic",
        executable="move_robot_server.py",
        name="simple_service_server",
        output="screen"
    )


    return LaunchDescription([
        client_robot_node,
        server_robot_node
    ])
