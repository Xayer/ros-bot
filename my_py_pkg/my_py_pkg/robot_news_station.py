#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String


class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station") # topic name

        self.robot_name_ = '3X4'
        self.publisher_ = self.create_publisher(String, 'robot_news', 10)
        # self.creat_publisher(<topic type>,<topic name>,<buffer size> (If some messages are late, up to 10 messages are collected, before being lost.)

        self.timer_ = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("Robot News station has been started")

    def publish_news(self):
        msg = String()
        msg.data = "Hello, this is " + str(self.robot_name_) + "at the robot news station."
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
