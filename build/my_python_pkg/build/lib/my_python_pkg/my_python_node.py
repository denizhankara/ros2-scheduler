# build code
# colcon build

# if package not found:
# source install/setup.bash

import rclpy
from rclpy.node import Node
import time
import random
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.create_timer(1, self.timer_callback)
    def timer_callback(self):
        self.get_logger().info("Hello ROS2")
        current_number = random.random()
        self.get_logger().info(str(current_number))


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
