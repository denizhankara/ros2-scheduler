import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import time
import random
import argparse
from collections import defaultdict
import json
import os
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        timestamp = time.time()
        message_data = self.i # sequence number for now
        priority = self.random_priority()

        # add data, time and i to the message
        msg.data = '%d-%f-%s' % (message_data, timestamp, priority)

        # log the messages and dictionary
        current_message = defaultdict(dict)
        current_message['priority'] = priority
        current_message['timestamp_sender'] = timestamp
        current_message['timestamp_receiver'] = time.time()
        current_message['difference'] = time.time() - timestamp
        current_message['sender_sequence'] = self.i
        self.append_record(current_message)

        # publish the message
        self.publisMessage(msg)

    def publisMessage(self,msg):
        ## publisher method for any message
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def random_priority(self):
        # generate a random number between 0 and 1
        # if the number is less than 0.1, return H
        # if the number is between 0.1 and 0.2, return M
        # if the number is between 0.2 and 1, return L

        # generate a random number between 0 and 1
        rand = random.random()
        # if the number is less than 0.1, return H
        if rand < 0.1:
            return 'H'
        # if the number is between 0.1 and 0.2, return M
        elif rand < 0.2:
            return 'M'
        # if the number is between 0.2 and 1, return L
        else:
            return 'L'
    
    def append_record(self,record):
        with open('publisher_records.json', 'a') as f:
            json.dump(record, f)
            f.write(os.linesep)
    
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()