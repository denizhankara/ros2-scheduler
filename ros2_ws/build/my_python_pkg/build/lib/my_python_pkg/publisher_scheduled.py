import rclpy
from rclpy.node import Node
from rclpy.utilities import get_default_context
from std_msgs.msg import String
import time
import random
import argparse
from collections import defaultdict
import json
import os
import numpy as np
import yaml
from .priority_node import PriorityNode


with open('/root/ros2-scheduler/ros2_ws/config.yaml') as f:
    d = yaml.safe_load(f)

sampling_rate_mode = d['sampling_rate_mode']  # 0: constant, 1: poisson
sampling_rate_expectation = d['sampling_rate_expectation']
continuity_mode = d['continuity_mode']  # 0: i.i.d, 1: realistic
continuity_max_length = d['continuity_max_length']

topic_queue_size = int(d['topic_queue_size'])
publisher_duration = int(d['publisher_duration'])



class MinimalPublisher(PriorityNode):

    def __init__(self):
        super().__init__('minimal_publisher')
        # self.publisher_H = self.create_publisher(
        #     String, 'topic_H', topic_queue_size)
        # self.publisher_M = self.create_publisher(
        #     String, 'topic_M', topic_queue_size)
        # self.publisher_L = self.create_publisher(
        #     String, 'topic_L', topic_queue_size)
        self.create_publisher_priority(3, String, 'topic', topic_queue_size)
        timer_period = 1.
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # set the start time
        self.publisher_duration = publisher_duration
        self.publisher_start_time = time.time()

        self.i = 0
        if continuity_mode == 1:
            self.cur_priority = None
            self.remaining_length = 0

    def timer_callback(self):

        if sampling_rate_mode == 0:
            sampling_rate = sampling_rate_expectation
        elif sampling_rate_mode == 1:
            sampling_rate = np.random.poisson(sampling_rate_expectation)

        for sampling_index in range(sampling_rate):
            if continuity_mode == 0:
                priority = self.random_priority()
            elif continuity_mode == 1:
                priority = self.realistic_priority()

            msg = String()
            timestamp = time.time()
            message_data = self.i  # sequence number for now

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
            self.publisMessage(msg, priority)

            if sampling_index != sampling_rate - 1:
                time.sleep(1 / sampling_rate)

            # check if the time has exceeded
            self.checkIfTimeExceeded()

    def publisMessage(self, msg, priority):
        # publisher method for any message
        if priority == 'H':
            self.publish_priority(0, 'topic', msg)
        elif priority == 'M':
            self.publish_priority(1,'topic', msg)
        elif priority == 'L':
            self.publish_priority(2, 'topic',msg)

        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def checkIfTimeExceeded(self):
        passed_time = time.time() - self.publisher_start_time
        if passed_time > self.publisher_duration:
            self.get_logger().info('Time exceeded, exiting the publisher...')
            exit()

    def realistic_priority(self):
        if self.remaining_length == 0:
            self.remaining_length = random.randint(1, continuity_max_length)
            self.cur_priority = self.random_priority()
        self.remaining_length -= 1
        return self.cur_priority

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

    def append_record(self, record):
        with open('publisher_records.json', 'a') as f:
            json.dump(record, f)
            f.write(os.linesep)


def main(args=None):

    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    # my_executor = MyExecutor()

    # context = get_default_context()

    # def reset_executor():
    #     my_executor.shutdown()
    #     my_executor = None

    # context.on_shutdown(reset_executor)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
