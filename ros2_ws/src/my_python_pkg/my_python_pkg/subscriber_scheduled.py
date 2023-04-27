import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import time
from collections import defaultdict
import json
import os
import yaml

from my_executor import MyExecutor


with open('/home/ros-scheduler/ros2_ws/config.yaml') as f:
    d = yaml.safe_load(f)

sampling_rate_mode = d['sampling_rate_mode']  # 0: constant, 1: poisson
sampling_rate_expectation = d['sampling_rate_expectation']
continuity_mode = d['continuity_mode']  # 0: i.i.d, 1: realistic
continuity_max_length = d['continuity_max_length']

topic_queue_size = int(d['topic_queue_size'])
subscriber_sample_rate = int(d['subscriber_sample_rate'])


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription_H = self.create_subscription(
            String,
            'topic_H',
            self.listener_callback,
            topic_queue_size)
        self.subscription_M = self.create_subscription(
            String,
            'topic_M',
            self.listener_callback,
            topic_queue_size)
        self.subscription_L = self.create_subscription(
            String,
            'topic_L',
            self.listener_callback,
            topic_queue_size)
        # self.subscription  # prevent unused variable warning
        self.records = []  # defaultdict(dict)
        self.receiver_sequence = 0  # sequence number for receiver, total messages processed

    def listener_callback(self, msg):

        msg_received = msg.data
        # msg.data = '%d-%f' % (self.i, timestamp)
        # parse the message
        msg_received = msg_received.split('-')
        # get the timestamp
        timestamp = float(msg_received[1])
        # get the i
        i = int(msg_received[0])

        # priority
        priority = str(msg_received[2])

        # here, add the messages to different priority queues
        # self.get_logger().info('I heard: "%s"' % i)

        # here, print the timestamp assigned
        # self.get_logger().info('Stamp: "%s"' % timestamp)

        # here, print the difference between the timestamp assigned and the current time
        self.get_logger().info('Difference: "%s"' % (time.time() - timestamp))
        self.get_logger().info('Priority: "%s"' % priority)

        # log the messages and dictionary
        current_message = defaultdict(dict)

        # total messages processed
        current_message['receiver_sequence'] = self.receiver_sequence
        current_message['priority'] = priority
        current_message['timestamp_sender'] = timestamp
        current_message['timestamp_receiver'] = time.time()
        current_message['difference'] = time.time() - timestamp
        current_message['sender_sequence'] = i
        self.receiver_sequence += 1

        self.append_record(current_message)

        # sleep for 1 second
        self.sample_period = 1/subscriber_sample_rate
        time.sleep(self.sample_period)

    def append_record(self, record):
        with open('subscriber_records.json', 'a') as f:
            json.dump(record, f)
            f.write(os.linesep)


def main(args=None):
    if os.path.exists('subscriber_records.json'):
        os.remove('subscriber_records.json')

    if os.path.exists('publisher_records.json'):
        os.remove('publisher_records.json')

    rclpy.init(args=args)

    my_executor = MyExecutor()

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber, my_executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
