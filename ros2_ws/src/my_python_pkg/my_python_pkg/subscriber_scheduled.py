import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import time
from collections import defaultdict
import json
import os
import yaml
from rclpy.executors import *
from rclpy.context import Context
from rclpy.subscription import Subscription
from collections import deque
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from .priority_node import PriorityNode


with open('/root/ros2-scheduler/ros2_ws/config.yaml') as f:
    d = yaml.safe_load(f)

sampling_rate_mode = d['sampling_rate_mode']  # 0: constant, 1: poisson
sampling_rate_expectation = d['sampling_rate_expectation']
continuity_mode = d['continuity_mode']  # 0: i.i.d, 1: realistic
continuity_max_length = d['continuity_max_length']

topic_queue_size = int(d['topic_queue_size'])
subscriber_sample_rate = int(d['subscriber_sample_rate'])


class MinimalSubscriber(PriorityNode):

    def __init__(self):
        super().__init__('minimal_subscriber')
        # my_callback_group = ReentrantCallbackGroup()

        # self.subscription_H = self.create_subscription(
        #     String,
        #     'topic_H',
        #     self.listener_callback,
        #     topic_queue_size,
        #     callback_group=my_callback_group)
        # self.subscription_M = self.create_subscription(
        #     String,
        #     'topic_M',
        #     self.listener_callback,
        #     topic_queue_size,
        #     callback_group=my_callback_group)
        # self.subscription_L = self.create_subscription(
        #     String,
        #     'topic_L',
        #     self.listener_callback,
        #     topic_queue_size,
        #     callback_group=my_callback_group)
        self.create_subscription_priority(3, String,
                                          'topic',
                                          self.listener_callback,
                                          topic_queue_size,)
                                        #   callback_group=my_callback_group)
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


class MyExecutor(Executor):
    def __init__(self, priority_topic_names, *, context: Context = None) -> None:
        super().__init__(context=context)
        self.priority_topic_names = priority_topic_names
        self.queue_else = deque(maxlen=1000)
        for name in priority_topic_names:
            setattr(self, 'queue_'+name, deque(maxlen=topic_queue_size))

        self.callback_empty = True

    def spin_once(self, timeout_sec: float = None) -> None:

        while True:
            try:
                timeout_sec = None if self.callback_empty else 0
                handler, entity, node = self.wait_for_ready_callbacks(
                    timeout_sec=timeout_sec)
            except ShutdownException:
                pass
            except TimeoutException:
                break
            else:
                self.callback_empty = False
                if isinstance(entity, Subscription):
                    topic_name = entity.topic_name
                    if topic_name in self.priority_topic_names:
                        getattr(self, 'queue_'+topic_name).append(handler)
                    else:
                        print('something appended to queue_else')
                        self.queue_else.append(handler)
                else:
                    self.queue_else.append(handler)

        # print(len(self.queue_else),len(self.queue_H),len(self.queue_M),len(self.queue_L))

        if len(self.queue_else) != 0:
            handler = self.queue_else.popleft()
            self.callback_empty = False

        else:
            # you should block next time
            self.callback_empty = True
            for name in self.priority_topic_names:
                queue = getattr(self, 'queue_'+name)
                if len(queue) != 0:
                    handler = queue.popleft()
                    self.callback_empty = False
                    break

        if self.callback_empty is False:
            handler()
            if handler.exception() is not None:
                raise handler.exception()


def main(args=None):
    if os.path.exists('subscriber_records.json'):
        os.remove('subscriber_records.json')

    if os.path.exists('publisher_records.json'):
        os.remove('publisher_records.json')

    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    my_executor = MyExecutor(minimal_subscriber.priority_topic_names)

    rclpy.spin(minimal_subscriber, my_executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
