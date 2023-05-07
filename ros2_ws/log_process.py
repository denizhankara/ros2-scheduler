import json
import os
import time
from collections import defaultdict
import argparse
from ast import literal_eval


def read_records(file_name):
    with open(file_name, 'r') as f:
        # read lines of dictionaries from the file
        # records = [json.loads(line) for line in f]
        records =f.readlines()
    return records

def calculate_drop_rate(sender_messages,receiver_messages):
    priorities = ['H','M','L']
    # calculate drop rate of messages based on priority

    # total number of messages sent from each priority
    sent_messages = defaultdict(int)
    # total number of messages dropped from each priority
    drop_rate = defaultdict(int)

    for message in sender_messages:
        # convert the string to a dictionary
        message = literal_eval(message)
        if type(message) is not dict:
            message = message[0]
        # parse the message
        priority = message['priority']
        
        # increment the total number of messages sent
        sent_messages[priority] += 1

    received_messages = defaultdict(int)

    for message in receiver_messages:
        message = literal_eval(message)
        if type(message) is not dict:
            message = message[0]
        # parse the message
        priority = message['priority']
        
        # increment the total number of messages received
        received_messages[priority] += 1
        
    # calculate the drop rate for each priority
    for priority in priorities:
        # calculate the drop rate
        drop_rate[priority] = 1 - (received_messages[priority] / sent_messages[priority])
        # print the drop rate
        print(drop_rate[priority])

    return drop_rate

def calculate_average_delay(sender_messages,receiver_records):
    # calculate average delay of messages based on priority
    priorities = ['H','M','L']
    # total delay of messages received from each priority
    total_delay = defaultdict(int)
    # total number of messages received from each priority
    received_messages = defaultdict(int)

    for message in receiver_records:
        # convert the string to a dictionary
        message = literal_eval(message)
        if type(message) is not dict:
            message = message[0]
        # parse the message
        priority = message['priority']
        difference = message['difference']

        # increment the total number of messages received
        received_messages[priority] += 1
        # increment the total delay
        total_delay[priority] += difference

    # calculate the average delay for each priority
    for priority in priorities:
        # calculate the average delay
        average_delay = total_delay[priority] / received_messages[priority]
        # print the average delay
        print(average_delay)

    return total_delay, received_messages

    pass

def main(args=None):
    base_path = "/root/ros2-scheduler/ros2_ws"
    subscriber_path = os.path.join(base_path, "subscriber_records.json")
    publisher_path = os.path.join(base_path, "publisher_records.json")
    # read the records of subscriber and publisher
    publisher_records = read_records(publisher_path)
    subscriber_records = read_records(subscriber_path)
    
    # calculate the drop rate
    calculate_drop_rate(publisher_records,subscriber_records)

    # calculate the average delay
    calculate_average_delay(publisher_records,subscriber_records)




if __name__ == '__main__':
    main()