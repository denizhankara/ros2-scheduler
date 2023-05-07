from rclpy.node import Node
import math

from typing import Any
from typing import Callable
from typing import Dict
from typing import Iterator
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple
from typing import Type
from typing import TypeVar
from typing import Union

import warnings
import weakref

from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import SetParametersResult

from rclpy.callback_groups import CallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.clock import Clock
from rclpy.clock import ROSClock
from rclpy.constants import S_TO_NS
from rclpy.context import Context
from rclpy.exceptions import InvalidHandle
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.exceptions import InvalidParameterValueException
from rclpy.exceptions import InvalidTopicNameException
from rclpy.exceptions import NotInitializedException
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.exceptions import ParameterImmutableException
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import Executor
from rclpy.expand_topic_name import expand_topic_name
from rclpy.guard_condition import GuardCondition
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.logging import get_logger
from rclpy.parameter import Parameter, PARAMETER_SEPARATOR_STRING
from rclpy.parameter_service import ParameterService
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_parameter_events
from rclpy.qos import qos_profile_services_default
from rclpy.qos import QoSProfile
from rclpy.qos_event import PublisherEventCallbacks
from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.qos_overriding_options import _declare_qos_parameters
from rclpy.qos_overriding_options import QoSOverridingOptions
from rclpy.service import Service
from rclpy.subscription import Subscription
from rclpy.time_source import TimeSource
from rclpy.timer import Rate
from rclpy.timer import Timer
from rclpy.topic_endpoint_info import TopicEndpointInfo
from rclpy.type_support import check_is_valid_msg_type
from rclpy.type_support import check_is_valid_srv_type
from rclpy.utilities import get_default_context
from rclpy.validate_full_topic_name import validate_full_topic_name
from rclpy.validate_namespace import validate_namespace
from rclpy.validate_node_name import validate_node_name
from rclpy.validate_parameter_name import validate_parameter_name
from rclpy.validate_topic_name import validate_topic_name
from rclpy.waitable import Waitable

# Used for documentation purposes only
MsgType = TypeVar('MsgType')
SrvType = TypeVar('SrvType')
SrvTypeRequest = TypeVar('SrvTypeRequest')
SrvTypeResponse = TypeVar('SrvTypeResponse')


class PriorityNode(Node):
    def __init__(self, node_name: str, *, context: Context = None, cli_args: List[str] = None, namespace: str = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: List[Parameter] = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services,
                         parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
        self.priority_topic_names = []

    def create_publisher_priority(self, levels, msg_type, topic: str, qos_profile: QoSProfile | int, *, callback_group: CallbackGroup | None = None, event_callbacks: PublisherEventCallbacks | None = None, qos_overriding_options: QoSOverridingOptions | None = None):
        for i in range(levels):
            publisher = self.create_publisher(msg_type, topic+'_'+str(i), qos_profile, callback_group=callback_group,
                                              event_callbacks=event_callbacks, qos_overriding_options=qos_overriding_options)
            setattr(self, f'publisher_{topic}_{i}', publisher)
            self.priority_topic_names.append(f'{topic}_{i}')


    def publish_priority(self, priority, topic, msg):
        publisher = getattr(self, f'publisher_{topic}_{priority}')
        publisher.publish(msg)

    def create_subscription_priority(self, levels, msg_type, topic: str, callback: Callable[[MsgType], None], qos_profile: QoSProfile | int, *, callback_group: CallbackGroup | None = None, event_callbacks: SubscriptionEventCallbacks | None = None, qos_overriding_options: QoSOverridingOptions | None = None, raw: bool = False):
        for i in range(levels):
            subscription = self.create_subscription(msg_type, topic+'_'+str(i), callback, qos_profile, callback_group=callback_group,
                                                    event_callbacks=event_callbacks, qos_overriding_options=qos_overriding_options, raw=raw)
            setattr(self, f'subscription_{topic}_{i}', subscription)
            self.priority_topic_names.append(f'{topic}_{i}')

