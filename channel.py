from srl.rate_controller import RateController
from std_msgs.msg import String, Header
from .impl.msg_converter import convert_ros_message_to_dictionary, convert_dictionary_to_ros_message
from .exceptions import *

from splash_interfaces.msg import SplashMessage
import json
from rclpy.time import Time
from .exceptions import *



class StreamPort():
    def __init__(self, name, parent):
        self.name = name
        self.parent = parent
        self._namespace = None

    def get_channel(self):
        return self._channel
        
    def set_channel(self, channel):
        self._channel = channel

    def get_msg_type(self):
        return self._msg_type
    def set_msg_type(self, msg_type):
        self._msg_type = msg_type

    def set_namespace(self, namespace):
        self._namespace = namespace

    def get_namespace(self):
        return self._namespace

class StreamInputPort(StreamPort):
    class FusionedObj(object):
        def __init__(self, d):
            for key, value in d.items():
                if isinstance(value, (list, tuple)):
                    setattr(self, key, [StreamInputPort.FusionedObj(x) if isinstance(x, dict) else x for x in value])
                else:
                    setattr(self, key, StreamInputPort.FusionedObj(value) if isinstance(value, dict) else value)

    def __init__(self, name, parent):
        super().__init__(name, parent)
        self.msg_list = []
        self.from_fusion = False

    def attach(self):
        topic = self._namespace + "/" + self._channel if self._namespace else self._channel
        self._subscription = self.parent.create_subscription(
            SplashMessage, topic, self._check_mode_and_execute_callback, 1)

    def set_callback(self, callback, args=None):
        self._callback = callback
        self._args = args

    def get_callback(self):
        return self._callback

    def _check_mode_and_execute_callback(self, msg):
        if self.parent.mode == self.parent.get_current_mode():
            # self.parent.get_logger().info(msg.body)
            # self.parent.get_logger().info(msg.header.frame_id)
            if msg.freshness:
                # self.parent.get_logger().info("freshness: {}".format(msg.freshness))
                time_exec_ms = (self.parent.get_clock().now().nanoseconds - Time.from_msg(msg.header.stamp).nanoseconds) / 1000000
                # self.parent.get_logger().info("time_exec: {}".format(time_exec_ms))
                if time_exec_ms > msg.freshness:
                    self.get_logger.warn('{}ms exceeded(constraint: {}ms, cur: {}ms'.format(time_exec_ms - msg.freshness, msg.freshness, time_exec_ms))
            self.msg_list.append(msg)
            msg_decoded = json.loads(msg.body)
            msg_converted = convert_dictionary_to_ros_message(self._msg_type, msg_decoded)
            if self._args:
                self._callback(msg, self._args[0])
            else:
                if self.from_fusion:
                    # self.parent.get_logger().info("{}".format(msg_converted))
                    msg_obj = self.FusionedObj(json.loads(msg_converted.data))
                    self._callback(msg_obj)
                else:
                    self._callback(msg_converted)
            self.msg_list.pop(0)
        else:
            pass

class StreamOutputPort(StreamPort):
    def __init__(self, name, parent):
        super().__init__(name, parent)
        self._rate_constraint = 0

    def attach(self):
        self._topic = self._namespace + "/" + self._channel if self._namespace else self._channel
        self._publisher = self.parent.create_publisher(
            SplashMessage, self._topic, 1)
        if self._rate_constraint > 0:
            self._rate_controller = RateController(self)
            self._rate_controller.exception = FreshnessConstraintViolationException

    def set_rate_constraint(self, rate_constraint):
        self._rate_constraint = rate_constraint

    def get_rate_constraint(self):
        return self._rate_constraint

    def write(self, msg, source_msg=None):
        if self.parent.mode == self.parent.get_current_mode():
            msg_splash = SplashMessage()
            if source_msg is None:
                msg_splash.header = Header()
                msg_splash.header.stamp = self.parent.get_clock().now().to_msg()
                msg_splash.header.frame_id = self.parent.name
                if self.parent.freshness != None and self.parent.freshness != 0:
                    msg_splash.freshness = self.parent.freshness
            else:
                msg_splash.header = source_msg.header
                if(source_msg.freshness):
                    msg_splash.freshness = source_msg.freshness
            msg_splash.body = json.dumps(convert_ros_message_to_dictionary(msg))
            if self._rate_constraint > 0:
                self._rate_controller.push(msg_splash)
            else:
                self._publisher.publish(msg_splash)
        else:
            pass

    def get_publisher(self):
        return self._publisher