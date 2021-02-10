from .exceptions import *
from rclpy.service import Service
from std_msgs.msg import String
from std_srvs.srv import Trigger
from splash_interfaces.srv import RegisterMode, RequestModeChange
import json
class ModeChangeInputPort:
    def __init__(self, component):
        self._component = component
        # print("Mode change input port:", component.name)
        self._request_register_mode()
        # print(f"splash_mode_change_{component.factory.name}")
        self._subsription = self._component.create_subscription(String, "splash_mode_change_{}".format(component.factory.name), self._mode_change_callback, 1)

    def _mode_change_callback(self, msg):
        # print(f"mode change callback: {msg.data} (current mode: {self._component.get_current_mode()})")
        if self._component.get_current_mode() != msg.data:
            self._component.get_logger().info("Change mode {} => {}".format(self._component.get_current_mode(), msg.data))
            self._component.set_current_mode(msg.data)
        else:
            # print("what the fuck")
            pass

    def _request_register_mode(self):
        self._component.get_logger().info('Mode registration...')
        _cli = self._component.create_client(RegisterMode, '/register_splash_mode')
        if not _cli.wait_for_service(timeout_sec=1.0):
            # raise ModeManagerAbsenceException
            self._component.get_logger().error("Splash server is not running. Please rerun splash server")
        _req = RegisterMode.Request()
        _req.name_space = self._component.get_namespace()
        _req.factory = self._component.factory.name
        _req.mode_configuration = json.dumps(self._component.factory.mode_configuration)
        future = _cli.call_async(_req)
        self._component.get_logger().info('Mode registration OK')
        

class ModeChangeOutputPort:
    def __init__(self, component, factory):
        self._component = component
        self._factory = factory
        self._client = self._component.create_client(RequestModeChange, "/request_splash_mode_change")
        self._event = None

    def trigger(self, event):
        _req = RequestModeChange.Request()
        _req.factory = self._factory
        _req.event = event
        self._event = event
        future = self._client.call_async(_req)
        future.add_done_callback(self.done_callback)
        return future
        
    def done_callback(self, future):
        result = future.result()
        if result.ok:
            self._component.get_logger().info("Mode change({}) service successfully done".format(self._event))
        else:
            self._component.get_logger().error("invalid mode change event: {}".format(self._event))

class EventInputPort:
    def __init__(self, component, event, callback):
        self._component = component
        self._event = event
        self._callback = callback
        self._service = self._component.create_service(Trigger, event, self._check_mode_and_execute_callback)

    def _check_mode_and_execute_callback(self, msg):
        if self._component.mode == self._component.get_current_mode():
            self._callback(msg)
        else:
            pass

class EventOutputPort:
    def __init__(self, component, event):
        self._component = component
        self._event = event
        self._client = self._component.create_client(Trigger, event)

    def trigger(self):
        _req = Trigger.Request()
        future = self._client.call_async(_req)
        return future
            
