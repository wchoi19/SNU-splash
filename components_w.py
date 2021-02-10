import json, time
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from .port_w import Port
from .channel import StreamInputPort, StreamOutputPort
from .clink import EventInputPort, EventOutputPort, ModeChangeInputPort, ModeChangeOutputPort
from .exceptions import *
from .impl.msg_converter import convert_ros_message_to_dictionary, convert_dictionary_to_ros_message
from splash_interfaces.srv import UnregisterMode
import inspect
import json


class Component(Node):
    def __init__(self, name):
        self._name = name    

    def attach(self, port):
        raise NotImplementedError()

    def write(self, port):
        raise NotImplementedError()

    def _attach_input_port(self, port):
        raise NotImplementedError()

    def _attach_output_port(self, port):
        raise NotImplementedError()
  

class ProcessingComponent(Component):
    def __init__(self, name):
        super().__init__(name)
        self._stream_input = {}
        self._stream_output = {}
        self._event_input = {}
        self._event_output = {}
        self._mode_input = {}
        self._mode_output = {}

    def attach(self, port):
        port.attach_component(self)
        if port.get_inout() == 'in':
            self._attach_input_port(port)
        else:    
            self._attach_output_port(port)

    def _attach_input_port(self, port):

        if port.get_port_type() == 'stream':
            self._stream_input[port.get_channel()] = port

        elif port.get_port_type() == 'event':
            self._event_input[port.get_clink()] = port    
            
        elif port.get_port_type() == 'mode':
            self._mode_input[port.get_clink()] = port     
    
    def _attach_output_port(self, port):     
        if port.get_port_type() == 'stream':
            self._stream_output[port.get_channel()] = port

        elif port.get_port_type() == 'event':
            self._event_output[port.get_clink()] = port    
            
        elif port.get_port_type() == 'mode':
            self._mode_output[port.get_clink()] = port                   

    def write(self, port, msg):
        if port.get_inout() == 'out':
            if port.get_port_type() == 'stream':
                self._stream_output[port.get_channel()].write(msg)            
            elif port.get_port_type() == 'event':
                self._event_output[port.get_clink()].write(msg)    
            else:
                self._mode_output[port.get_clink()].write(msg)  

        
class SourceComponent(Component):
    def __init__(self, name):
        super().__init__(name)
        self._stream_output = None

    def attach(self, port):
        port.attach_component(self)

        if port.get_inout() == 'in':
            self._attach_input_port(port)
        else:    
            self._attach_output_port(port)        
    
    def _attach_output_port(self, port):
        
        if port.get_port_type() == 'stream':
            self._stream_output = port

    def write(self, port, msg):
        if port.get_inout() == 'out':
            if port.get_port_type() == 'stream':
                self._stream_output.write(msg)        

class SinkComponent(Component):
    def __init__(self, name):
        super().__init__(name)
        self._stream_input = None
    
    def attach(self, port):
        port.attach_component(self)

        if port.get_inout() == 'in':
            self._attach_input_port(port)
        else:    
            self._attach_output_port(port)

    def _attach_input_port(self, port):
        if port.get_port_type() == 'stream':
            self._stream_input = port


class FusionOperator(Component):        
    def __init__(self):
        super().__init__(name)
        self._stream_input = {}
        self._stream_output = None

    def attach(self, port):
        port.attach_component(self)

        if port.get_inout() == 'in':
            self._attach_input_port(port)
        else:    
            self._attach_output_port(port)
   
    def _attach_input_port(self, port):
        if port.get_port_type() == 'stream':
            self._stream_input[port.get_channel()] = port

    def _attach_output_port(self, port):
        if port.get_port_type() == 'stream':
            self._stream_output = port

    def write(self, port, msg):
        if port.get_inout() == 'out':
            if port.get_port_type() == 'stream':
                self._stream_output.write(msg)   

    def _do_fusion_operation():
        TODO: fusion operating stuffs                       


    
