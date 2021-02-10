# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import base64
import copy
from rclpy.clock import Clock
from rclpy.duration import Duration
import numpy as np
python_string_types = [str]
python_int_types = [int]
python_float_types = [float]
ros_primitive_types = ['bool', 'byte', 'char', 'int8', 'uint8', 'int16',
                       'uint16', 'int32', 'uint32', 'int64', 'uint64', 'float',
                       'float32', 'float64', 'string']
ros_time_types = ['time', 'duration']
ros_header_types = ['Header', 'std_msgs/Header', 'roslib/Header']
ros_to_python_type_map = {
    'bool'    : [bool],
    'float32' : python_float_types + python_int_types + [np.float32, np.int8, np.int16, np.uint8, np.uint16],
       # don't include int32, because conversion to float may change value: v = np.iinfo(np.int32).max; np.float32(v) != v
    'float64' : python_float_types + python_int_types + [np.float32, np.float64, np.int8, np.int16, np.int32, np.uint8, np.uint16, np.uint32],
    'int8'    : python_int_types + [np.int8],
    'int16'   : python_int_types + [np.int8, np.int16, np.uint8],
    'int32'   : python_int_types + [np.int8, np.int16, np.int32, np.uint8, np.uint16],
    'int64'   : python_int_types + [np.int8, np.int16, np.int32, np.int64, np.uint8, np.uint16, np.uint32],
    'uint8'   : python_int_types + [np.uint8],
    'uint16'  : python_int_types + [np.uint8, np.uint16],
    'uint32'  : python_int_types + [np.uint8, np.uint16, np.uint32],
    'uint64'  : python_int_types + [np.uint8, np.uint16, np.uint32, np.uint64],
    'byte'    : python_int_types + [np.int8],
    'char'    : python_int_types + [np.uint8],
    'string'  : python_string_types
}
def convert_ros_message_to_dictionary(message):
    """
    Takes in a ROS message and returns a Python dictionary.
    Example:
        ros_message = std_msgs.msg.String(data="Hello, Robot")
        dict_message = convert_ros_message_to_dictionary(ros_message)
    """
    dictionary = {}
    message_fields = message.get_fields_and_field_types().items()
    for field_name, field_type in message_fields:
        field_value = getattr(message, field_name)
        dictionary[field_name] = _convert_from_ros_type(field_type, field_value)

    return dictionary

def convert_dictionary_to_ros_message(message_type, dictionary):
    """
    Takes in the message type and a Python dictionary and returns a ROS message.
    Example:
        message_type = "std_msgs/String"
        dict_message = { "data": "Hello, Robot" }
        ros_message = convert_dictionary_to_ros_message(message_type, dict_message)
    """
    message = message_type()
    message_fields = message.get_fields_and_field_types()
    remaining_message_fields = copy.deepcopy(message_fields)
    for field_name, field_value in dictionary.items():
        if field_name in message_fields:
            field_type = message_fields[field_name]
            field_value = _convert_to_ros_type(field_name, field_type, field_value)
            setattr(message, field_name, field_value)
            del remaining_message_fields[field_name]
        else:
            error_message = 'ROS message type "{0}" has no field named "{1}"'\
                .format(message_type, field_name)
            if strict_mode:
                raise ValueError(error_message)
            else:
                print('{}! It will be ignored.'.format(error_message))
    return message

def _convert_to_ros_type(field_name, field_type, field_value):
    if _is_ros_binary_type(field_type):
        field_value = _convert_to_ros_binary(field_type, field_value)
    elif field_type in ros_time_types:
        field_value = _convert_to_ros_time(field_type, field_value)
    elif field_type in ros_primitive_types:
        pass
    elif _is_field_type_a_primitive_array(field_type):
        field_value = field_value
    elif _is_field_type_an_array(field_type):
        field_value = _convert_to_ros_array(field_name, field_type, field_value)
    else:
        field_value = convert_dictionary_to_ros_message(field_type, field_value)

    return field_value

def _convert_to_ros_binary(field_type, field_value):
    if type(field_value) in python_string_types:
        binary_value_as_string = base64.standard_b64decode(field_value)
    elif python3:
        binary_value_as_string = bytes(bytearray(field_value))
    else:
        binary_value_as_string = str(bytearray(field_value))
    return binary_value_as_string

def _convert_from_ros_type(field_type, field_value):
    if field_type in ros_primitive_types:
        field_value = field_value
    elif field_type in ros_time_types:
        field_value = _convert_from_ros_time(field_type, field_value)
    elif _is_ros_binary_type(field_type):
        field_value = _convert_from_ros_binary(field_type, field_value)
    elif _is_field_type_a_primitive_array(field_type):
        field_value = list(field_value)
    elif _is_field_type_an_array(field_type):
        field_value = _convert_from_ros_array(field_type, field_value)
    else:
        field_value = convert_ros_message_to_dictionary(field_value)

    return field_value

def _convert_to_ros_time(field_type, field_value):
    time = None
    clock = Clock()
    if field_type == 'time' and field_value == 'now':
        time = clock.now()
    else:
        if field_type == 'time':
            time = clock.now()
        elif field_type == 'duration':
            time = Duration()
        if 'secs' in field_value:
            setattr(time, 'secs', field_value['secs'])
        if 'nsecs' in field_value:
            setattr(time, 'nsecs', field_value['nsecs'])

    return time
    
def _convert_from_ros_time(field_type, field_value):
    field_value = {
        'secs'  : field_value.secs,
        'nsecs' : field_value.nsecs
    }
    return field_value

def _convert_from_ros_binary(field_type, field_value):
    field_value = base64.standard_b64encode(field_value).decode('utf-8')
    return field_value

def _convert_from_ros_array(field_type, field_value):
    # use index to raise ValueError if '[' not present
    list_type = field_type[:field_type.index('[')]
    return [_convert_from_ros_type(list_type, value) for value in field_value]

def _is_ros_binary_type(field_type):
    """ Checks if the field is a binary array one, fixed size or not
    """
    return field_type.startswith('uint8[') or field_type.startswith('char[')

def _is_field_type_an_array(field_type):
    return field_type.find('[') >= 0

def _is_field_type_a_primitive_array(field_type):
    bracket_index = field_type.find('[')
    if bracket_index < 0:
        return False
    else:
        list_type = field_type[:bracket_index]
        return list_type in ros_primitive_types

def _convert_to_ros_array(field_name, field_type, list_value):
    # use index to raise ValueError if '[' not present
    list_type = field_type[:field_type.index('[')]
    return [_convert_to_ros_type(field_name, list_type, value) for value in list_value]