import json, time
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from .channel import StreamInputPort, StreamOutputPort
from .clink import EventInputPort, EventOutputPort, ModeChangeInputPort, ModeChangeOutputPort
from .exceptions import *
from .impl.msg_converter import convert_ros_message_to_dictionary, convert_dictionary_to_ros_message
from splash_interfaces.srv import UnregisterMode
import inspect
import json
class Component(Node):
    def __init__(self, name, factory, mode):
        self.name = name
        self.factory = factory
        self.mode = mode
        self._current_mode = None
        self._stream_input_ports = {}
        self._stream_output_ports = {}
        self._mode_input_port = None
        self._mode_output_ports = {}
        self._event_input_ports = {}
        self._event_output_ports = {}
        self.build_unit = None
        self.freshness = None
        namespace = factory.get_namespace() if factory else ""
        self._namespace = namespace + '/' + \
            mode.lower().replace(" ", "_") if mode else namespace

    def destroy_node(self):
        if self.mode is not None:
            self.get_logger().info('Mode unregistration...')
            _cli = self.create_client(UnregisterMode, '/unregister_splash_mode')
            _req = UnregisterMode.Request()
            _req.factory = self.factory.name
            future = _cli.call_async(_req)
        return super().destroy_node()
        
    def set_current_mode(self, mode):
        self._current_mode = mode

    def get_current_mode(self):
        return self._current_mode

    def set_build_unit(self, build_unit):
        self.build_unit = build_unit
        self._create_node()
        if self.factory and self.factory.mode_configuration:
            self.set_current_mode(self.factory.mode_configuration["initial_mode"])
            print("set_build_unit, initial mode is ", self.factory.mode_configuration["initial_mode"])
            self.attach_modechange_input_port()

    def _create_node(self):
        super().__init__(self.name, context=self.build_unit.context, namespace=self._namespace)
        
    def set_links(self, links):
        self.links = links

    def get_namespace(self):
        return self._namespace

    def attach_stream_input_port(self, channel, callback, msg_type=None, from_fusion=False):
        for link in self.links:
            if link.channel == channel:
                port = link.dst
                if from_fusion:
                    port.set_msg_type(String)
                else:
                    port.set_msg_type(msg_type)
                port.from_fusion = from_fusion
                port.set_channel(channel)
                port.set_callback(callback)
                port.set_namespace(link.src.parent.get_namespace())
                port.attach()
                if not channel in self._stream_input_ports.keys():
                    self._stream_input_ports[channel] = []
                self._stream_input_ports[channel].append(port)

    def attach_stream_output_port(self, msg_type, channel):
        for link in self.links:
            if link.channel == channel:
                port = link.src
                port.set_msg_type(msg_type)
                port.set_channel(channel)
                port.set_namespace(link.dst.parent.get_namespace())
                port.attach()
                if not channel in self._stream_output_ports.keys():
                    self._stream_output_ports[channel] = []
                self._stream_output_ports[channel].append(port)
            
    def attach_modechange_input_port(self):
        mode_info = next((item for item in self.factory.mode_configuration["mode_list"] if item["name"] == self.mode), None)
        if mode_info:
            self._mode_input_port = ModeChangeInputPort(self)
        
    def attach_modechange_output_port(self, factory):
        self._mode_output_ports[factory] = ModeChangeOutputPort(self, factory)

    def attach_event_input_port(self, event, callback):
        self._event_input_ports[event] = EventInputPort(self, event, callback)

    def attach_event_output_port(self, event):
        self._event_output_ports[event]= EventOutputPort(self, event)

    def write(self, channel, msg):
        curframe = inspect.currentframe()
        calframe = inspect.getouterframes(curframe, 2)
        caller_name = calframe[1][3]
        source_msg = None
        for c, port_list in self._stream_input_ports.items():
            for port in port_list:
                if caller_name == port.get_callback().__name__:
                    source_msg = port.msg_list[0]
                    break
            if source_msg:
                break
        for port in self._stream_output_ports[channel]:
            port.write(msg, source_msg)

    def trigger_event(self, event):
        self._event_output_ports[event].trigger()

    def trigger_modechange(self, factory, event):
        print(f"trigger_modechange({factory}, {event})")
        self._mode_output_ports[factory].trigger(event)

    def setup(self):
        pass

    def run(self):
        pass

class FusionOperator(Component):
    class FusionRule():
        def __init__(self, m_ports, o_ports, o_ports_threshhold, correlation):
            self.mandatory_ports = m_ports
            self.optional_ports = o_ports
            self.optional_ports_threshold = o_ports_threshhold
            self.correlation = correlation

    def __init__(self, name, factory, mode):
        super().__init__(name, factory, mode)
        self._fusion_rule = None
        self._queues_for_each_input_port = {}
        self._stream_output_ports = []

    def attach_stream_input_port(self, channel, msg_type=None, from_fusion=False):
        for link in self.links:
            if link.channel == channel:
                port = link.dst
                if from_fusion:
                    port.set_msg_type(String)
                else:
                    port.set_msg_type(msg_type)
                port.set_channel(channel)
                port.set_callback(self._check_and_fusion, (channel,))
                port.set_namespace(link.src.parent.get_namespace())
                port.attach()
                if not channel in self._stream_input_ports.keys():
                    self._stream_input_ports[channel] = []
                self._stream_input_ports[channel].append(port)
                
                self._queues_for_each_input_port[channel] = []

    def attach_stream_output_port(self, channel):
        for link in self.links:
            if link.channel == channel:
                port = link.src
                port.set_msg_type(String)
                port.set_channel(channel)
                port.set_namespace(link.dst.parent.get_namespace())
                port.attach()
                self._stream_output_ports.append(port)

    def set_fusion_rule(self, fusion_rule):
        print('set fusion rule')
        print(fusion_rule)
        m_ports = self._get_ports_from_key(fusion_rule["mandatory_ports"])
        o_ports = self._get_ports_from_key(fusion_rule["optional_ports"])
        o_ports_threshhold = int(fusion_rule["optional_ports_threshold"])
        correlation = int(fusion_rule["correlation"])
        self._set_fusion_rule(m_ports, o_ports,
                              o_ports_threshhold, correlation)
    def _get_ports_from_key(self, ports):
        new_ports = []
        for port in ports:
            port = port.lower().replace(" ", "_")
            for link in self.links:
                if link.dst.name == port:
                    new_ports.append(link.dst)
        return new_ports
    def _set_fusion_rule(self, m_ports, o_ports, o_ports_threshhold, correlation):
        self._fusion_rule = self.FusionRule(
            m_ports, o_ports, o_ports_threshhold, correlation)
    
    def _check_and_fusion(self, msg, channel):
        # print("=====================================")
        # print("check_and_fusion: ", channel)
        msg_decoded = json.loads(msg.body)
        msg_converted = convert_dictionary_to_ros_message(self._stream_input_ports[channel][0].get_msg_type(), msg_decoded)
        for c, queue in self._queues_for_each_input_port.items():
            index = 0
            new_queue = []
            for item in queue:
                time_exec_ms = (self.get_clock().now().nanoseconds - item["time"]) / 1000000
                if item["freshness"] == None or item["freshness"] == 0 or item["freshness"] > time_exec_ms:
                    new_queue.append(item)
                else:
                    self.get_logger.warn('{}ms exceeded(constraint: {}ms, cur: {}ms'.format(time_exec_ms - item["freshness"], item["freshness"], time_exec_ms))
                index = index + 1
            self._queues_for_each_input_port[c] = new_queue
        self._queues_for_each_input_port[channel].append({"message": msg_converted, "time": Time.from_msg(msg.header.stamp).nanoseconds, "freshness": msg.freshness})
        valid_input_data = self._find_valid_input_data(self._fusion_rule, self._queues_for_each_input_port)
        if valid_input_data:
            # print(valid_input_data)
            data_encoded = json.dumps(valid_input_data)
            
        else:
            # print("==============empty data===============")
            return
            # empty_input_data = {}
            # for c in self._queues_for_each_input_port.keys():
            #     empty_input_data[c] = None
            # data_encoded = json.dumps(empty_input_data)

        new_msg = String()
        new_msg.data = data_encoded
        for port in self._stream_output_ports:
            port.write(new_msg)
        # print("================{}================".format(valid_input_data))
    def _find_valid_input_data(self, fusion_rule, queues_for_each_input_port):
        # print("FIND VALID INPUT DATA")
        index_list = [None] * len(queues_for_each_input_port.keys())
        i = 0
        optional_ports_count = 0
        # mandatory ports check
        # print("MANDATORY PORTS CHECK", end=": ")

        for mandatory_port in fusion_rule.mandatory_ports:
            queue = next((item for key, item in queues_for_each_input_port.items() if mandatory_port.get_channel() == key), False)
            if not queue or len(queue) == 0:
                # print("False")
                return None

        # print("True")
        for channel, queue in queues_for_each_input_port.items():
            if len(queue) > 0:
                index_list[i] = 0
                # count optional ports
                if next((item for item in fusion_rule.optional_ports if item.get_channel() == channel), False):
                    optional_ports_count = optional_ports_count + 1
            i = i + 1
        # num of optional ports should greater than threshold
        # print("NUM OF OPTIONAL PORTS", end=" ")
        if optional_ports_count < fusion_rule.optional_ports_threshold:
            # print("LESS THAN THRESHOLD")
            return None
        # print("GREATER THAN TRESHOLD")
        flag = True
        while flag:
            if self._is_valid_data(fusion_rule, queues_for_each_input_port, index_list):
                return self._build_data(queues_for_each_input_port, index_list)
            k, is_last = self._get_earlist_index(queues_for_each_input_port, index_list)

            if is_last:
                index_list[k] = None
            else:   
                index_list[k] = index_list[k] + 1
            flag = False
            for index in index_list:
                if index is not None:
                    flag = True
                    break
        return None

    def _is_valid_data(self, fusion_rule, queues_for_each_input_port, index_list):
        # print("CHECK IF IS VALID INPUT DATA")
        i = 0
        cur_data_list = []
        for key, queue in queues_for_each_input_port.items():
            if index_list[i] is not None:
                cur_data_list.append(queue[index_list[i]])
            i = i + 1
        if len(cur_data_list) < 2:
            return False
        for data in cur_data_list:
            for data2 in cur_data_list:
                if data == data2: continue
                time_diff_ms = abs(data["time"] - data2["time"]) / 1000000
                if  time_diff_ms > fusion_rule.correlation:
                    # print("False")s
                    return False
        # print("True")
        return True
    
    def _build_data(self, queues_for_each_input_port, index_list):
        # print("BUILD DATA", end=": ")

        i = 0
        data = {}
        for key, queue in queues_for_each_input_port.items():
            data[key] = None
            if index_list[i] is not None:
                data[key] = convert_ros_message_to_dictionary(queue[index_list[i]]["message"])
                if len(queue) == index_list[i] + 1:
                    queue = []
                else:
                    queue = queue[index_list[i]+1:]
                queues_for_each_input_port[key] = queue
            i = i + 1
        
        # print(data)
        return data
    
    def _get_earlist_index(self, queues_for_each_input_port, index_list):
        # print("GET EARLIST INDEX... ", end="")
        is_last = False
        earlist_index = -1
        index = 0
        queue_list = []
        for channel, queue in queues_for_each_input_port.items():
            queue_list.append(queue)
            index = index + 1
        index = 0
        for queue in queue_list:
            if index_list[index] is not None:
                if earlist_index < 0 or (len(queue) > 0 and queue[index_list[index]]["time"] < queue_list[earlist_index][index_list[earlist_index]]["time"]):
                    earlist_index = index
            index = index + 1
        if len(queue_list[earlist_index]) == index_list[earlist_index] + 1:
            is_last = True
        # print(earlist_index, is_last)
        return earlist_index, is_last