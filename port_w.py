class Port:
    def __init__(self, name):
        self._name = name
        self._path = None
        self._inout = None
        self._port_type = None
        self._observer = None
        self._component = None

    def attach_component(self, component):
        self._component = component

    def get_attached_component(self):
        return self._component    
    
    def attach_observer(self, observer):
        self._observer = observer

    def get_inout(self):
        return self._inout

    def get_port_type(self):
        return self._port_type
    
    def get_channel(self):
        return self._path.get_channel()

    def get_clink(self):
        return self._path.get_channel()

    def write(self):
        raise NotImplementedError()

    def send_message(self):
        raise NotImplementedError()

    def receive_message(self, msg):
        raise NotImplementedError()


class StreamInputPort(Port):
    def __init__(self, name, path):
        super.__init__(name)
        self._path = path
        self._inout = 'in'
        self._port_type = 'stream'
        self._msg_queue = []

    def receive_message(self, msg):
        self._msg_queue.append(msg)



class StreamOutputPort(Port):
    def __init__(self, name, path):
        super.__init__(name)
        self._path = path
        self._inout = 'out'
        self._port_type = 'stream'
        self._msg_queue = []

    def write(self, msg):
        self._msg_queue.append(msg)
        self._observer.port_to_path(self, self._path)


    def send_message(self):
        msg = self._msg_queue.pop(0)
        return msg    


class EventInputPort(Port):
    def __init__(self, name, path):
        super.__init__(name)
        self._path = path
        self._inout = 'in'
        self._port_type = 'event'
        self._msg_queue = []

    def receive_message(self, msg):
        self._msg_queue.append(msg)   


class EventOutputPort(Port):
    def __init__(self, name, path):
        super.__init__(name)
        self._path = path
        self._inout = 'out'
        self._port_type = 'event'
        self._msg_queue = []

    def write(self, msg):
        self._msg_queue.append(msg)
        self._observer.port_to_path()

    def send_message(self):
        msg = self._msg_queue.pop(0)
        return msg    


class ModeInputPort(Port):
    def __init__(self, name, path):
        super.__init__(name)
        self._path = path
        self._inout = 'in'
        self._port_type = 'mode'
        self._msg_queue = []

    def receive_message(self, msg):
        self._msg_queue.append(msg)


class ModeOutputPort(Port):
    def __init__(self, name, path):
        super.__init__(name)
        self._path = path
        self._inout = 'out'
        self._port_type = 'mode'
        self._msg_queue = []

    def write(self, msg):
        self._msg_queue.append(msg)
        self._observer.port_to_path()

    def send_message(self):
        msg = self._msg_queue.pop(0)
        return msg    
