from splash_interfaces.msg import SplashMessage
//wooyoung
class Path:
    def __init__(self, name):
        self._name = name
        self._channel = None
        self._observer = None
        self._port = None

    def set_port(self, port):
        self._port = port    

    def get_channel(self):
        return self._channel

    def attach_observer(self, observer):
        self._observer = observer

    def _create_publisher(self, channel, callback):
        raise NotImplementedError()

    def _create_subscriber(self, channel):
        raise NotImplementedError()
   
    def _create_client(self, event):
        raise NotImplementedError()

    def _create_serivce(self, event, callback):
        raise NotImplementedError()

class Channel(Path):
    def __init__(self, name, channel, callback):
        super.__init__(name)
        self._channel = channel
        self._callback = callback
        self._msg_queue= []
        self._publisher = None
        self._create_publisher(channel)
        self._subscriber = None
        self._create_subscriber(channel, callback)

    def send_msg(self):
        msg = self._msg_queue.pop(0)
        return msg

    def receive_msg(self, msg):
        self._msg_queue.append(msg)                

    def _create_publisher(self, channel):
        self._topic = channel
        component = self._port.get_attached_component()
        self._publisher = component.create_publisher(SplashMessage, self._topic, 1)

        # put rate constraint here ?? not sure yet

    def _create_subscriber(self, channel, callback):
        self._topic = channel
        component = self._port.get_attached_component()

        self._subscriber = component.create_subscription(SplashMessage, self._topic, self._execute_callback, 1) 

    def _execute_callback(self, msg):
        self._msg_queue.append(msg)
        self._observer.path_to_port(self, self._port)


class Clink(Path):
    def __init__(self, name, event, callback):
        super.__init__(name)
        self._callback = callback
        self._channel = event
        self._msg_queue = []
   
        self._client = None
        self._create_client(event)
        self._service = None
        self._create_serivce(event, callback)

    def send_msg(self):
        msg = self._msg_queue.pop(0)
        return msg
    def receive_msg(self, msg):
        self._msg_queue.append(msg)        

    def _create_service(self, event, callback):
        self._topic = event
        component = self._port.get_attached_component()

        self._publisher = component.create_publisher(SplashMessage, self._topic, 1)      

    def _create_client(self, event):
        self._topic = event
        component = self._port.get_attached_component()

        self._subscriber = component.create_subscription(SplashMessage, self._topic, self._execute_callback, 1) 

    def _execute_callback(self, msg):
        self._msg_queue.append(msg)
        self._observer.path_to_port(self, self._port)



