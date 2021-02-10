class Observer():
    def __init__(self):
        
        self._msg_port = None
        self._msg_path = None

    def port_to_path(self, port, path):
        self._msg_port = port.send_message()
        path.receive_message(self._msg_port)

    def path_to_port(self, path, port):
        self._msg_path = path.send_message()
        port.receive_message(self._msg_path)

