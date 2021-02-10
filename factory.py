from .impl.singleton import Singleton

class Factory(metaclass=Singleton):
    def __init__(self, name, parent, mode):
        self.name = name
        self.parent = parent
        self.mode = mode
        self.mode_configuration = None

    def get_namespace(self):
        parent = self.parent
        namespace = self.mode + "/" + self.name if self.mode else self.name
        if(parent):
            namespace = parent.get_namespace() + "/" + namespace
        return namespace

    def set_mode_configuration(self, mode_configuration):
        self.mode_configuration = mode_configuration
        # # rclpy.init()
        # # rclpy.create_node(node_name="splash_mode_manager")
        # self.node = Node(name="splash_mode_manager")
