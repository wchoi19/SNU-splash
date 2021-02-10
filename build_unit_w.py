class BuildUnit:
    def __init__(self):

        self._components_list = []
        self._executor = SplashExecutor()

    def add_components(self, component):
        self._components_list.append(component) 

    def run(self):        
        for component in self._components_list:
            self._executor.add_node(component)
        
        try:
            while rclpy.ok():
                self._executor.spin()
        except KeyboardInterrupt:
            print("KeyboardInterrupt")
            for component in self._components_list:
                component.destroy_node()
            self._executor.shutdown()    
