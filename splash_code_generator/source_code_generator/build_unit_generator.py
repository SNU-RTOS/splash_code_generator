class BuildUnitGenerator:
    def __init__(self, node_data_list, link_data_list):
        self.build_units = []
        self.__processing_components = []
        self.__source_components = []
        self.__sink_components = []
        self.__fusion_operators = []
        self.__factories = []
        self.__build_units = []
        self.__stream_ports = []
        self.__event_input_ports = []
        self.__event_output_ports = []
        self.__modechange_input_ports = []
        self.__modechange_output_ports = []
        self.__links = link_data_list
        for node in node_data_list:
            category = node["category"]
            if(category == "processingComponent"):
                self.__processing_components.append(node)
            elif(category == "sourceComponent"):
                self.__source_components.append(node)
            elif(category == "sinkComponent"):
                self.__sink_components.append(node)
            elif(category == "fusionOperator"):
                self.__fusion_operators.append(node)
            elif(category == "factory"):
                self.__factories.append(node)
            elif(category == "buildUnit"):
                self.__build_units.append(node)
            elif(category == "streamPort"):
                self.__stream_ports.append(node)
            elif(category == "streamOutputPort"):
                self.__stream_output_ports.append(node)
            elif(category == "eventInputPort"):
                self.__event_input_ports.append(node)
            elif(category == "eventOutputPort"):
                self.__event_output_ports.append(node)
            elif(category == "modeChangeInputPort"):
                self.__modechange_input_ports.append(node)
            elif(category == "modeChangeOutputPort"):
                self.__modechange_output_ports.append(ndoe)
            else:
                print("Error")

    def generate(self):
        # generate a default build unit
        self.__generate_build_unit()
        # generate build units user made
        for _build_unit in self.__build_units:
            self.__generate_build_unit(
                _build_unit["key"])

    def __generate_build_unit(self, name=""):
        build_unit = {}
        build_unit["name"] = name
        build_unit["file_name"] = name.lower().replace(
            " ", "_") if name else "default_build_unit"
        build_unit["processing_components"] = self.__parse_component_info(
            "processingComponent", name)
        build_unit["source_components"] = self.__parse_component_info(
            "sourceComponent", name)
        build_unit["sink_components"] = self.__parse_component_info(
            "sinkComponent", name)
        build_unit["fusion_operator"] = self.__parse_component_info(
            "fusionOperator", name)
        build_unit["source_code"] = self.__generate_source_code(build_unit)

    def __parse_component_info(self, category, build_unit_name):
        components = []
        name = ""
        if(category == "processingComponent"):
            components = self.__processing_components
            name = "processing_components"
        elif(category == "sourceComponent"):
            components = self.__source_components
            name = "source_components"
        elif(category == "sinkComponent"):
            components = self.__sink_components
            name = "sink_components"
        elif(category == "fusionOperator"):
            components = self.__fusion_operators
            name = "fusion_operators"
        result = []
        for _component in components:
            if(_component["buildUnit"] != build_unit_name):
                continue
            component = {}
            component.update(_component)
            component["stream_input_ports"] = []
            component["stream_output_ports"] = []
            component["event_input_ports"] = []
            component["event_output_ports"] = []
            component["modechange_input_ports"] = []
            component["modechange_output_ports"] = []
            for _stream_port in self.__stream_ports:
                if(_stream_port["group"] == _component["key"]):
                    if(_stream_port["PORT_TYPE"] == "STREAM_INPUT_PORT"):
                        component["stream_input_ports"].append(_stream_port)
                    elif(_stream_port["PORT_TYPE"] == "STREAM_OUTPUT_PORT"):
                        component["stream_output_ports"].append(_stream_port)

            for _event_port in self.__event_input_ports:
                component["event_input_ports"].append(_event_port)
            for _event_port in self.__event_output_ports:
                component["event_output_ports"].append(_event_port)

            for _modechange_port in self.__modechange_input_ports:
                component["modechange_input_ports"].append(_modechange_port)
            for _modechange_port in self.__modechange_output_ports:
                component["modechange_output_ports"].append(_modechange_port)

            result.append(component)
        return result

    def __generate_source_code(self, build_unit):
        _str = ""
        _str = self.__append_line(_str, "'''", 0)
        _str = self.__append_line(
            _str, "Generated automatically by Splash Code Generator", 1)
        _str = self.__append_line(_str, "'''", 0)
        _str = self.__append_line(_str, self.__import_rcl(), 0)
        print(_str)

    def __import_rcl(self):
        _str = ""

        _str = self.__append_line(_str, "import rclpy", 0)
        _str = self.__append_line(_str, "from rclpy.node import Node", 0)
        _str = self.__append_line(
            _str, "from rclpy.executor import MultiThreadedExecutor", 0)
        _str = self.__append_line(_str, "from std_msgs.msg import String", 0)

        return _str

    def __append_line(self, string, new_string, indent):
        return string + (" " * indent * 4) + new_string + "\n"

    def __generate_node(self, component):
        pass

    def __generate_subscription(self, stream_port):
        pass

    def __generate_publisher(self, stream_port):
        pass

    def __generate_service(self, event):
        pass
