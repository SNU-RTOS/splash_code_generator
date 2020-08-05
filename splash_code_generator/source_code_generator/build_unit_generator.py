
def append_lines(string, lines, indent):
    line_list = lines.split('\n')
    for line in line_list:
        string = string + (" " * indent * 4) + line + "\n"
    return string


class BuildUnitGenerator:
    def __init__(self, node_data):
        self.__processing_components = node_data["processing_components"]
        self.__source_components = node_data["source_components"]
        self.__sink_components = node_data["sink_components"]
        self.__fusion_operators = node_data["fusion_operators"]
        self.__factories = node_data["factories"]
        self.__build_units = node_data["build_units"]
        self.__stream_ports = node_data["stream_ports"]
        self.__event_input_ports = node_data["event_input_ports"]
        self.__event_output_ports = node_data["event_output_ports"]
        self.__modechange_input_ports = node_data["modechange_input_ports"]
        self.__modechange_output_ports = node_data["modechange_output_ports"]
        # self.__links = link_data_list

    def generate(self):
        build_units = []
        # generate a default build unit
        build_units.append(self.__generate_build_unit())
        # generate build units user made
        for _build_unit in self.__build_units:
            build_units.append(self.__generate_build_unit(_build_unit["key"]))

        return build_units

    def __generate_build_unit(self, name=""):
        build_unit = {}
        build_unit["name"] = name
        build_unit["file_name"] = name if name else "default_build_unit"
        build_unit["processing_components"] = self.__parse_component_info(
            "processingComponent", name)
        build_unit["source_components"] = self.__parse_component_info(
            "sourceComponent", name)
        build_unit["sink_components"] = self.__parse_component_info(
            "sinkComponent", name)
        build_unit["fusion_operators"] = self.__parse_component_info(
            "fusionOperator", name)
        build_unit["source_code"] = self.__generate_source_code(build_unit)

        return build_unit

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
        _str = append_lines(_str, "'''", 0)
        _str = append_lines(
            _str, "Generated automatically by Splash Code Generator for {}".format(build_unit["file_name"]), 1)
        _str = append_lines(_str, "'''", 0)
        _str = append_lines(_str, self.__import_rcl(), 0)
        _str = append_lines(_str, self.__import_scl_component_node(), 0)
        if(len(build_unit["processing_components"]) > 0):
            _str = append_lines(_str, self.__import_components(
                build_unit["processing_components"]), 0)
        if(len(build_unit["source_components"]) > 0):
            _str = append_lines(_str, self.__import_components(
                build_unit["source_components"]), 0)
        if(len(build_unit["sink_components"]) > 0):
            _str = append_lines(_str, self.__import_components(
                build_unit["sink_components"]), 0)
        if(len(build_unit["fusion_operators"]) > 0):
            _str = append_lines(_str, self.__import_components(
                build_unit["fusion_operators"]), 0)
        _str = append_lines(_str, self.__generate_main(build_unit), 0)
        print(_str)
        return _str

    def __import_rcl(self):
        _str = ""

        _str = append_lines(_str, "import rclpy", 0)
        _str = append_lines(_str, "from rclpy.node import Node", 0)
        _str = append_lines(
            _str, "from rclpy.executor import MultiThreadedExecutor", 0)
        return _str

    def __import_scl_component_node(self):
        _str = ""

        _str = append_lines(_str, "from scl import ComponentNode", 0)

        return _str

    def __import_components(self, components):
        _str = ""
        for component in components:
            _str = append_lines(
                _str, "from .splash.{} import {}".format(component["name"], component["class_name"]), 0)

        return _str

    def __generate_main(self, build_unit):
        _str = ""
        _str = append_lines(_str, "def main(args=None):", 0)
        _str = append_lines(_str, "rclpy.init(args=args)", 1)
        _str = append_lines(_str, "executor = MultiThreadedExecutor()\n", 1)
        for component in build_unit["processing_components"]:
            _str = append_lines(
                _str, self.__generate_lines_for_component(component), 1)
        for component in build_unit["source_components"]:
            _str = append_lines(
                _str, self.__generate_lines_for_component(component), 1)
        for component in build_unit["sink_components"]:
            _str = append_lines(
                _str, self.__generate_lines_for_component(component), 1)
        for component in build_unit["fusion_operators"]:
            _str = append_lines(
                _str, self.__generate_lines_for_component(component), 1)
        _str = append_lines(_str, "executor.spin()", 1)
        _str = append_lines(_str, "rclpy.shutdown()", 1)
        return _str

    def __generate_lines_for_component(self, component):
        _str = ""
        _str = append_lines(_str, "{} = {}()".format(
            component["name"], component["class_name"]), 0)
        _str = append_lines(_str, "{}.setup()".format(component["name"]), 0)
        _str = append_lines(
            _str, "{0}_node = ComponentNode({0})".format(component["name"]), 0)
        _str = append_lines(
            _str, "executor.add_node({}_node)".format(component["name"]), 0)
        _str = append_lines(_str, "{}.run()".format(component["name"]), 0)
        return _str

    def __generate_subscription(self, stream_port):
        _str = ""
        return _str

    def __generate_publisher(self, stream_port):
        _str = ""
        return _str

    def __generate_service(self, event):
        _str = ""
        return _str
