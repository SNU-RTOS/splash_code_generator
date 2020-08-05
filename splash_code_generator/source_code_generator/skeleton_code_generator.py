from .__util import append_lines
import copy


class SkeletonCodeGenerator:
    def __init__(self, node_data, link_data_list):
        self.skeletons = []
        node_data_cp = copy.deepcopy(node_data)
        self.__processing_components = node_data_cp["processing_components"]
        self.__source_components = node_data_cp["source_components"]
        self.__sink_components = node_data_cp["sink_components"]
        self.__fusion_operators = node_data_cp["fusion_operators"]
        self.__factories = node_data_cp["factories"]
        self.__build_units = node_data_cp["build_units"]
        self.__stream_ports = node_data_cp["stream_ports"]
        self.__event_input_ports = node_data_cp["event_input_ports"]
        self.__event_output_ports = node_data_cp["event_output_ports"]
        self.__modechange_input_ports = node_data_cp["modechange_input_ports"]
        self.__modechange_output_ports = node_data_cp["modechange_output_ports"]
        self.__links = link_data_list

    def generate(self):
        skeletons = []
        not_assigned_stream_ports = self.__stream_ports
        for component in self.__processing_components:
            component, not_assigned_stream_ports = self.__relate_stream_ports(
                component, not_assigned_stream_ports)
            skeletons.append(self.__generate_skeleton(component))
        for component in self.__source_components:
            component, not_assigned_stream_ports = self.__relate_stream_ports(
                component, not_assigned_stream_ports)
            skeletons.append(self.__generate_skeleton(component))
        for component in self.__sink_components:
            component, not_assigned_stream_ports = self.__relate_stream_ports(
                component, not_assigned_stream_ports)
            skeletons.append(self.__generate_skeleton(component))
        for component in self.__fusion_operators:
            component, not_assigned_stream_ports = self.__relate_stream_ports(
                component, not_assigned_stream_ports)
            skeletons.append(self.__generate_skeleton(component))

        return skeletons

    def __relate_stream_ports(self, component, stream_ports):
        component["stream_input_ports"] = []
        component["stream_output_ports"] = []
        new_stream_port = []
        for stream_port in stream_ports:
            if(stream_port["group"] == component["key"]):
                if(stream_port["PORT_TYPE"] == "STREAM_INPUT_PORT"):
                    stream_port = self.__simplify_port(stream_port)
                    component["stream_input_ports"].append(stream_port)
                elif(stream_port["PORT_TYPE"] == "STREAM_OUTPUT_PORT"):
                    stream_port = self.__simplify_port(stream_port)
                    component["stream_output_ports"].append(stream_port)
            else:
                new_stream_port.append(stream_port)
        return component, new_stream_port

    def __simplify_port(self, port):
        try:
            del port['category']
            del port['loc']
            del port['group']
            del port['buildUnit']
        except KeyError:
            pass

        return port

    def __generate_skeleton(self, component):
        skeleton = {}
        skeleton["name"] = component["name"]
        skeleton["source_code"] = self.__generate_source_code(component)
        return skeleton

    def __generate_source_code(self, component):
        _str = ""
        _str = append_lines(_str, "'''", 0)
        _str = append_lines(
            _str, "Generated automatically by Splash Code Generator for {}".format(component["name"]), 1)
        _str = append_lines(_str, "'''", 0)
        _str = append_lines(_str, self.__import_scl(), 0)
        _str = append_lines(_str, self.__generate_class(component), 0)

        print(_str)
        return _str

    def __import_scl(self):
        _str = ""
        _str = append_lines(_str, "from scl import Component", 0)
        return _str

    def __generate_class(self, component):
        _str = ""
        _str = append_lines(
            _str, "class {}(Component):".format(component["class_name"]), 0)
        _str = append_lines(_str, self.__generate_init(component), 1)
        _str = append_lines(_str, self.__generate_setup(component), 1)
        _str = append_lines(_str, self.__generate_run(), 1)
        _str = append_lines(_str, self.__generate_user_funcs(component), 1)
        return _str

    def __generate_init(self, component):
        _str = ""
        _str = append_lines(_str, "def __init__(self):", 0)
        _str = append_lines(
            _str, "super().__init__(\"{}\")".format(component["name"]), 1)
        return _str

    def __generate_setup(self, component):
        _str = ""
        _str = append_lines(_str, "def setup(self):", 0)
        count = 1
        for input_port in component["stream_input_ports"]:
            channel = self.__find_channel_name_for_input_port(input_port)
            _str = append_lines(_str, self.__append_input_port(
                channel, "user_func_{}".format(count)), 1)
            count = count + 1
        for output_port in component["stream_output_ports"]:
            channel = output_port["Channel"]
            _str = append_lines(_str, self.__append_output_port(channel), 1)
        return _str

    def __generate_user_funcs(self, component):
        _str = ""
        count = 1
        for input_port in component["stream_input_ports"]:
            _str = append_lines(
                _str, "def user_func_{}(self, msg):".format(count), 0)
            _str = append_lines(_str, "pass\n", 1)
            count = count + 1
        return _str

    def __generate_run(self):
        _str = ""
        _str = append_lines(_str, "def run(self):", 0)
        _str = append_lines(_str, "pass", 1)
        return _str

    def __append_input_port(self, channel, user_func_name):
        _str = ""
        _str = "self.attach_input_port(String, \"{}\", {})".format(
            channel, user_func_name)
        return _str

    def __find_channel_name_for_input_port(self, input_port):
        for link in self.__links:
            if(link["to"] == input_port["key"]):
                for stream_port in self.__stream_ports:
                    if(stream_port["PORT_TYPE"] == "STREAM_OUTPUT_PORT" and stream_port["key"] == link["from"]):
                        return stream_port["Channel"]
        return None

    def __append_output_port(self, channel):
        _str = ""
        _str = "self.attach_output_port(String, \"{}\")".format(channel)
        return _str
