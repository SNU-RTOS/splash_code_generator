from .__util import *
import copy


class BuildUnitGenerator:
    def __init__(self, pkg_name, node_data, link_data):
        node_data_cp = copy.deepcopy(node_data)
        self.__pkg_name = pkg_name
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
        self.__links = link_data
        self.__links_transparent = self.__make_links_transparent(link_data)

    def generate(self):
        build_units = []
        not_assigned_stream_ports = self.__stream_ports
        not_assigned_stream_ports = self.__stream_ports
        not_assigned_event_input_ports = self.__event_input_ports
        not_assigned_event_output_ports = self.__event_output_ports
        not_assigned_modechange_output_ports = self.__modechange_output_ports

        for component in self.__processing_components:
            not_assigned_stream_ports = relate_stream_ports(
                component, not_assigned_stream_ports)
            not_assigned_event_input_ports = relate_event_input_ports(
                component, not_assigned_event_input_ports)
            not_assigned_event_output_ports = relate_event_output_ports(
                component, not_assigned_event_output_ports)
            not_assigned_modechange_output_ports = relate_modechange_output_ports(
                component, not_assigned_modechange_output_ports)
        for component in self.__source_components:
            not_assigned_stream_ports = relate_stream_ports(
                component, not_assigned_stream_ports)
            not_assigned_event_input_ports = relate_event_input_ports(
                component, not_assigned_event_input_ports)
            not_assigned_event_output_ports = relate_event_output_ports(
                component, not_assigned_event_output_ports)
            not_assigned_modechange_output_ports = relate_modechange_output_ports(
                component, not_assigned_modechange_output_ports)
        for component in self.__sink_components:
            not_assigned_stream_ports = relate_stream_ports(
                component, not_assigned_stream_ports)
            not_assigned_event_input_ports = relate_event_input_ports(
                component, not_assigned_event_input_ports)
            not_assigned_event_output_ports = relate_event_output_ports(
                component, not_assigned_event_output_ports)
            not_assigned_modechange_output_ports = relate_modechange_output_ports(
                component, not_assigned_modechange_output_ports)
        for component in self.__fusion_operators:
            not_assigned_stream_ports = relate_stream_ports(
                component, not_assigned_stream_ports)
            not_assigned_event_input_ports = relate_event_input_ports(
                component, not_assigned_event_input_ports)
            not_assigned_event_output_ports = relate_event_output_ports(
                component, not_assigned_event_output_ports)
            not_assigned_modechange_output_ports = relate_modechange_output_ports(
                component, not_assigned_modechange_output_ports)
        # generate a default build unit
        default_build_unit = self.__generate_build_unit("")
        if(default_build_unit):
            build_units.append(default_build_unit)
        # generate build units user made
        for _build_unit in self.__build_units:
            build_units.append(self.__generate_build_unit(
                _build_unit["key"], _build_unit["name"], _build_unit["class_name"]))

        return build_units

    def __make_links_transparent(self, link_data):
        new_links = []
        for link in link_data:
            new_link = {"from": None, "to": None}
            for port in self.__stream_ports:
                key = ""
                factory = ""
                mode = ""
                if port["PORT_TYPE"] == "STREAM_OUTPUT_PORT" and port["key"] == link["from"]:
                    parent = next(
                        (component for component in self.__processing_components if component["key"] == port["group"]), False)
                    if not parent:
                        parent = next(
                            (component for component in self.__source_components if component["key"] == port["group"]), False)
                    if not parent:
                        parent = next(
                            (component for component in self.__fusion_operators if component["key"] == port["group"]), False)
                    try:
                        key = parent["key"]
                        factory = parent["group"]
                        mode = parent["mode"]
                    except KeyError:
                        pass
                    new_link["from"] = {
                        "key": port["key"], "parent": {"key": key, "factory": factory, "mode": mode}, "channel": port["Channel"]}

                elif port["PORT_TYPE"] == "STREAM_INPUT_PORT" and port["key"] == link["to"]:
                    parent = next(
                        (component for component in self.__processing_components if component["key"] == port["group"]), False)
                    if not parent:
                        parent = next(
                            (component for component in self.__sink_components if component["key"] == port["group"]), False)
                    if not parent:
                        parent = next(
                            (component for component in self.__fusion_operators if component["key"] == port["group"]), False)
                    try:
                        key = parent["key"]
                        factory = parent["group"]
                        mode = parent["mode"]
                    except KeyError:
                        pass
                    new_link["to"] = {
                        "key": port["key"], "parent": {"key": key, "factory": factory, "mode": mode}, "channel": self.__find_channel_name_for_input_port(port)}

                if(new_link["from"] and new_link["to"]):
                    new_links.append(new_link)
                    break

        return new_links

    def __find_channel_name_for_input_port(self, input_port):
        for link in self.__links:
            if(link["to"] == input_port["key"]):
                for stream_port in self.__stream_ports:
                    if(stream_port["PORT_TYPE"] == "STREAM_OUTPUT_PORT" and stream_port["key"] == link["from"]):
                        return stream_port["Channel"]
        return None

    def __find_event_name_for_input_port(self, input_port):
        for link in self.__links:
            if(link["to"] == input_port["key"]):
                for output_port in self.__event_output_ports:
                    return output_port["Event"]
        return None

    def __generate_build_unit(self, key, name="", class_name=""):
        build_unit = {}
        build_unit["name"] = name if name else "default_build_unit"
        build_unit["class_name"] = class_name if class_name else "DefaultBuildUnit"
        build_unit["processing_components"] = self.__parse_component_info(
            "processingComponent", key)
        build_unit["source_components"] = self.__parse_component_info(
            "sourceComponent", key)
        build_unit["sink_components"] = self.__parse_component_info(
            "sinkComponent", key)
        build_unit["fusion_operators"] = self.__parse_component_info(
            "fusionOperator", key)
        build_unit["source_code"] = self.__generate_source_code(build_unit)
        build_unit["exec_code"] = self.__generate_exec_code(build_unit)

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
            result.append(_component)
        return result

    def __generate_source_code(self, build_unit):
        _str = ""
        _str = append_lines(_str, "'''", 0)
        _str = append_lines(
            _str, "Generated automatically by Splash Code Generator for {}".format(build_unit["name"]), 1)
        _str = append_lines(_str, "'''", 0)
        _str = append_lines(_str, self.__import_rcl(), 0)
        _str = append_lines(_str, self.__import_srvs(), 0)
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
        _str = append_lines(
            _str, self.__generate_build_unit_class(build_unit), 0)
        return _str

    def __generate_exec_code(self, build_unit):
        _str = ""
        _str = append_lines(_str, "'''", 0)
        _str = append_lines(
            _str, "Generated automatically by Splash Code Generator for {}".format(build_unit["name"]), 1)
        _str = append_lines(_str, "'''", 0)
        # # temp

        # _str = append_lines(_str, "import sys", 0)
        # _str = append_lines(
        #     _str, "sys.path.append(\"C:/Workspace/rtos/Splash/ClientLibraries\")", 0)
        # # temp end
        _str = append_lines(_str, self.__import_rcl(), 0)
        _str = append_lines(
            _str, self.__import_build_unit(build_unit["name"], build_unit["class_name"]), 0)
        _str = append_lines(_str, self.__generate_main(build_unit), 0)
        return _str

    def __import_rcl(self):
        _str = ""

        _str = append_lines(_str, "import rclpy", 0)
        _str = append_lines(
            _str, "from rclpy.executors import MultiThreadedExecutor", 0)
        return _str

    def __import_srvs(self):
        _str = ""
        for event_port in self.__event_input_ports:

            event_name = self.__find_event_name_for_input_port(event_port)
            _str = _str + "from {}_interfaces.srv import {}\n".format(
                self.__pkg_name, CamelCaseConverter(event_name).__str__())
        return _str

    def __import_build_unit(self, name, class_name):
        _str = ""

        _str = append_lines(
            _str, "from .splash.build_unit.{} import {}".format(name, class_name), 0)
        return _str

    def __import_components(self, components):
        _str = ""
        for component in components:
            _str = append_lines(
                _str, "from ..component.{} import {}".format(component["name"], component["class_name"]), 0)

        return _str

    def __generate_main(self, build_unit):
        _str = ""
        _str = append_lines(_str, "def main(args=None):", 0)
        _str = append_lines(_str, "rclpy.init(args=args)", 1)
        _str = append_lines(_str, "executor = MultiThreadedExecutor()\n", 1)
        _str = append_lines(_str, "build_unit = {}()".format(
            build_unit["class_name"]), 1)
        _str = append_lines(_str, "for component in build_unit.components:", 1)
        _str = append_lines(_str, "executor.add_node(component)", 2)
        _str = append_lines(_str, "executor.spin()", 1)
        _str = append_lines(_str, "rclpy.shutdown()", 1)
        return _str

    def __generate_build_unit_class(self, build_unit):
        _str = ""
        _str = append_lines(_str, "class {}():".format(
            build_unit["class_name"]), 0)
        _str = append_lines(_str, "def __init__(self):", 1)
        _str = append_lines(_str, "self.components = []", 2)
        for component in build_unit["processing_components"]:
            _str = append_lines(
                _str, self.__generate_lines_for_component(component), 2)
        for component in build_unit["source_components"]:
            _str = append_lines(
                _str, self.__generate_lines_for_component(component), 2)
        for component in build_unit["sink_components"]:
            _str = append_lines(
                _str, self.__generate_lines_for_component(component), 2)
        for component in build_unit["fusion_operators"]:
            _str = append_lines(
                _str, self.__generate_lines_for_component(component), 2)
        return _str

    def __generate_lines_for_component(self, component):
        _str = ""
        name = ""
        factory = ""
        mode = ""
        stream_input_ports = []
        stream_output_ports = []
        event_input_ports = []
        event_output_ports = []
        modechange_output_ports = []
        name = component["name"]
        try:
            factory = component["group"]
            mode = component["mode"]
        except KeyError:
            pass
        input_ports = component["stream_input_ports"]
        output_ports = component["stream_output_ports"]
        event_input_ports = component["event_input_ports"]
        event_output_ports = component["event_output_ports"]
        modechange_output_ports = component["modechange_output_ports"]
        links = []
        for link in self.__links_transparent:
            for input_port in input_ports:
                if input_port["key"] == link["to"]["key"]:
                    links.append(link)
            for output_port in output_ports:
                if output_port["key"] == link["from"]["key"]:
                    links.append(link)
        _str = append_lines(_str, "{} = {}()".format(
            component["name"], component["class_name"]), 0)
        _str = append_lines(
            _str, "{}.set_info(factory=\"{}\",mode=\"{}\", stream_input_ports={}, stream_output_ports={}, event_input_ports={}, event_output_ports={}, modechange_output_ports={}, links={})".format(name, factory, mode, stream_input_ports, stream_output_ports, event_input_ports, event_output_ports, modechange_output_ports, links), 0)
        _str = append_lines(_str, self.__register_event_callback(
            component["name"], event_input_ports), 0)
        _str = append_lines(_str, "{}.setup()".format(component["name"]), 0)
        _str = append_lines(
            _str, "self.components.append({})".format(component["name"]), 0)
        _str = append_lines(_str, "{}.run()".format(component["name"]), 0)
        return _str

    def __register_event_callback(self, component_name, event_input_ports):
        _str = ""
        for event_port in event_input_ports:
            event_name = self.__find_event_name_for_input_port(event_port)
            _str = _str + "{}.attach_event_input_port({}, \"{}\", {}.{}_callback)".format(
                component_name, CamelCaseConverter(event_name).__str__(), event_name, component_name, event_name.lower().replace(" ", "_"))
        return _str
