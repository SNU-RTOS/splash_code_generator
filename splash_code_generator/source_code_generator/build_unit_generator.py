from ._util import *
import copy


class BuildUnitGenerator:
    def __init__(self, pkg_name, node_data, link_data):
        node_data_cp = copy.deepcopy(node_data)
        self._pkg_name = pkg_name
        self._processing_components = node_data_cp["processing_components"]
        self._source_components = node_data_cp["source_components"]
        self._sink_components = node_data_cp["sink_components"]
        self._fusion_operators = node_data_cp["fusion_operators"]
        self._factories = node_data_cp["factories"]
        self._build_units = node_data_cp["build_units"]
        self._stream_ports = node_data_cp["stream_ports"]
        self._event_input_ports = node_data_cp["event_input_ports"]
        self._event_output_ports = node_data_cp["event_output_ports"]
        self._modechange_input_ports = node_data_cp["modechange_input_ports"]
        self._modechange_output_ports = node_data_cp["modechange_output_ports"]
        self._links = link_data
        self._links_transparent = self._make_links_transparent(link_data)

    def generate(self):
        build_units = []
        not_assigned_stream_ports = self._stream_ports
        not_assigned_stream_ports = self._stream_ports
        not_assigned_event_input_ports = self._event_input_ports
        not_assigned_event_output_ports = self._event_output_ports
        not_assigned_modechange_output_ports = self._modechange_output_ports
        factory_set = set()
        for factory in self._factories:
            for component in self._processing_components:
                if "group" in component.keys() and factory["key"] == component["group"]:
                    factory_set.add(factory["key"])
            for component in self._source_components:
                if "group" in component.keys() and factory["key"] == component["group"]:
                    factory_set.add(factory["key"])
            for component in self._sink_components:
                if "group" in component.keys() and factory["key"] == component["group"]:
                    factory_set.add(factory["key"])
            for component in self._fusion_operators:
                if "group" in component.keys() and factory["key"] == component["group"]:
                    factory_set.add(factory["key"])
        self._factories = [
            factory for factory in self._factories if factory["key"] in factory_set]

        for component in self._processing_components:
            not_assigned_stream_ports = relate_stream_ports(
                component, not_assigned_stream_ports)
            not_assigned_event_input_ports = relate_event_input_ports(
                component, not_assigned_event_input_ports)
            not_assigned_event_output_ports = relate_event_output_ports(
                component, not_assigned_event_output_ports)
            not_assigned_modechange_output_ports = relate_modechange_output_ports(
                component, not_assigned_modechange_output_ports)
        for component in self._source_components:
            not_assigned_stream_ports = relate_stream_ports(
                component, not_assigned_stream_ports)
            not_assigned_event_input_ports = relate_event_input_ports(
                component, not_assigned_event_input_ports)
            not_assigned_event_output_ports = relate_event_output_ports(
                component, not_assigned_event_output_ports)
            not_assigned_modechange_output_ports = relate_modechange_output_ports(
                component, not_assigned_modechange_output_ports)
        for component in self._sink_components:
            not_assigned_stream_ports = relate_stream_ports(
                component, not_assigned_stream_ports)
            not_assigned_event_input_ports = relate_event_input_ports(
                component, not_assigned_event_input_ports)
            not_assigned_event_output_ports = relate_event_output_ports(
                component, not_assigned_event_output_ports)
            not_assigned_modechange_output_ports = relate_modechange_output_ports(
                component, not_assigned_modechange_output_ports)
        for component in self._fusion_operators:
            not_assigned_stream_ports = relate_stream_ports(
                component, not_assigned_stream_ports)
            not_assigned_event_input_ports = relate_event_input_ports(
                component, not_assigned_event_input_ports)
            not_assigned_event_output_ports = relate_event_output_ports(
                component, not_assigned_event_output_ports)
            not_assigned_modechange_output_ports = relate_modechange_output_ports(
                component, not_assigned_modechange_output_ports)
        # generate a default build unit
        default_build_unit = self._generate_build_unit("")
        if(default_build_unit):
            build_units.append(default_build_unit)
        # generate build units user made
        for _build_unit in self._build_units:
            build_units.append(self._generate_build_unit(
                _build_unit["key"], _build_unit["name"], _build_unit["class_name"]))

        return build_units

    def _make_links_transparent(self, link_data):
        new_links = []
        for link in link_data:
            new_link = {"from": None, "to": None}
            for port in self._stream_ports:
                key = ""
                factory = ""
                mode = ""
                if port["PORT_TYPE"] == "STREAM_OUTPUT_PORT" and port["key"] == link["from"]:
                    parent = next(
                        (component for component in self._processing_components if component["key"] == port["group"]), False)
                    if not parent:
                        parent = next(
                            (component for component in self._source_components if component["key"] == port["group"]), False)
                    if not parent:
                        parent = next(
                            (component for component in self._fusion_operators if component["key"] == port["group"]), False)
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
                        (component for component in self._processing_components if component["key"] == port["group"]), False)
                    if not parent:
                        parent = next(
                            (component for component in self._sink_components if component["key"] == port["group"]), False)
                    if not parent:
                        parent = next(
                            (component for component in self._fusion_operators if component["key"] == port["group"]), False)
                    try:
                        key = parent["key"]
                        factory = parent["group"]
                        mode = parent["mode"]
                    except KeyError:
                        pass
                    new_link["to"] = {
                        "key": port["key"], "parent": {"key": key, "factory": factory, "mode": mode}, "channel": self._find_channel_name_for_input_port(port)}

                if(new_link["from"] and new_link["to"]):
                    new_links.append(new_link)
                    break

        return new_links

    def _find_channel_name_for_input_port(self, input_port):
        for link in self._links:
            if(link["to"] == input_port["key"]):
                for stream_port in self._stream_ports:
                    if(stream_port["PORT_TYPE"] == "STREAM_OUTPUT_PORT" and stream_port["key"] == link["from"]):
                        return stream_port["Channel"]
        return None

    def _find_event_name_for_input_port(self, input_port):
        for link in self._links:
            if(link["to"] == input_port["key"]):
                for output_port in self._event_output_ports:
                    return output_port["Event"]
        return None

    def _generate_build_unit(self, key, name="", class_name=""):
        build_unit = {}
        build_unit["name"] = name if name else "default_build_unit"
        build_unit["class_name"] = class_name if class_name else "DefaultBuildUnit"
        build_unit["processing_components"] = self._parse_component_info(
            "processingComponent", key)
        build_unit["source_components"] = self._parse_component_info(
            "sourceComponent", key)
        build_unit["sink_components"] = self._parse_component_info(
            "sinkComponent", key)
        build_unit["fusion_operators"] = self._parse_component_info(
            "fusionOperator", key)
        build_unit["source_code"] = self._generate_source_code(build_unit)
        build_unit["exec_code"] = self._generate_exec_code(build_unit)

        return build_unit

    def _parse_component_info(self, category, build_unit_name):
        components = []
        name = ""
        if(category == "processingComponent"):
            components = self._processing_components
            name = "processing_components"
        elif(category == "sourceComponent"):
            components = self._source_components
            name = "source_components"
        elif(category == "sinkComponent"):
            components = self._sink_components
            name = "sink_components"
        elif(category == "fusionOperator"):
            components = self._fusion_operators
            name = "fusion_operators"
        result = []
        for _component in components:
            if(_component["buildUnit"] != build_unit_name):
                continue
            result.append(_component)
        return result

    def _generate_source_code(self, build_unit):
        _str = ""
        _str = append_lines(_str, "'''", 0)
        _str = append_lines(
            _str, "Generated automatically by Splash Code Generator for {}".format(build_unit["name"]), 1)
        _str = append_lines(_str, "'''", 0)
        _str = append_lines(_str, self._import_rcl(), 0)
        _str = append_lines(_str, self._import_scl(), 0)
        # _str = append_lines(_str, self._import_srvs(), 0)
        if(len(build_unit["processing_components"]) > 0):
            _str = append_lines(_str, self._import_components(
                build_unit["processing_components"]), 0)
        if(len(build_unit["source_components"]) > 0):
            _str = append_lines(_str, self._import_components(
                build_unit["source_components"]), 0)
        if(len(build_unit["sink_components"]) > 0):
            _str = append_lines(_str, self._import_components(
                build_unit["sink_components"]), 0)
        if(len(build_unit["fusion_operators"]) > 0):
            _str = append_lines(_str, self._import_components(
                build_unit["fusion_operators"]), 0)

        _str = append_lines(_str, self._import_stream_ports(), 0)
        _str = append_lines(
            _str, self._generate_build_unit_class(build_unit), 0)
        return _str

    def _generate_exec_code(self, build_unit):
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
        _str = append_lines(
            _str, self._import_build_unit(build_unit["name"], build_unit["class_name"]), 0)
        _str = append_lines(_str, self._generate_main(build_unit), 0)
        return _str

    def _import_rcl(self):
        _str = ""

        _str = append_lines(_str, "import rclpy", 0)
        _str = append_lines(
            _str, "from rclpy.executors import MultiThreadedExecutor", 0)
        return _str

    def _import_scl(self):
        _str = ""

        _str = append_lines(_str, "from scl.link import Link", 0)
        _str = append_lines(_str, "from scl.build_unit import BuildUnit", 0)
        return _str

    def _import_srvs(self):
        _str = ""
        for event_port in self._event_input_ports:

            event_name = self._find_event_name_for_input_port(event_port)
            _str = _str + "from {}_interfaces.srv import {}\n".format(
                self._pkg_name, CamelCaseConverter(event_name).__str__())
        return _str

    def _import_build_unit(self, name, class_name):
        _str = ""

        _str = append_lines(
            _str, "from .splash.build_unit.{} import {}".format(name, class_name), 0)
        return _str

    def _import_components(self, components):
        _str = ""
        for component in components:
            _str = append_lines(
                _str, "from ..component.{} import {}".format(component["name"], component["class_name"]), 0)

        return _str

    def _import_factories(self):
        _str = ""
        for factory in self._factories:
            _str = append_lines(
                _str, "from ..factory.{} import {}".format(factory["name"], factory["class_name"]), 0)

        return _str

    def _import_stream_ports(self):
        _str = ""
        _str = append_lines(_str, "from ..stream_port import *", 0)

        return _str

    def _generate_main(self, build_unit):
        _str = ""
        _str = append_lines(_str, "def main(args=None):", 0)
        _str = append_lines(_str, "build_unit = {}()".format(build_unit["class_name"]), 1)
        _str = append_lines(_str, "build_unit.run()", 1)
        return _str

    def _generate_build_unit_class(self, build_unit):
        _str = ""
        _str = append_lines(_str, "class {}(BuildUnit):".format(
            build_unit["class_name"]), 0)
        _str = append_lines(_str, "def __init__(self):", 1)
        _str = append_lines(_str, "super().__init__()", 2)
        for component in build_unit["processing_components"]:
            _str = append_lines(
                _str, self._generate_lines_for_component(component), 2)
        for component in build_unit["source_components"]:
            _str = append_lines(
                _str, self._generate_lines_for_component(component), 2)
        for component in build_unit["sink_components"]:
            _str = append_lines(
                _str, self._generate_lines_for_component(component), 2)
        for component in build_unit["fusion_operators"]:
            _str = append_lines(
                _str, self._generate_lines_for_component(component), 2)
        return _str

    def _generate_lines_for_component(self, component):
        _str = ""
        name = ""
        factory = "None"
        mode = ""
        name = component["name"]
        if "group" in component.keys():
            factory = "{}()".format(
                CamelCaseConverter(component["group"]).__str__())
            mode = component["mode"]
        links = []
        input_ports = component["stream_input_ports"]
        output_ports = component["stream_output_ports"]
        event_input_ports = component["event_input_ports"]
        for link in self._links_transparent:
            for input_port in input_ports:
                if input_port["key"] == link["to"]["key"]:
                    _from = CamelCaseConverter(link["from"]["key"]).__str__()
                    _to = CamelCaseConverter(link["to"]["key"]).__str__()
                    _channel = link["to"]["channel"]
                    links.append("Link({}(), {}(), \"{}\")".format(
                        _from, _to, _channel))
            for output_port in output_ports:
                if output_port["key"] == link["from"]["key"]:
                    _from = CamelCaseConverter(link["from"]["key"]).__str__()
                    _to = CamelCaseConverter(link["to"]["key"]).__str__()
                    _channel = link["from"]["channel"]
                    links.append("Link({}(), {}(), \"{}\")".format(
                        _from, _to, _channel))
        _str = append_lines(_str, "{} = {}()".format(
            component["name"], component["class_name"]), 0)
        links_str = str(links).replace("\'", "")

        _str = append_lines(_str, "{}.set_links(links={})".format(
            name, links_str), 0)
        _str = append_lines(_str, "{}.set_build_unit(self)".format(name), 0)
        if component["category"] == "fusionOperator":
            fusion_rule = component["fusionRule"]
            _str = append_lines(
                _str, "{}.set_fusion_rule(fusion_rule={})".format(name, fusion_rule), 0)
        _str = append_lines(_str, self._register_event_callback(
            component["name"], event_input_ports), 0)
        _str = append_lines(_str, "{}.setup()".format(component["name"]), 0)
        _str = append_lines(
            _str, "self.components.append({})".format(component["name"]), 0)
        return _str

    def _register_event_callback(self, component_name, event_input_ports):
        _str = ""
        for event_port in event_input_ports:
            event_name = self._find_event_name_for_input_port(event_port)
            _str = _str + "{0}.attach_event_input_port(\"{1}\", {0}.{2}_callback)".format(
                component_name, event_name, event_name.lower().replace(" ", "_"))
        return _str
