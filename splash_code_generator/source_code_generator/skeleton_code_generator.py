from ._util import *
import copy


class SkeletonCodeGenerator:
    def __init__(self, node_data, link_data_list):
        node_data_cp = copy.deepcopy(node_data)
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
        self._links = link_data_list

    def generate(self):
        skeletons = []
        not_assigned_stream_ports = self._stream_ports
        not_assigned_event_input_ports = self._event_input_ports
        not_assigned_event_output_ports = self._event_output_ports
        not_assigned_modechange_output_ports = self._modechange_output_ports
        for component in self._processing_components:
            not_assigned_stream_ports = relate_stream_ports(
                component, not_assigned_stream_ports)
            not_assigned_event_input_ports = relate_event_input_ports(
                component, not_assigned_event_input_ports)
            not_assigned_event_output_ports = relate_event_output_ports(
                component, not_assigned_event_output_ports)
            not_assigned_modechange_output_ports = relate_modechange_output_ports(
                component, not_assigned_modechange_output_ports)
            component["factory"] = relate_factory(component, self._factories)
        for component in self._source_components:
            not_assigned_stream_ports = relate_stream_ports(
                component, not_assigned_stream_ports)
            not_assigned_event_input_ports = relate_event_input_ports(
                component, not_assigned_event_input_ports)
            not_assigned_event_output_ports = relate_event_output_ports(
                component, not_assigned_event_output_ports)
            not_assigned_modechange_output_ports = relate_modechange_output_ports(
                component, not_assigned_modechange_output_ports)
            component["factory"] = relate_factory(component, self._factories)
        for component in self._sink_components:
            not_assigned_stream_ports = relate_stream_ports(
                component, not_assigned_stream_ports)
            not_assigned_event_input_ports = relate_event_input_ports(
                component, not_assigned_event_input_ports)
            not_assigned_event_output_ports = relate_event_output_ports(
                component, not_assigned_event_output_ports)
            not_assigned_modechange_output_ports = relate_modechange_output_ports(
                component, not_assigned_modechange_output_ports)
            component["factory"] = relate_factory(component, self._factories)
        for component in self._fusion_operators:
            not_assigned_stream_ports = relate_stream_ports(
                component, not_assigned_stream_ports)
            not_assigned_event_input_ports = relate_event_input_ports(
                component, not_assigned_event_input_ports)
            not_assigned_event_output_ports = relate_event_output_ports(
                component, not_assigned_event_output_ports)
            not_assigned_modechange_output_ports = relate_modechange_output_ports(
                component, not_assigned_modechange_output_ports)
            component["factory"] = relate_factory(component, self._factories)

        for component in self._processing_components:
            skeletons.append(self._generate_skeleton(component))
        for component in self._source_components:
            skeletons.append(self._generate_skeleton(component))
        for component in self._sink_components:
            skeletons.append(self._generate_skeleton(component))
        for component in self._fusion_operators:
            skeletons.append(self._generate_skeleton(component))

        return skeletons

    def _generate_skeleton(self, component):
        skeleton = {}
        skeleton["name"] = component["name"]
        skeleton["source_code"] = self._generate_source_code(component)
        return skeleton

    def _generate_source_code(self, component):
        _str = ""
        _str = append_lines(_str, "'''", 0)
        _str = append_lines(
            _str, "Generated automatically by Splash Code Generator for {}".format(component["name"]), 1)
        _str = append_lines(_str, "'''", 0)
        _str = append_lines(_str, self._import_message(component), 0)
        _str = append_lines(_str, self._import_scl(), 0)
        if component["factory"]:
            _str = append_lines(_str, self._import_factory(component), 0)
        _str = append_lines(_str, self._generate_class(component), 0)
        return _str

    def _import_message(self, component):
        _str = ""
        message_type_list = []
        for output_port in component["stream_output_ports"]:
            if "MessageType" in output_port.keys():
                message_type_list.append(output_port["MessageType"])
        message_type_list = list(set(message_type_list))
        message_type_str = ""
        i = 0
        for message_type in message_type_list:
            if i > 0:
                message_type_str += ", "
            message_type_str += message_type
            i += 1
        _str = append_lines(_str, "from std_msgs.msg import {}".format(message_type_str), 0)
        return _str

    def _import_factory(self, component):
        _str = ""
        factory = component["factory"]
        _str = append_lines(_str, "from ..factory.{} import {}".format(
            factory["name"], factory["class_name"]), 0)
        return _str

    def _import_scl(self):
        _str = ""
        _str = append_lines(_str, "from scl.components import *", 0)
        _str = append_lines(_str, "from scl.impl.singleton import Singleton", 0)
        return _str

    def _generate_class(self, component):
        _str = ""
        if(component["category"] == "fusionOperator"):
            _str = append_lines(_str, "class {}(FusionOperator, metaclass=Singleton):".format(
                component["class_name"]), 0)
        else:
            _str = append_lines(_str, "class {}(Component, metaclass=Singleton):".format(
                component["class_name"]), 0)
        _str = append_lines(_str, self._generate_init(component), 1)
        _str = append_lines(_str, self._generate_setup(component), 1)
        _str = append_lines(_str, self._generate_run(), 1)
        if(component["category"] != "fusionOperator"):
            _str = append_lines(_str, self._generate_user_callbacks(component), 1)
        _str = append_lines(
            _str, self._generate_event_callbacks(component), 1)
        return _str

    def _generate_init(self, component):
        _str = ""
        _str = append_lines(_str, "def __init__(self):", 0)
        factory = component["factory"]["class_name"] + \
            "()" if component["factory"] else None
        mode = "\"{}\"".format(component["mode"]) if "mode" in component.keys(
        ) else None
        _str = append_lines(
            _str, "super().__init__(name=\"{}\", factory={}, mode={})".format(component["name"], factory, mode), 1)
        return _str

    def _generate_setup(self, component):
        _str = ""
        _str = append_lines(_str, "def setup(self):", 0)
        from_fusion_flag = False
        if(component["category"] == "fusionOperator"):
            for input_port in component["stream_input_ports"]:
                pair = self._find_output_port_for_input_port(input_port)
                channel = pair["Channel"]
                message_type = ""
                if "MessageType" in pair.keys():
                    message_type = pair["MessageType"]
                for fusion_operator in self._fusion_operators:
                    if pair in fusion_operator["stream_output_ports"]:
                        from_fusion_flag = True
                        break
                _str = append_lines(
                    _str, self._append_input_port_for_fusion(channel, message_type, from_fusion_flag), 1)
        else:
            count = 1
            for input_port in component["stream_input_ports"]:
                pair = self._find_output_port_for_input_port(input_port)
                channel = pair["Channel"]
                message_type = ""
                if "MessageType" in pair.keys():
                    message_type = pair["MessageType"]
                
                for fusion_operator in self._fusion_operators:
                    if pair in fusion_operator["stream_output_ports"]:
                        from_fusion_flag = True
                        break
                _str = append_lines(_str, self._append_input_port(
                    channel, message_type, "user_callback_{}".format(count), from_fusion_flag), 1)
                count = count + 1
        for output_port in component["stream_output_ports"]:
            channel = output_port["Channel"]
            message_type = ""
            if "MessageType" in output_port.keys():
                message_type = output_port["MessageType"]
            if(component["category"] == "fusionOperator"):
                _str = append_lines(
                    _str, self._append_output_port_for_fusion(channel), 1)
            else:
                _str = append_lines(
                    _str, self._append_output_port(channel, message_type), 1)
        return _str

    def _generate_user_callbacks(self, component):
        _str = ""
        count = 1
        for input_port in component["stream_input_ports"]:
            _str = append_lines(
                _str, "def user_callback_{}(self, msg):".format(count), 0)
            _str = append_lines(_str, "pass\n", 1)
            count = count + 1
        return _str

    def _generate_event_callbacks(self, component):
        _str = ""
        for input_port in component["event_input_ports"]:
            event_name = self._find_event_name_for_input_port(input_port)
            _str = append_lines(
                _str, "def {}_callback(self, event, response):".format(event_name.lower().replace(" ", "_")), 0)
            _str = append_lines(_str, "return response\n", 1)
        return _str

    def _generate_run(self):
        _str = ""
        _str = append_lines(_str, "def run(self):", 0)
        _str = append_lines(_str, "pass", 1)
        return _str

    def _append_input_port(self, channel, message_type, user_callback_name, from_fusion=False):
        _str = ""
        if from_fusion:
            _str = "self.attach_stream_input_port(channel=\"{}\", callback=self.{}, from_fusion={})".format(channel, user_callback_name, from_fusion)
        else:
            _str = "self.attach_stream_input_port(msg_type={}, channel=\"{}\", callback=self.{})".format(message_type, channel, user_callback_name)
        return _str

    def _append_input_port_for_fusion(self, channel, message_type, from_fusion=False):
        _str = ""
        if from_fusion:
            _str = "self.attach_stream_input_port(channel=\"{}\", from_fusion={})".format(channel, from_fusion)
        else:
            _str = "self.attach_stream_input_port(msg_type={}, channel=\"{}\")".format(message_type, channel)
        return _str

    def _find_output_port_for_input_port(self, input_port):
        for link in self._links:
            if(link["to"] == input_port["key"]):
                for stream_port in self._stream_ports:
                    if(stream_port["PORT_TYPE"] == "STREAM_OUTPUT_PORT" and stream_port["key"] == link["from"]):
                        return stream_port
        return None
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
                for event_port in self._event_output_ports:
                    if(event_port["key"] == link["from"]):
                        return event_port["Event"]
        return None

    def _append_output_port(self, channel, message_type):
        _str = ""
        _str = "self.attach_stream_output_port(msg_type={}, channel=\"{}\")".format(message_type, channel)
        return _str

    def _append_output_port_for_fusion(self, channel):
        _str = ""
        _str = "self.attach_stream_output_port(channel=\"{}\")".format(channel)
        return _str
