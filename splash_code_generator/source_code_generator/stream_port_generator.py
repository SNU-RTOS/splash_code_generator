from ._util import *
import copy


class StreamPortGenerator:
    def __init__(self, ports):
        self._ports = copy.deepcopy(ports)

    def generate(self):
        ports = []
        for port in self._ports:
            ports.append(self._generate_port(port))
        return ports

    def _generate_port(self, port):
        _port = {}
        _port["name"] = port["name"]
        _port["rate"] = port["Rate"] if "Rate" in port.keys() else None
        
        _port["type"] = port["PORT_TYPE"]
        _port["class_name"] = port["class_name"]
        _port["parent"] = port["group"]
        _port["source_code"] = self._generate_source_code(_port)

        return _port

    def _generate_source_code(self, port):
        _str = ""
        _str = append_lines(_str, "'''", 0)
        _str = append_lines(
            _str, "Generated automatically by Splash Code Generator for {}".format(port["name"]), 1)
        _str = append_lines(_str, "'''", 0)
        _str = append_lines(_str, self._import_scl(port), 0)
        _str = append_lines(_str, self._generate_port_class(port), 0)
        return _str

    def _import_scl(self, port):
        _str = ""
        port_type = ""
        if port["type"] == "STREAM_INPUT_PORT":
            port_type = "StreamInputPort"
        elif port["type"] == "STREAM_OUTPUT_PORT":
            port_type = "StreamOutputPort"
        _str = append_lines(
            _str, "from scl.channel import {}".format(port_type), 0)
        _str = append_lines(_str, "from ..component.{} import {}".format(port["parent"], CamelCaseConverter(
            port["parent"]).__str__()), 0) if port["parent"] else _str
        return _str

    def _generate_port_class(self, port):
        _str = ""
        port_type = ""
        if port["type"] == "STREAM_INPUT_PORT":
            port_type = "StreamInputPort"
        elif port["type"] == "STREAM_OUTPUT_PORT":
            port_type = "StreamOutputPort"
        parent = CamelCaseConverter(
            port["parent"]).__str__() + "()" if port["parent"] else None
        _str = append_lines(
            _str, "class {}({}):".format(port["class_name"], port_type), 0)
        _str = append_lines(_str, "def __init__(self):", 1)
        _str = append_lines(
            _str, "super().__init__(name=\"{}\", parent={})".format(port["name"], parent), 2)
        if port["rate"] and not self._rate_is_nan(port["rate"]):
            _str = append_lines(_str, "self.set_rate_constraint({})".format(port["rate"]), 2)

        return _str
    def _rate_is_nan(self, rate):
        if type(rate) == int:
            return False
        return True