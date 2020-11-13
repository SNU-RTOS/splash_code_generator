import re

from .build_unit_generator import BuildUnitGenerator
from .factory_generator import FactoryGenerator
from .skeleton_code_generator import SkeletonCodeGenerator
from .stream_port_generator import StreamPortGenerator
from ._util import *


class SourceCodeGenerator:
    def __init__(self, pkg_name, json):
        try:
            node_data_list = json["nodeDataArray"] 
        except:
            node_Data_list = []
        try:
            link_data_list = json["linkDataArray"]
        except:
            link_data_list = [] 
        node_data_parsed = self._parse_node_data(node_data_list)
        try:
            factory_list = node_data_parsed["factories"]
        except:
            factory_list = []
        try:
            stream_port_list = node_data_parsed("stream_ports")
        except:
            stream_port_list = []

        self.buildUnitGenerator = BuildUnitGenerator(
            pkg_name, node_data_parsed, link_data_list)
        
        self.factoryGenerator = FactoryGenerator(factory_list)
        self.streamPortGenerator = StreamPortGenerator(stream_port_list)
        self.skeletonCodeGenerator = SkeletonCodeGenerator(
            node_data_parsed, link_data_list)

    def generate(self):
        build_units = self.buildUnitGenerator.generate()
        factories = self.factoryGenerator.generate()
        stream_ports = self.streamPortGenerator.generate()
        skeletons = self.skeletonCodeGenerator.generate()

        return {"build_units": build_units, "factories": factories, "stream_ports": stream_ports, "skeletons": skeletons}

    def _parse_node_data(self, node_data_list):
        processing_components = []
        source_components = []
        sink_components = []
        fusion_operators = []
        factories = []
        build_units = []
        stream_ports = []
        event_input_ports = []
        event_output_ports = []
        modechange_input_ports = []
        modechange_output_ports = []
        for node in node_data_list:
            category = node["category"]
            node["name"] = node["key"].lower().replace(" ", "_")

            class_name = CamelCaseConverter(node["name"]).__str__()
            node["class_name"] = class_name

            if(category == "processingComponent"):
                processing_components.append(node)
            elif(category == "sourceComponent"):
                source_components.append(node)
            elif(category == "sinkComponent"):
                sink_components.append(node)
            elif(category == "fusionOperator"):
                fusion_operators.append(node)
            elif(category == "factory"):
                factories.append(node)
            elif(category == "buildUnit"):
                build_units.append(node)
            elif(category == "streamPort"):
                stream_ports.append(node)
            elif(category == "eventInputPort"):
                event_input_ports.append(node)
            elif(category == "eventOutputPort"):
                event_output_ports.append(node)
            elif(category == "modeChangeInputPort"):
                modechange_input_ports.append(node)
            elif(category == "modeChangeOutputPort"):
                modechange_output_ports.append(node)
            else:
                print("Error")

        node_data = {
            "processing_components": processing_components,
            "source_components": source_components,
            "sink_components": sink_components,
            "fusion_operators": fusion_operators,
            "factories": factories,
            "build_units": build_units,
            "stream_ports": stream_ports,
            "event_input_ports": event_input_ports,
            "event_output_ports": event_output_ports,
            "modechange_input_ports": modechange_input_ports,
            "modechange_output_ports": modechange_output_ports,
        }

        return node_data
