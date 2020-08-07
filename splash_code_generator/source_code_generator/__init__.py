import re

from .build_unit_generator import BuildUnitGenerator
from .skeleton_code_generator import SkeletonCodeGenerator
from .__util import *


class SourceCodeGenerator:
    def __init__(self, pkg_name, json):
        node_data_list = json["nodeDataArray"]
        link_data_list = json["linkDataArray"]
        node_data_parsed = self.__parse_node_data(node_data_list)

        self.buildUnitGenerator = BuildUnitGenerator(
            pkg_name, node_data_parsed, link_data_list)
        self.skeletonCodeGenerator = SkeletonCodeGenerator(
            node_data_parsed, link_data_list)

    def generate(self):
        build_units = self.buildUnitGenerator.generate()
        skeletons = self.skeletonCodeGenerator.generate()
        return {"build_units": build_units, "skeletons": skeletons}

    def __parse_node_data(self, node_data_list):
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
                modechange_output_ports.append(ndoe)
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
