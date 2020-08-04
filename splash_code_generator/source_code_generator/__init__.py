from .build_unit_generator import BuildUnitGenerator
from .skeleton_code_generator import SkeletonCodeGenerator


class SourceCodeGenerator:
    def __init__(self, json):
        node_data_list = json["nodeDataArray"]
        link_data_list = json["linkDataArray"]

        self.buildUnitGenerator = BuildUnitGenerator(
            node_data_list, link_data_list)
        self.skeletonCodeGenerator = SkeletonCodeGenerator(
            node_data_list, link_data_list)

    def generate(self):
        self.build_units = self.buildUnitGenerator.generate()
        self.skeletons = self.skeletonCodeGenerator.generate()
