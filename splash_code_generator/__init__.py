from .ros_package_generator import ROSPackageGenerator
from .json_parser import JsonParser
from .source_code_generator import SourceCodeGenerator
from .source_code_validator import SourceCodeValidator


class CodeGenerator():
    def __init__(self, args):
        self.ros_package_generator = ROSPackageGenerator(args.name, args.path)
        self.json_parser = JsonParser(args.file)
        self.source_code_generator = SourceCodeGenerator(
            self.json_parser.get_json_decoded())
        self.source_code_validator = SourceCodeValidator()

    def generate(self):
        pass

    def validate(self):
        pass
