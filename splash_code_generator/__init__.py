from .ros_package_generator import ROSPackageGenerator
from .json_parser import JsonParser
from .source_code_generator import SourceCodeGenerator
from .source_code_validator import SourceCodeValidator


class CodeGenerator():
    def __init__(self, args):
        rosPackageGenerator = ROSPackageGenerator(args.name, args.path)

        jsonParser = JsonParser(args.file)
        json_parsed = jsonParser.parse()

        self._sourceCodeGenerator = SourceCodeGenerator(json_parsed)

        self._sourceCodeValidator = SourceCodeValidator()

        self.source_code = None

    def generate(self):
        self.source_code = self._sourceCodeGenerator.generate()

    def validate(self):
        if(self.source_code):
            self.validation = self.sourceCodeValidator.validate(source_code)
        else:
            print("Source code not generated yet")
