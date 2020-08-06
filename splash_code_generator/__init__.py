from .ros_package_generator import ROSPackageGenerator
from .json_parser import JsonParser
from .source_code_generator import SourceCodeGenerator
from .source_code_validator import SourceCodeValidator
from .source_code_generator.__util import append_lines

import os


class CodeGenerator():
    def __init__(self, args):
        rosPackageGenerator = ROSPackageGenerator(args.name, args.path)

        jsonParser = JsonParser(args.file)
        json_parsed = jsonParser.parse()

        self.__sourceCodeGenerator = SourceCodeGenerator(json_parsed)

        self.__sourceCodeValidator = SourceCodeValidator()

        self.__pkg_name = args.name
        self.__pkg_path = os.path.join(args.path, 'src', args.name)

        self.__node_dir = os.path.join(self.__pkg_path, self.__pkg_name)

        self.__splash_dir = os.path.join(self.__node_dir, "splash")

        self.source_code = None

    def generate(self):
        self.source_code = self.__sourceCodeGenerator.generate()

        # make splash dir
        try:
            if not(os.path.isdir(self.__splash_dir)):
                os.makedirs(self.__splash_dir)
        except OSError as e:
            print(e)
        # make __init__.py in splash
        file_path = os.path.join(self.__splash_dir, "__init__.py")
        with open(file_path, "w") as f:
            f.write("")
        # make __init__.py in splash/build_unit
        self.__locate_build_unit_source_code("__init__", "")
        # make __init__.py in spalsh/component
        self.__locate_skeleton_source_code("__init__", "")
        self.__generate_setup_py(self.source_code["build_units"])
        for build_unit in self.source_code["build_units"]:
            # locate source code for each build unit in splash/build_unit
            self.__locate_build_unit_source_code(
                build_unit["name"], build_unit["source_code"])
            # locate exec code for each build unit in node_dir
            self.__locate_build_unit_exec_code(
                build_unit["name"], build_unit["exec_code"])
        for skeleton in self.source_code["skeletons"]:
            # locate skeleton source code in splash/component
            self.__locate_skeleton_source_code(
                skeleton["name"], skeleton["source_code"])

    def validate(self):
        if(self.source_code):
            self.validation = self.__sourceCodeValidator.validate(
                self.source_code)
        else:
            print("Source code not generated yet")

    def __generate_setup_py(self, build_units):
        _str = ""
        console_scripts = []
        for build_unit in build_units:
            console_scripts.append("{0} = {1}.{0}_exec:main".format(
                build_unit["name"], self.__pkg_name))
        _str = append_lines(
            _str, "from setuptools import setup, find_packages", 0)
        _str = append_lines(_str, "", 0)
        _str = append_lines(
            _str, "package_name = '{}'".format(self.__pkg_name), 0)
        _str = append_lines(_str, "setup(", 0)
        _str = append_lines(_str, "name=package_name,", 1)
        _str = append_lines(_str, "version='0.0.0',", 1)
        _str = append_lines(_str, "packages=find_packages(),", 1)
        _str = append_lines(
            _str, "data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml']),],", 1)
        _str = append_lines(_str, "install_requires=['setuptools'],", 1)
        _str = append_lines(_str, "zip_safe=True,", 1)
        _str = append_lines(_str, "maintainer='TODO: Your name',", 1)
        _str = append_lines(_str, "maintainer_email='TODO: Your email',", 1)
        _str = append_lines(
            _str, "description='TODO: Package description',", 1)
        _str = append_lines(_str, "license='TODO: License declaration',", 1)
        _str = append_lines(_str, "tests_require=['pytest'],", 1)
        _str = append_lines(
            _str, "entry_points={{'console_scripts': {0},}},".format(console_scripts), 1)
        _str = append_lines(_str, ")", 0)
        # make __init__.py in splash
        file_path = os.path.join(self.__pkg_path, "setup.py")
        with open(file_path, "w") as f:
            f.write(_str)

    def __locate_build_unit_source_code(self, name, code):
        build_unit_dir = os.path.join(self.__splash_dir, "build_unit")
        try:
            if not(os.path.isdir(build_unit_dir)):
                os.makedirs(build_unit_dir)
        except OSError as e:
            print(e)
        file_path = os.path.join(build_unit_dir, name+".py")
        with open(file_path, "w") as f:
            f.write(code)

    def __locate_build_unit_exec_code(self, name, code):
        file_path = os.path.join(self.__node_dir, name+"_exec.py")
        with open(file_path, "w") as f:
            f.write(code)

    def __locate_skeleton_source_code(self, name, code):
        skeleton_dir = os.path.join(self.__splash_dir, "component")
        try:
            if not(os.path.isdir(skeleton_dir)):
                os.makedirs(skeleton_dir)
        except OSError as e:
            print(e)
        file_path = os.path.join(skeleton_dir, name+".py")
        with open(file_path, "w") as f:
            f.write(code)
