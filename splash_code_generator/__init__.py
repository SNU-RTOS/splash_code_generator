from .ros_package_generator import ROSPackageGenerator
from .json_parser import JsonParser
from .source_code_generator import SourceCodeGenerator
from .source_code_validator import SourceCodeValidator
from .source_code_generator._util import append_lines, CamelCaseConverter

import os


class CodeGenerator():
    def __init__(self, args):
        rosPackageGenerator = ROSPackageGenerator(args.name, args.path)

        jsonParser = JsonParser(args.file)
        json_parsed = jsonParser.parse()

        self._sourceCodeGenerator = SourceCodeGenerator(
            args.name, json_parsed)

        self._sourceCodeValidator = SourceCodeValidator()

        self._pkg_name = args.name
        self._pkg_path = os.path.join(args.path, 'src', args.name)
        self._interface_pkg_path = os.path.join(
            args.path, 'src', args.name+'_interfaces')
        self._node_dir = os.path.join(self._pkg_path, self._pkg_name)

        self._splash_dir = os.path.join(self._node_dir, "splash")

        self.source_code = None

    def generate(self):
        self.source_code = self._sourceCodeGenerator.generate()

        # make splash dir
        try:
            if not(os.path.isdir(self._splash_dir)):
                os.makedirs(self._splash_dir)
        except OSError as e:
            print(e)
        # make __init__.py in splash
        file_path = os.path.join(self._node_dir, "__init__.py")
        with open(file_path, "w") as f:
            f.write(self._generate_splash_server_source_code())
        # make __init__.py in splash
        file_path = os.path.join(self._splash_dir, "__init__.py")
        with open(file_path, "w") as f:
            f.write("")
        # make __init__.py in splash/build_unit
        self._locate_build_unit_source_code("__init__", "")
        # make __init__.py in splash/factory
        self._locate_factory_source_code("__init__", "")
        # make __init__.py in splash/stream_port
        self._locate_stream_port_source_code("__init__", "")
        # make __init__.py in spalsh/component
        self._locate_skeleton_source_code("__init__", "")
        self._generate_setup_py(self.source_code["build_units"])
        self._generate_srvs(self.source_code["build_units"])

        for build_unit in self.source_code["build_units"]:
            # locate source code for each build unit in splash/build_unit
            self._locate_build_unit_source_code(
                build_unit["name"], build_unit["source_code"])
            # locate exec code for each build unit in node_dir
            self._locate_build_unit_exec_code(
                build_unit["name"], build_unit["exec_code"])

        for port in self.source_code["stream_ports"]:
            # locate source code for each port in splash/port
            self._locate_stream_port_source_code(
                "__init__", port["source_code"])

        for factory in self.source_code["factories"]:
            # locate source code for each factory in splash/factory
            self._locate_factory_source_code(
                factory["name"], factory["source_code"])

        for skeleton in self.source_code["skeletons"]:
            # locate skeleton source code in splash/component
            self._locate_skeleton_source_code(
                skeleton["name"], skeleton["source_code"])

    def validate(self):
        if(self.source_code):
            self.validation = self._sourceCodeValidator.validate(
                self.source_code)
        else:
            print("Source code not generated yet")
    def _generate_splash_server_source_code(self):
        _str = ""
        _str = append_lines(_str, "import sys", 0)
        _str = append_lines(_str, "sys.path.append(\"C:/Workspace/rtos/Splash/RuntimeLibraries\")", 0)
        _str = append_lines(_str, "import srl", 0)
        _str = append_lines(_str, "def main():", 0)
        _str = append_lines(_str, "srl.run()", 1)
        return _str
    def _generate_setup_py(self, build_units):
        _str = ""
        console_scripts = ["splash_server = {}.__init__:main".format(self._pkg_name)]
        for build_unit in build_units:
            console_scripts.append("{0} = {1}.{0}_exec:main".format(
                build_unit["name"], self._pkg_name))
        
        _str = append_lines(
            _str, "from setuptools import setup, find_packages", 0)
        _str = append_lines(_str, "", 0)
        _str = append_lines(
            _str, "package_name = '{}'".format(self._pkg_name), 0)
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
        file_path = os.path.join(self._pkg_path, "setup.py")
        with open(file_path, "w") as f:
            f.write(_str)

    def _generate_srvs(self, build_units):
        srv_dir = os.path.join(self._interface_pkg_path, "srv")
        try:
            if not(os.path.isdir(srv_dir)):
                os.makedirs(srv_dir)
        except OSError as e:
            print(e)
        srv_str = "# Auto-generated srv file by Splash Code Generator\n"
        srv_str = srv_str + "---\n"
        srv_str = srv_str + "bool success\n"
        event_list = []
        file_path_rm = os.path.join(srv_dir, "RegisterMode.srv")
        with open(file_path_rm, "w") as f:
            srv_str_rm = "# Auto-generated srv file by Splash Code Generator\n"
            srv_str_rm = srv_str_rm + "string factory\n"
            srv_str_rm = srv_str_rm + "string mode_configuration\n"
            srv_str_rm = srv_str_rm + "---\n"
            srv_str_rm = srv_str_rm + "bool ok\n"
            f.write(srv_str_rm)
        event_list.append("RegisterMode")

        file_path_rmc = os.path.join(srv_dir, "RequestModeChange.srv")
        with open(file_path_rmc, "w") as f:
            srv_str_rmc = "# Auto-generated srv file by Splash Code Generator\n"
            srv_str_rmc = srv_str_rmc + "string factory\n"
            srv_str_rmc = srv_str_rmc + "string event\n"
            srv_str_rmc = srv_str_rmc + "---\n"
            srv_str_rmc = srv_str_rmc + "bool ok\n"
            f.write(srv_str_rmc)
        event_list.append("RequestModeChange")

        for build_unit in build_units:
            for component in build_unit["processing_components"]:
                for event_port in component["event_output_ports"]:
                    event_name = CamelCaseConverter(
                        event_port["Event"]).__str__()
                    event_list.append(event_name)
                    file_path = os.path.join(
                        srv_dir, event_name+".srv")
                    with open(file_path, "w") as f:
                        f.write(srv_str)
        self._generate_interfaces_xml()
        self._generate_interfaces_cmake(event_list)

    def _generate_interfaces_xml(self):
        _str = ""
        _str = append_lines(_str, "<?xml version=\"1.0\"?>", 0)
        _str = append_lines(
            _str, "<?xml-model href=\"http://download.ros.org/schema/package_format3.xsd\" schematypens=\"http://www.w3.org/2001/XMLSchema\"?>", 0)
        _str = append_lines(_str, "<package format=\"3\">", 0)
        _str = append_lines(
            _str, "<name>{}_interfaces</name>".format(self._pkg_name), 1)
        _str = append_lines(_str, "<version>0.0.0</version>", 1)
        _str = append_lines(
            _str, "<description>TODO: Package description</description>", 1)
        _str = append_lines(
            _str, "<maintainer email=\"you@email.com\">Your name</maintainer>", 1)
        _str = append_lines(
            _str, "<license>TODO: License declaration</license>", 1)

        _str = append_lines(
            _str, "<buildtool_depend>ament_cmake</buildtool_depend>", 1)
        _str = append_lines(
            _str, "<buildtool_depend>rosidl_default_generators</buildtool_depend>", 1)
        _str = append_lines(
            _str, "<exec_depend>rosidl_default_runtime</exec_depend>", 1)
        _str = append_lines(
            _str, "<member_of_group>rosidl_interface_packages</member_of_group>", 1)
        _str = append_lines(
            _str, "<test_depend>ament_lint_auto</test_depend>", 1)
        _str = append_lines(
            _str, "<test_depend>ament_lint_common</test_depend>", 1)
        _str = append_lines(_str, "<export>", 1)
        _str = append_lines(_str, "<build_type>ament_cmake</build_type>", 2)
        _str = append_lines(_str, "</export>", 1)
        _str = append_lines(_str, "</package>", 0)

        file_path = os.path.join(self._interface_pkg_path, "package.xml")

        with open(file_path, "w") as f:
            f.write(_str)

    def _generate_interfaces_cmake(self, event_list):
        _str = ""
        _str = append_lines(_str, "cmake_minimum_required(VERSION 3.5)", 0)
        _str = append_lines(_str, "project(test_pkg_interfaces)", 0)
        _str = append_lines(_str, "# Default to C99", 0)
        _str = append_lines(_str, "if(NOT CMAKE_C_STANDARD)", 0)
        _str = append_lines(_str, "set(CMAKE_C_STANDARD 99)", 1)
        _str = append_lines(_str, "endif()", 0)

        _str = append_lines(_str, "# Default to C++14", 0)
        _str = append_lines(_str, "if(NOT CMAKE_CXX_STANDARD)", 0)
        _str = append_lines(_str, "set(CMAKE_CXX_STANDARD 14)", 1)
        _str = append_lines(_str, "endif()", 0)

        _str = append_lines(
            _str, "if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES \"Clang\")", 0)
        _str = append_lines(
            _str, "add_compile_options(-Wall -Wextra -Wpedantic)", 1)
        _str = append_lines(_str, "endif()", 0)

        _str = append_lines(_str, "# find dependencies", 0)
        _str = append_lines(_str, "find_package(ament_cmake REQUIRED)", 0)
        _str = append_lines(
            _str, "find_package(rosidl_default_generators REQUIRED)", 0)

        _str = append_lines(
            _str, "rosidl_generate_interfaces(${PROJECT_NAME}", 0)
        for event in event_list:
            _str = append_lines(_str, "\"srv/{}.srv\"".format(event), 1)
        _str = append_lines(_str, ")", 0)

        _str = append_lines(
            _str, "ament_export_dependencies(rosidl_default_runtime)", 0)
        _str = append_lines(_str, "ament_package()", 0)

        file_path = os.path.join(self._interface_pkg_path, "CMakeLists.txt")

        with open(file_path, "w") as f:
            f.write(_str)

    def _locate_build_unit_source_code(self, name, code):
        build_unit_dir = os.path.join(self._splash_dir, "build_unit")
        try:
            if not(os.path.isdir(build_unit_dir)):
                os.makedirs(build_unit_dir)
        except OSError as e:
            print(e)
        file_path = os.path.join(build_unit_dir, name+".py")
        with open(file_path, "w") as f:
            f.write(code)

    def _locate_build_unit_exec_code(self, name, code):
        file_path = os.path.join(self._node_dir, name+"_exec.py")
        with open(file_path, "w") as f:
            f.write(code)

    def _locate_factory_source_code(self, name, code):
        factory_dir = os.path.join(self._splash_dir, "factory")
        try:
            if not(os.path.isdir(factory_dir)):
                os.makedirs(factory_dir)
        except OSError as e:
            print(e)
        file_path = os.path.join(factory_dir, name+".py")
        with open(file_path, "w") as f:
            f.write(code)

    def _locate_stream_port_source_code(self, name, code):
        stream_port_dir = os.path.join(self._splash_dir, "stream_port")
        try:
            if not(os.path.isdir(stream_port_dir)):
                os.makedirs(stream_port_dir)
        except OSError as e:
            print(e)
        file_path = os.path.join(stream_port_dir, name+".py")
        with open(file_path, "a" if code else "w") as f:
            f.write(code)

    def _locate_skeleton_source_code(self, name, code):
        skeleton_dir = os.path.join(self._splash_dir, "component")
        try:
            if not(os.path.isdir(skeleton_dir)):
                os.makedirs(skeleton_dir)
        except OSError as e:
            print(e)
        file_path = os.path.join(skeleton_dir, name+".py")
        with open(file_path, "w") as f:
            f.write(code)
