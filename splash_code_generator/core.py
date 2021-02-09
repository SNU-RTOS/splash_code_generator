from .json_parser import JsonParser
from .source_code_generator import SourceCodeGenerator
from ._util import append_lines, CamelCaseConverter

import os, json, subprocess


class Core():
    def __init__(self, args):
        self._email = "TODO: Your email"
        self._username = "TODO: Your name"
        if(args.update):
            print('update')
            return
        if(args.email):
            self._email = args.email
        if(args.username):
            self._username = args.username


        jsonParser = JsonParser(args.file)
        json_parsed = jsonParser.parse()
        self._json_file_path = args.file
        self._ws_path = args.path
        self._sourceCodeGenerator = SourceCodeGenerator(
            args.username, args.name, json_parsed)
        self._node_list = json_parsed['nodeDataArray']
        self._link_list = json_parsed['linkDataArray']

        self._pkg_name = args.name
        self._pkg_path = os.path.join(args.path, 'src', args.name)
        # self._interface_pkg_path = os.path.join(
        #     args.path, 'src', args.name+'_interfaces')
        self._node_dir = os.path.join(self._pkg_path, self._pkg_name)

        self._splash_dir = os.path.join(self._node_dir, "splash")
        
        self.source_code = None
        
    def _generate_package(self, name, path):
        command = "ros2 pkg create --build-type ament_python %s --dependencies rclpy scl" % (name)

        process = subprocess.Popen(command.split(), cwd=path + "/src",
                                   stdin=subprocess.PIPE, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
        out, err = process.communicate()
        if(out):
            print(out.decode('utf-8').strip())
        elif(err):
            print(err.decode('utf-8').strip())

        
    def generate(self):
        self._generate_package(self._pkg_name, self._ws_path)

        self._sourceCodeGenerator.generate()
        
        # make  splash dir
        try:
            if not(os.path.isdir(self._splash_dir)):
                os.makedirs(self._splash_dir)
        except OSError as e:
            print(e)
        # make splash_server.py in node
        file_path = os.path.join(self._node_dir, "splash_server.py")
        with open(file_path, "w") as f:
            f.write(self._generate_splash_server_source_code())
        # make __init__.py in splash
        file_path = os.path.join(self._splash_dir, "__init__.py")
        with open(file_path, "w") as f:
            f.write("")
        # make __init__.py in splash/build_unit
        self._generate_splash_factory_structure()
        self._locate_componenets()
        self._locate_build_units()
        self._generate_setup_py()
        self._generate_package_xml()
        self._generate_launch()

    def _generate_splash_server_source_code(self):
        _str = ""
        # _str = append_lines(_str, "import sys", 0)
        # _str = append_lines(_str, "sys.path.append(\"C:/Workspace/rtos/Splash/RuntimeLibraries\")", 0)
        _str = append_lines(_str, "import scl", 0)
        _str = append_lines(_str, "def main():", 0)
        _str = append_lines(_str, "scl.init()", 1)
        return _str

    def _generate_package_xml(self):
        _str = ""
        _str = append_lines(_str, "<?xml version=\"1.0\"?>", 0)
        _str = append_lines(
            _str, "<?xml-model href=\"http://download.ros.org/schema/package_format3.xsd\" schematypens=\"http://www.w3.org/2001/XMLSchema\"?>", 0)
        _str = append_lines(_str, "<package format=\"3\">", 0)
        _str = append_lines(
            _str, "<name>{}</name>".format(self._pkg_name), 1)
        _str = append_lines(_str, "<version>0.0.0</version>", 1)
        _str = append_lines(
            _str, "<description>TODO: Package description</description>", 1)
        _str = append_lines(
            _str, f"<maintainer email=\"{self._email}\">{self._username}</maintainer>", 1)
        _str = append_lines(
            _str, "<license>TODO: License declaration</license>", 1)

        _str = append_lines(
            _str, "<depend>rclpy</depend>", 1)
        _str = append_lines(
            _str, "<depend>scl</depend>", 1)
        _str = append_lines(_str, "<export>", 1)
        _str = append_lines(_str, "<build_type>ament_python</build_type>", 2)
        _str = append_lines(_str, "</export>", 1)
        _str = append_lines(_str, "</package>", 0)

        file_path = os.path.join(self._pkg_path, "package.xml")

        with open(file_path, "w") as f:
            f.write(_str)

    def _generate_launch(self):
        _str = ""
        _str = append_lines(_str, "from launch import LaunchDescription", 0)
        _str = append_lines(_str, "from launch_ros.actions import Node", 0)
        _str = append_lines(_str, "def generate_launch_description():", 0)
        _str = append_lines(_str, "return LaunchDescription([", 1) 
        _str = append_lines(_str, "Node(package='{}', node_executable='splash_server'),".format(self._pkg_name), 2)
        for build_unit in self._sourceCodeGenerator.build_units_dict.values():
            _str = append_lines(_str, "Node(package='{}', node_executable='{}'),".format(self._pkg_name, build_unit["key"]), 2)
        _str = append_lines(_str, "])", 1)

        launch_dir = os.path.join(self._pkg_path, "launch")
        try:
            if not(os.path.isdir(launch_dir)):
                os.makedirs(launch_dir)
        except OSError as e:
            print(e)

        file_path = os.path.join(launch_dir, "splash.launch.py")
        with open(file_path, "w") as f:
            f.write(_str)

    def _generate_setup_py(self):
        _str = ""
        console_scripts = ["splash_server = {}.splash_server:main".format(self._pkg_name)]
        for build_unit in self._sourceCodeGenerator.build_units_dict.values():
            console_scripts.append("{0} = {1}.{0}_exec:main".format(
                build_unit["key"], self._pkg_name))
        
        _str = append_lines(_str, "import os", 0)
        _str = append_lines(_str, "from glob import glob", 0)
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
            _str, "data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml']), (os.path.join('share', package_name), glob('launch/*.launch.py'))],", 1)
        _str = append_lines(_str, "install_requires=['setuptools'],", 1)
        _str = append_lines(_str, "zip_safe=True,", 1)
        _str = append_lines(_str, f"maintainer='{self._username}',", 1)
        _str = append_lines(_str, f"maintainer_email='{self._email}',", 1)
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

    def _generate_splash_factory_structure(self):
        for tree in self._sourceCodeGenerator.forest:
            os.makedirs(self._splash_dir + '/root_factory/' + tree['leap_path'])
            for (root, dirs, files) in os.walk(self._splash_dir + '/root_factory'):
                with open(root + '/' + '__init__.py', 'w') as f:
                    f.write('')
    
    def _locate_componenets(self):
        for component in self._sourceCodeGenerator.components_dict.values():
            dest_path = self._splash_dir + '/root_factory/'
            with open(dest_path+'/__init__.py', 'w') as f:
                f.write('')
            if 'group' in component and component['group']:
                dest_path += '/' + self._sourceCodeGenerator.factories_dict[component['group']]['path'] + '/' + component['group']
            dest_path += '/' + component['key']
            self._sourceCodeGenerator.components_dict[component['key']]['path'] = dest_path
            ports_path = dest_path + '/input_ports'
            build_script_path = dest_path + '/' + component['key'] + '.py'
            os.makedirs(dest_path)
            with open(dest_path+'/__init__.py', 'w') as f:
                f.write('')
            with open(build_script_path, 'w') as f:
                f.write(component['build_script'])
            if component['category'] == 'fusionOperator':
                continue
            for port in component['ports']:
                if port['port_type'] == 'STREAM_INPUT_PORT':
                    if not os.path.isdir(ports_path):
                        os.makedirs(ports_path)
                        with open(ports_path+'/__init__.py', 'w') as f:
                            f.write('')
                    callback_script_path = ports_path + '/' + port['Channel'] + '.py'
                    with open(callback_script_path, 'w') as f:
                        f.write(port['callback_script'])
        
    def _locate_build_units(self):
        build_unit_dir = os.path.join(self._splash_dir, "build_units")
        os.makedirs(build_unit_dir)
        with open(build_unit_dir+'/__init__.py', 'w') as f:
            f.write('')
        for build_unit in self._sourceCodeGenerator.build_units_dict.values():
            build_unit_name = build_unit['key']
            build_unit_path = build_unit_dir + '/' + build_unit_name + '.py'
            with open(build_unit_path, 'w') as f:
                f.write(build_unit['assemble_script'])
            main_scripts = ''
            main_scripts = append_lines(main_scripts, f'from .splash.build_units import {build_unit_name}', 0)
            main_scripts = append_lines(main_scripts, 'def main():', 0)
            main_scripts = append_lines(main_scripts, f'{build_unit_name}.run()', 1)
            with open(self._node_dir + '/' + build_unit_name + '_exec.py', 'w') as f:
                f.write(main_scripts)
