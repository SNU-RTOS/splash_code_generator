import re

from ._util import *
import os, json


class SourceCodeGenerator:
    def __init__(self, username, pkgname, json):
        self._username = username
        self._pkgname = pkgname

        self.node_data_list = []
        self.link_data_list = []
        self.factories_dict = {}
        self.components_dict = {}
        self.ports_dict = {}
        self.build_units_dict = {}
        self.forest = None
        try:
            self.node_data_list = json["nodeDataArray"] 
        except:
            pass
        try:
            self.link_data_list = json["linkDataArray"]
        except:
            pass

        self.node_data_list.append({'key': 'default_build_unit', 'category': 'buildUnit', 'UUID': 'splash-default-build-unit'})
        for node in self.node_data_list:
            if (node['category'] == 'fusionOperator' or 'Component' in node['category']) and node['buildUnit'] == '':
                node['buildUnit'] = 'default_build_unit'
            if 'Port' in node['category']:
                node['MessagePkg'] = 'std_msgs'
                node['MessageType'] = 'String'

    def generate(self):
        self._make_dicts()
        # for factory in self.factories_dict.values():
            # print(factory)
        self._make_structure_tree()
        for port in self.ports_dict.values():
            try:
                if port['category'] == 'modeChangeOutputPort':
                    self._find_factory_for_modechange_port(port)
                self.components_dict[port['group']]['ports'].append(port)
                
                if 'port_type' in port and port['port_type'] == 'STREAM_INPUT_PORT':
                    self._make_callback_script(port)
                    self._find_channel_for_input_port(port)
                elif port['category'] == 'eventInputPort':
                    self._make_callback_script(port)
                    self._find_event_for_event_input_port(port)

            except Exception as e:
                self.factories_dict[port['group']]['ports'].append(port)
        for component in self.components_dict.values():
            self.build_units_dict[component['buildUnit']]['components'].append(component)
            self._make_build_script(component)

        for build_unit in self.build_units_dict.values():
            self._make_assemble_script(build_unit)

    def _make_dicts(self):
        for node in self.node_data_list:
            if node['category'] == 'fusionOperator' or 'Component' in node['category']:
                node['ports'] = []
                self.components_dict[node['key']] = node
            elif 'Port' in node['category']:
                self.ports_dict[node['key']] = node
            elif node['category'] == 'factory':
                node['ports'] = []
                self.factories_dict[node['key']] = node
            elif node['category'] == 'buildUnit':
                node['components'] = []
                self.build_units_dict[node['key']] = node

    def _make_structure_tree(self):
        def build_tree(raw_nodes, parent_name):
            nodes = {}
            for i in raw_nodes:
                id, obj = (i['key'], i)
                nodes[id] = obj

            forest = []
            for i in raw_nodes:
                id, parent_id, obj = (i['key'], i[parent_name], i)
                node = nodes[id]

                if parent_id == -1:
                    forest.append(node)
                else:
                    parent = nodes[parent_id]
                    if not 'children' in parent:
                        parent['children'] = []
                    parent['children'].append(node)

            return forest

        def find_leap(tree, path):
            if tree['group'] != -1:
                path += '/' + tree['group']
            if 'mode' in tree and tree['mode']:
                path = path + '/' + tree['mode']
            self.factories_dict[tree['key']]['path'] = path
            if 'children' not in tree:
                return tree, path
            for child in tree['children']:
                return find_leap(child, path)

        for factory in self.factories_dict.values():
            if 'group' not in factory or factory['group'] == '':
                factory['group'] = -1

        self.forest = build_tree(self.factories_dict.values(), 'group')
        for tree in self.forest:
            leap, path = find_leap(tree, '')
            leap_path =  path + '/' + leap['key']
            tree['leap_path'] = leap_path

    def _make_callback_script(self, port):
        
        scripts = ''
        if port['category'] == 'eventInputPort':
            scripts = append_lines(scripts, 'def callback(component, event): # Note: Never Edit or Delete this line', 0)
        else:
            scripts = append_lines(scripts, 'def callback(component, msg): # Note: Never Edit or Delete this line', 0)
        scripts = append_lines(scripts, '# TODO: write your algorithm', 1)
        scripts = append_lines(scripts, 'pass', 1)

        port['callback_script'] = scripts

    def _make_build_script(self, component):
        scripts = ''
        classes_dict = {"processingComponent": 'ProcessingComponent', "sourceComponent": 'SourceComponent', "sinkComponent": 'SinkComponent', 'fusionOperator': 'FusionOperator'}
        component_class = classes_dict[component['category']]
        component_name = component['key']
        factory = None
        if 'group' in component:
            factory = component['group']
            factory = f'\"{factory}\"'
        scripts = append_lines(scripts, f'from scl.components import {component_class}', 0)
        # scripts = append_lines(scripts, 'from scl.ports import StreamInputPort, StreamOutputPort', 0)
        messageTypes = []
        import_msg_scripts = ''
        import_callback_scripts = ''
        for port in component['ports']:
            if 'stream' in port['category']:
                messagePkg = port['MessagePkg']
                messageType = port['MessageType']
                channel = port['Channel']
                if messageType not in messageTypes:
                    import_msg_scripts = append_lines(import_msg_scripts, f'from {messagePkg}.msg import {messageType}', 0)
                    messageTypes.append(messageType)
                if port['port_type'] == 'STREAM_INPUT_PORT' and component['category'] != 'fusionOperator':
                    import_callback_scripts = append_lines(import_callback_scripts, f'from .input_ports import {channel}', 0)
        scripts = append_lines(scripts, import_msg_scripts, 0)
        scripts = append_lines(scripts, import_callback_scripts, 0)
        scripts = append_lines(scripts, 'def build():', 0)
        scripts = append_lines(scripts, f'component = {component_class}(\"{component_name}\", {factory})', 1)
        for port in component['ports']:
            port_name = port['key']
            if 'stream' in port['category']:
                port_channel = port['Channel']
                port_type = port['MessageType']
                if port['port_type'] == 'STREAM_INPUT_PORT':
                    if component['category'] == 'fusionOperator':
                        scripts = append_lines(scripts, f'component.create_and_attach_stream_input_port(\"{port_name}\", {port_type}, \"{port_channel}\")', 1)
                    else:
                        scripts = append_lines(scripts, f'component.create_and_attach_stream_input_port(\"{port_name}\", {port_type}, \"{port_channel}\", {port_channel}.callback)', 1)
                elif port['port_type'] == 'STREAM_OUTPUT_PORT':
                    port_rate = 'None'
                    if 'Rate' in port and port['Rate'] > 0:
                        port_rate = port['Rate']
                    if(component['category'] == 'fusionOperator'):
                        scripts = append_lines(scripts, f'component.create_and_attach_stream_output_port(\"{port_name}\", \"{port_channel}\", {port_rate})', 1)
                    else:
                        scripts = append_lines(scripts, f'component.create_and_attach_stream_output_port(\"{port_name}\", {port_type}, \"{port_channel}\", {port_rate})', 1)

            elif port['category'] == 'modeChangeOutputPort':
                target_factory = port['target_factory']
                scripts = append_lines(scripts, f'component.create_and_attach_mode_change_port(\"{port_name}\", \"{target_factory}\")', 1)
            elif port['category'] == 'eventInputPort':
                event = port['Event']
                scripts = append_lines(scripts, f'component.create_and_attach_event_input_port(\"{port_name}\", \"{event}\", {event}.callback)', 1)
            elif port['category'] == 'eventOutputPort':
                event = port['Event']
                scripts = append_lines(scripts, f'component.create_and_attach_event_output_port(\"{port_name}\", \"{event}\")', 1)
        scripts = append_lines(scripts, 'return component', 1)
        component['build_script'] = scripts
    
    def _make_assemble_script(self, build_unit):
        scripts = ''
        scripts = append_lines(scripts, 'from scl.build_unit import BuildUnit', 0)
        build_unit_key = build_unit['key']
        import_component_script = ''
        component_build_script = ''
        modechange_configuration_script = ''
        factory_set = [] 
        for component in build_unit['components']:
            key = component['key']
            path = ''
            if 'group' in component and component['group'] in self.factories_dict and 'path' in self.factories_dict[component['group']]:
                path = self.factories_dict[component['group']]['path'].replace('/', '.') + '.' + self.factories_dict[component['group']]['key']
                factory_set.append(component['group'])
            import_component_script = append_lines(import_component_script, f'from ..root_factory{path}.{key} import {key}', 0)
            component_build_script = append_lines(component_build_script, f'build_unit.add({key}.build())', 0)
        factory_set = set(factory_set)
        
        for factory in factory_set:
            configuration = self.factories_dict[factory]["mode_configuration"]
            for mode in configuration["mode_list"]:
                mode['components'] = []
                for component in build_unit['components']:
                    if 'group' in component and component['group'] == factory:
                        if component['mode'] == mode['name']:
                            mode['components'].append(component['key'])
            modechange_configuration_script = append_lines(modechange_configuration_script, f'build_unit.configure_modechange(\"{factory}\", {configuration})', 0)
        scripts = append_lines(scripts, import_component_script, 0)
        scripts = append_lines(scripts, 'def run():', 0)
        scripts = append_lines(scripts, f'build_unit = BuildUnit(\"{build_unit_key}\")', 1)
        scripts = append_lines(scripts, component_build_script, 1)
        scripts = append_lines(scripts, modechange_configuration_script, 1)
        scripts = append_lines(scripts, 'build_unit.run()', 1)
        build_unit['assemble_script'] = scripts

    def _find_channel_for_input_port(self, input_port):
        for link in self.link_data_list:
            if link["to"] == input_port["key"]:
                for node in self.node_data_list:
                    if node['category'] == 'streamPort' and (node["port_type"] == "STREAM_OUTPUT_PORT" and node["key"] == link["from"]):
                        input_port['Channel'] = node['Channel']
                        input_port['MessagePkg'] = node['MessagePkg']
                        input_port['MessageType'] = node['MessageType']

    def _find_event_for_event_input_port(self, input_port):
        for link in self.link_data_list:
            if link["to"] == input_port["key"]:
                for node in self.node_data_list:
                    if node['category'] == 'eventInputPort' and node["key"] == link["from"]:
                        input_port['Event'] = node['Event']

    def _find_children_for_node(self, parent):
        children = []
        for node in self.node_data_list:
            if 'group' in node and node['group'] == parent['key']:
                children.append(node['UUID'])
        return children
    
    def _find_components_for_build_unit(self, build_unit):
        components = []
        for node in self.node_data_list:
            if ('Component' in node['category'] or node['category'] == 'fusionOperator') and node['buildUnit'] == build_unit['key']:
                components.append(node['UUID'])
        return components
    
    def _find_input_channels_for_component(self, component):
        input_channels = []
        
        return input_channels
    
    def _find_output_channels_for_component(self, component):
        output_channels = []
        return output_channels
    
    def _find_factory_for_modechange_port(self, port):
        for link in self.link_data_list:
            if link["from"] == port["key"]:
                for node in self.node_data_list:
                    if node['category'] == 'modeChangeInputPort' and node['key'] == link['to']:
                        port["target_factory"] = node['group']