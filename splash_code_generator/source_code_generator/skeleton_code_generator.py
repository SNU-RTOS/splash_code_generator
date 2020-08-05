class SkeletonCodeGenerator:
    def __init__(self, node_data, link_data_list):
        self.skeletons = []
        self.__processing_components = node_data["processing_components"]
        self.__source_components = node_data["source_components"]
        self.__sink_components = node_data["sink_components"]
        self.__fusion_operators = node_data["fusion_operators"]
        self.__factories = node_data["factories"]
        self.__build_units = node_data["build_units"]
        self.__stream_ports = node_data["stream_ports"]
        self.__event_input_ports = node_data["event_input_ports"]
        self.__event_output_ports = node_data["event_output_ports"]
        self.__modechange_input_ports = node_data["modechange_input_ports"]
        self.__modechange_output_ports = node_data["modechange_output_ports"]
        self.__links = link_data_list

    def generate(self):
        pass
