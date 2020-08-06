def append_lines(string, lines, indent):
    line_list = lines.split('\n')
    for line in line_list:
        string = string + (" " * indent * 4) + line + "\n"
    return string


def relate_stream_ports(component, stream_ports):
    component["stream_input_ports"] = []
    component["stream_output_ports"] = []
    new_stream_port = []
    for stream_port in stream_ports:
        if(stream_port["group"] == component["key"]):
            if(stream_port["PORT_TYPE"] == "STREAM_INPUT_PORT"):
                stream_port = __simplify_port(stream_port)
                component["stream_input_ports"].append(stream_port)
            elif(stream_port["PORT_TYPE"] == "STREAM_OUTPUT_PORT"):
                stream_port = __simplify_port(stream_port)
                component["stream_output_ports"].append(stream_port)
        else:
            new_stream_port.append(stream_port)
    return new_stream_port


def relate_event_input_ports(component, event_input_ports):
    component["event_input_ports"] = []
    new_event_port = []
    for event_port in event_input_ports:
        if(event_port["group"] == component["key"]):
            event_port = __simplify_port(event_port)
            component["event_input_ports"].append(event_port)
        else:
            new_event_port.append(event_port)
    return new_event_port


def relate_event_output_ports(component, event_output_ports):
    component["event_output_ports"] = []
    new_event_port = []
    for event_port in event_output_ports:
        if(event_port["group"] == component["key"]):
            event_port = __simplify_port(event_port)
            component["event_output_ports"].append(event_port)
        else:
            new_event_port.append(event_port)
    return new_event_port


def relate_modechange_output_ports(component, modechange_output_ports):
    component["modechange_output_ports"] = []
    new_modechange_port = []
    for modechange_port in modechange_output_ports:
        if(modechange_port["group"] == component["key"]):
            modechange_port = __simplify_port(modechange_port)
            component["modechange_output_ports"].append(modechange_port)
        else:
            new_modechange_port.append(modechange_port)
    return new_modechange_port


def __simplify_port(port):
    try:
        del port['category']
        del port['loc']
        del port['buildUnit']
    except KeyError:
        pass

    return port
