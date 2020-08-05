def append_lines(string, lines, indent):
    line_list = lines.split('\n')
    for line in line_list:
        string = string + (" " * indent * 4) + line + "\n"
    return string
