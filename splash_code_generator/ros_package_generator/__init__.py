import subprocess


class ROSPackageGenerator:
    def __init__(self, name, path):
        command = "ros2 pkg create --build-type ament_python %s" % (name)
        subprocess.Popen(command.split(),
                         cwd=path + "/src", stdin=subprocess.PIPE, stderr=subprocess.PIPE)
