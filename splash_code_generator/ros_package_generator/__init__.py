import subprocess


class ROSPackageGenerator:
    def __init__(self, name, path):
        command = "ros2 pkg create --build-type ament_python %s --dependencies rclpy %s_interfaces" % (
            name, name)

        process = subprocess.Popen(command.split(), cwd=path + "\\src",
                                   stdin=subprocess.PIPE, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
        out, err = process.communicate()
        if(out):
            print(out.decode('utf-8').strip())
        elif(err):
            print(err.decode('utf-8').strip())
        command2 = "ros2 pkg create --build-type ament_cmake %s_interfaces" % (
            name)
        process2 = subprocess.Popen(command2.split(), cwd=path + "\\src",
                                    stdin=subprocess.PIPE, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
        out2, err2 = process2.communicate()
        if(out2):
            print(out2.decode('utf-8').strip())
        elif(err2):
            print(err2.decode('utf-8').strip())
