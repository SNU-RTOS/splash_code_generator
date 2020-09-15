import subprocess


class ROSPackageGenerator:
    def __init__(self, name, path):
        command = "ros2 pkg create --build-type ament_python %s --dependencies rclpy scl" % (name)

        process = subprocess.Popen(command.split(), cwd=path + "/src",
                                   stdin=subprocess.PIPE, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
        out, err = process.communicate()
        if(out):
            print(out.decode('utf-8').strip())
        elif(err):
            print(err.decode('utf-8').strip())
