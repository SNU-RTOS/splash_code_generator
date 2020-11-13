import argparse
from splash_code_generator import CodeGenerator


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--name', required=True, help='Project name')
    parser.add_argument('-f', '--file', required=True, help='JSON file')
    parser.add_argument('-p', '--path', required=True,
                        help="Your ROS workspace path")
    parser.add_argument('-u', '--update', action='store_true', help="Update code" )
    args = parser.parse_args()
    code_generator = CodeGenerator(args)
    code_generator.generate()
    code_generator.save()


if __name__ == '__main__':
    main()
