import argparse
from splash_code_generator import CodeGenerator


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', '-n', required=True, help='Project name')
    parser.add_argument('--file', '-f', required=True, help='JSON file')
    parser.add_argument('--path', '-p', required=True,
                        help="Your ROS workspace path")
    args = parser.parse_args()
    code_generator = CodeGenerator(args)
    code_generator.generate()
    code_generator.validate()


if __name__ == '__main__':
    main()
