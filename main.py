import argparse
from splash_code_generator.core import Core


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--name', required=True, help='Project name')
    parser.add_argument('-f', '--file', required=True, help='JSON file')
    parser.add_argument('-p', '--path', required=True,
                        help="Your ROS workspace path")
    parser.add_argument('-u', '--update', action='store_true', help="Update code")
    parser.add_argument('--username', help="User name(from Splash Hub)")
    parser.add_argument('--email', help="User email(from Splash Hub)")
    args = parser.parse_args()
    code_generator = Core(args)
    code_generator.generate()


if __name__ == '__main__':
    main()
