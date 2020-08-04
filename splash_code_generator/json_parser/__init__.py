import json


class JsonParser:
    def __init__(self, file_name):
        self.__file = file_name

    def parse(self):
        with open(self.__file, "r") as st_json:
            return json.load(st_json)
