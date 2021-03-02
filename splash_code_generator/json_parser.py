import json

class JsonParser:
    def __init__(self, file_name):
        self._file = file_name

    def parse(self):
        with open(self._file, "r") as st_json:
            return json.load(st_json)
