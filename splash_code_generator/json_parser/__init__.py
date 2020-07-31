import json


class JsonParser:
    def __init__(self, file_name):
        self.json_decoded = None
        with open(file_name, "r") as st_json:
            self.json_decoded = json.load(st_json)

    def get_json_decoded(self):
        return self.json_decoded
