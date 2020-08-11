from ._util import *
import copy


class FactoryGenerator:
    def __init__(self, factories):
        self._factories = copy.deepcopy(factories)

    def generate(self):
        factories = []

        for factory in self._factories:
            factories.append(self._generate_factory(factory))
        return factories

    def _generate_factory(self, factory):
        _factory = {}
        _factory["name"] = factory["name"]
        _factory["class_name"] = factory["class_name"]
        _factory["mode_configuration"] = factory["mode_configuration"]
        _factory["parent"] = factory["group"] if "group" in factory.keys() else None
        _factory["source_code"] = self._generate_source_code(_factory)

        return _factory

    def _generate_source_code(self, factory):
        _str = ""
        _str = append_lines(_str, "'''", 0)
        _str = append_lines(
            _str, "Generated automatically by Splash Code Generator for {}".format(factory["name"]), 1)
        _str = append_lines(_str, "'''", 0)
        _str = append_lines(_str, self._import_scl(), 0)
        _str = append_lines(_str, self._generate_factory_class(factory), 0)
        return _str

    def _import_scl(self):
        _str = ""
        _str = append_lines(_str, "from scl.factory import Factory", 0)
        return _str

    def _generate_factory_class(self, factory):
        _str = ""
        _str = append_lines(
            _str, "class {}(Factory):".format(factory["class_name"]), 0)
        _str = append_lines(_str, "def __init__(self):", 1)
        _str = append_lines(
            _str, "super().__init__(name=\"{}\", parent={}, mode_configuration={})".format(factory["name"], factory["parent"], factory["mode_configuration"]), 2)
        return _str
