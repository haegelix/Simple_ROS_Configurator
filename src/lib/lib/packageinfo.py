import json
from src.lib.lib.paths import paths


class PackageInfo(object):
    class __PackageInfo(object):
        """
        __PackageInfo stores meta information about the package
        """
        version: str  # Version of the package
        description: str  # Description of the package
        maintainer_mail: str  # Email address of the packages maintainer
        maintainer: str  # Name of the packages maintainer
        license: str  # Licence of the package
        test_depends: [str]  # Dependencies to be used in testing of the package # TODO implement tests
        exec_depends: [str]  # Dependencies to be used in execution of the package
        exec_depends_str: str  # space-separated storage of above exec_depends

        def __init__(self, json_obj):
            self.version = json_obj["version"]
            self.description = json_obj["description"]
            self.maintainer_mail = json_obj["maintainer_mail"]
            self.maintainer = json_obj["maintainer"]
            self.license = json_obj["license"]
            self.test_depends = json_obj["test_depends"]
            self.exec_depends = json_obj["exec_depends"]
            self.exec_depends_str = ""
            for x in self.exec_depends:
                self.exec_depends_str += x
                if x != self.exec_depends[-1]:  # add spaces BETWEEN the entries
                    self.exec_depends_str += " "

        def __str__(self):
            s = ""
            s += "Version:      " + str(self.version) + "\n"
            s += "Description:  " + str(self.description) + "\n"
            s += "Maintainer:   " + str(self.maintainer) + " (" + str(self.maintainer_mail) + ")\n"
            s += "License:      " + str(self.license) + "\n"
            s += "Test deps:    " + str(self.test_depends) + "\n"  # TODO deprecated
            s += "Dependencies: " + str(self.exec_depends)
            return s

    class __AdditionalImport(object):
        from_: str
        modules: [str]

        def __init__(self, json_obj):
            self.from_ = json_obj["from_"]
            self.modules = json_obj["modules"]

        def __str__(self):
            s = ""
            s += str(self.from_) + "|"
            s += str(self.modules)
            return s

    class __Pub(object):
        node_name: str
        topic: str
        type: str
        src: str

        def __init__(self, json_obj):
            self.node_name = json_obj["node_name"]
            self.topic = json_obj["topic"]
            self.type = json_obj["type"]
            self.src = json_obj["src"]

        def __str__(self):
            s = ""
            s += str(self.node_name) + "|"
            s += str(self.topic) + "|"
            s += str(self.type) + "|"
            s += str(self.src)
            return s

    class __Sub(object):
        node_name: str
        topic: str
        type: str
        callback: any

        def __init__(self, json_obj):
            self.node_name = json_obj["node_name"]
            self.topic = json_obj["topic"]
            self.type = json_obj["type"]
            self.callback = json_obj["callback"]

        def __str__(self):
            s = ""
            s += str(self.node_name) + "|"
            s += str(self.topic) + "|"
            s += str(self.type) + "|"
            s += str(self.callback)
            return s

    package_name: str
    package_info: __PackageInfo
    pubs: [__Pub]
    subs: [__Sub]
    additional_imports: [__AdditionalImport]

    def __init__(self, json_obj):
        self.package_name = json_obj["package_name"]
        self.package_info = self.__PackageInfo(json_obj["package_info"])
        self.subs = [self.__Sub(j) for j in json_obj["subs"]]
        self.pubs = [self.__Pub(j) for j in json_obj["pubs"]]
        self.additional_imports = [self.__AdditionalImport(j) for j in json_obj["additional_imports"]]

    def __str__(self):
        s = "### Package ###\n"
        s += "Package name: " + str(self.package_name) + "\n"
        s += "Package info: " + "\n|    " + str(self.package_info).replace("\n", "\n|    ") + "\n"
        s += "Subscribers:  " + str([str(j) for j in self.subs]) + "\n"
        s += "Publishers:   " + str([str(j) for j in self.pubs]) + "\n"
        s += "Addi imports: " + str([str(j) for j in self.additional_imports])  # TODO deprecated
        return s


def load_package_config_from_file(filename) -> PackageInfo:
    """
    Load a package config file and store all info in instances of above helper-classes
    :param filename: config file to be loaded
    """
    in_file = open(paths.get_package_config_path(filename))
    json_obj = json.load(in_file)
    in_file.close()
    return PackageInfo(json_obj)


def load_package_config_from_string(json_string: str) -> PackageInfo:
    """
    Load a package config file and store all info in instances of above helper-classes
    :param json_string: config string to be loaded
    """
    return PackageInfo(json.loads(json_string))
