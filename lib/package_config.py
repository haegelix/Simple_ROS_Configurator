import json
from inspect import getmembers
from pprint import pprint
from xml.dom import minidom
from lib.paths import Paths
from lib.config import Config


class PackageInfo(object):
    class __PackageConfig(object):
        class __PackageInfo(object):
            version: str
            description: str
            maintainer_mail: str
            maintainer: str
            license: str
            test_depends: [str]

            def __init__(self, json_obj):
                self.version = json_obj["version"]
                self.description = json_obj["description"]
                self.maintainer_mail = json_obj["maintainer_mail"]
                self.maintainer = json_obj["maintainer"]
                self.license = json_obj["license"]
                self.test_depends = json_obj["test_depends"]

            def __str__(self):
                s = ""
                s += str(self.version) + "\n"
                s += str(self.description) + "\n"
                s += str(self.maintainer_mail) + "\n"
                s += str(self.maintainer) + "\n"
                s += str(self.license) + "\n"
                s += str(self.test_depends) + "\n"
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
            topic: str
            type: str
            src: str

            def __init__(self, json_obj):
                self.topic = json_obj["topic"]
                self.type = json_obj["type"]
                self.src = json_obj["src"]

            def __str__(self):
                s = ""
                s += str(self.topic) + "|"
                s += str(self.type) + "|"
                s += str(self.src)
                return s

        class __Sub(object):
            topic: str
            type: str
            callback: str

            def __init__(self, json_obj):
                self.topic = json_obj["topic"]
                self.type = json_obj["type"]
                self.callback = json_obj["callback"]

            def __str__(self):
                s = ""
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
            s = ""
            s += str(self.package_name) + "\n"
            s += str(self.package_info) + "\n"
            s += str([str(j) for j in self.subs]) + "\n"
            s += str([str(j) for j in self.pubs]) + "\n"
            s += str([str(j) for j in self.additional_imports]) + "\n"
            return s

    p: Paths
    c: Config
    pkg_config: __PackageConfig

    def __init__(self, args):
        self.p, self.c = args

    def load_package_config(self, filename):
        json_obj = json.load(open(self.p.get_package_config_path(filename)))
        self.pkg_config = self.__PackageConfig(json_obj)
        print(str(self.pkg_config))

    def load_packagexml(self, package_name):
        pass

    def __str__(self):
        s = ""
        s += str(self.p)
        s += str(self.c)
        s += str(self.pkg_config) + "\n"
        return s
