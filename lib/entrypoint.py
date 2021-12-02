class EntryPoint(object):
    name: str
    package_name: str
    filename: str
    method: str

    def __init__(self, name: str, package_name: str, filename: str, method="main"):
        self.name = name
        self.package_name = package_name
        self.filename = filename
        self.method = method

    def to_python(self) -> str:
        # Pattern: ["'" + name + " = " + package_name + "." + filename + ":main'"]
        s = ""
        s += "'"
        s += self.name
        s += " = "
        s += self.package_name
        s += "."
        s += self.filename
        s += ":"
        s += self.method
        s += "'"
        return s

