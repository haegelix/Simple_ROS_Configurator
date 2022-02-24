class EntryPoint(object):
    """
    Models an EntryPoint for ROS2 to run the generated code.
    """
    name: str
    package_name: str
    filename: str
    method: str

    def __init__(self, name: str, package_name: str, filename: str, method="main"):
        """
        Init new EntryPoint
        :param name: Name for this EntryPoint
        :param package_name: Package name containing this EntryPoint
        :param filename: Filename of the python file containing this EntryPoint's target.
        :param method: Method to be called. Default is "main".
        """
        self.name = name
        self.package_name = package_name
        self.filename = filename
        self.method = method

    def to_python(self) -> str:
        """
        Returns str representation according to this scheme: 'name = package_name.filename:method'
        (Single-Quotes are included in the str.)
        :return: python code representing this EntryPoint
        """
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

    def __str__(self):
        return self.to_python()

    def __repr__(self):
        return self.to_python()
