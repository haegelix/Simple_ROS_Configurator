class MissingDescriptionError(Exception):
    pass


class MyPrettyPrint(object):
    """
    Pretty strings on str and repr.
    """
    DESCRIPTION = {}

    def prettyprint(self) -> str:
        s = ""
        for key, value in self.__dict__.items():
            if key in self.DESCRIPTION:
                s += self.DESCRIPTION[key] + " (" + key + "): " + str(value) + "\n"
            else:
                continue
                # s += key + ": " + str(value) + "\n"
                # raise MissingDescriptionError("Missing description for variable " + key)
        if len(s) > 0:
            return s[:-1]
        else:
            return "Nothing here to be pretty-printed"

    def __str__(self):
        return self.prettyprint()

    def __repr__(self):
        return self.prettyprint()
