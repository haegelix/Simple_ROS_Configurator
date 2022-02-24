import dataclasses
from dataclass_dict_convert import dataclass_dict_convert


@dataclass_dict_convert
@dataclasses.dataclass
class Node:
    node_name: str
    topic: str
    msg_type: str
    uses_stdout: bool
    needs_tty: bool


@dataclass_dict_convert
@dataclasses.dataclass
class Publisher(Node):
    src: str


@dataclass_dict_convert
@dataclasses.dataclass
class Subscriber(Node):
    callback: str
    user_code: str

