from ros2_ui.domains.Node import Node, Publisher, Subscriber


class ROS2_Node:
    node: Node

    def __init__(self, node: Node):
        self.node = node


class ROS2_Publisher(ROS2_Node):
    node: Publisher

    def __init__(self, node: Publisher):
        super(ROS2_Publisher, self).__init__(node)

    def get_python_snippet(self):
        return 'p = _Pub("' + self.node.node_name + '", ' + self.node.msg_type + ', "' + self.node.topic + '")'


# TODO make functional
class ROS2_Subscriber(ROS2_Node):
    node: Subscriber

    def __init__(self, node: Subscriber):
        super(ROS2_Subscriber, self).__init__(node)

    def get_python_sub_snippet(self):
        cb_ins = "cb_module.callback"
        return 'sub = _Sub("' + self.node.node_name + '", ' + self.node.msg_type + ', "' \
               + self.node.topic + '", ' + cb_ins + ')'

    def get_python_import_snippet(self):
        cb_import = self.node.node_name + "_sub_runner"
        return 'import ' + cb_import + ' as cb_module'

    def get_filename(self):
        return self.node.node_name + "_sub"

    def get_runner_filename(self):
        return self.node.node_name + "_sub_runner"
