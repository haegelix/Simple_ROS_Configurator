from ros2_ui.domains.Node import Node
from ros2_ui.domains.Project import Project


class LaunchfileNode:
    def __init__(self, project: Project, node: Node, executable: str):
        self.project = project
        self.node = node
        self.executable = executable

    def to_python(self) -> str:
        s = ""
        s += " " * 8 + "launch_ros.actions.Node(" + "\n"
        s += " " * 12 + "package = '" + self.project.project_info.package_name + "'," + "\n"
        s += " " * 12 + "executable = '" + self.executable + "',\n"
        if self.node.uses_stdout:
            s += " " * 12 + "output='screen'," + "\n"
        if self.node.needs_tty:
            s += " " * 12 + "emulate_tty=True," + "\n"
        s += " " * 8 + "),"
        return s
