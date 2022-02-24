import ros2_ui.template_files
from os import path

from ros2_ui.domain.Project import Project
from ros2_ui.domain.Node import Node, Publisher, Subscriber
from ros2_ui.interfaces.ros2_cli import get_package_py_dir, get_package_root_dir
from ros2_ui.interfaces.ros_interface_classes.launchfile_node import LaunchfileNode
from ros2_ui.interfaces.ros_interface_classes.ros_node import ROS2_Publisher, ROS2_Subscriber
from ros2_ui.settings import settings
from ros2_ui.interfaces.ros_interface_classes.entrypoint import EntryPoint
from ros2_ui.interfaces.Log import logging

template_dir = path.dirname(ros2_ui.template_files.__file__)


class repl:
    old: str
    new: str

    def __init__(self, old: str, new: str):
        self.old = old
        self.new = new


def replace_std():
    return [
        repl("!VERSION!", settings.version),
        repl("!SRC_LINK!", settings.src_link)
    ]


def modify_and_write_python_file_from_string(project: Project, py_code: str, dest_name: str, replacements: [repl]):
    # insert code-snippet
    for r in replacements:
        py_code = py_code.replace(r.old, r.new)

    # write file
    dest_path = path.join(get_package_py_dir(project), dest_name + ".py")
    out_file = open(dest_path, "w")
    out_file.write(py_code)
    out_file.close()


def modify_and_copy_python_file(project: Project, templ_name: str, dest_name: str, replacements: [repl]):
    # read template file
    in_file = open(path.join(template_dir, templ_name), "r")
    py_code = in_file.read()
    in_file.close()

    # insert code-snippet
    for r in replacements:
        py_code = py_code.replace(r.old, r.new)

    # write file
    dest_path = path.join(get_package_py_dir(project), dest_name + ".py")
    out_file = open(dest_path, "w")
    out_file.write(py_code)
    out_file.close()


def copy_edit_template_file(node: Node, project: Project) -> {(None, None), (EntryPoint, LaunchfileNode)}:
    replace = replace_std()
    # handle publisher nodes
    if isinstance(node, Publisher):
        n = ROS2_Publisher(node)
        replace.append(repl("# !INSERT_PUBLISHER_DECLARATION_HERE!", n.get_python_snippet()))
        modify_and_copy_python_file(project, "pub.py", node.node_name + "_pub", replace)

        # add runner-scripts
        replace = replace_std()
        replace.append(repl("import pub as pub", ""))
        replace.append(repl("# !INSERT_PUB_IMPORT_HERE!", "import " + node.node_name + "_pub as pub"))
        filename = ""
        if node.src == "-stdin-":
            filename = node.node_name + "_runner_stdin"
            modify_and_copy_python_file(project, "pub_runner_stdin.py", filename, replace)
        elif node.src == "-button-":
            filename = node.node_name + "_runner_button"
            modify_and_copy_python_file(project, "pub_runner_button.py", filename, replace)
        else:
            print("Unknown Type of Entry Point!")
            return None, None
        entry_point = EntryPoint(filename, project.project_info.package_name, filename)
        launch_node = LaunchfileNode(project, node, filename)
        return entry_point, launch_node
    # handle subscriber nodes
    elif isinstance(node, Subscriber):
        n = ROS2_Subscriber(node)
        logging.info("callback is " + n.node.callback)
        replace.append(repl("# !INSERT_SUBSCRIBER_DECLARATION_HERE!", n.get_python_sub_snippet()))
        if node.callback == "-stdout-":
            replace.append(repl("# !INSERT_SUBSCRIBER_IMPORT_HERE!", n.get_python_import_snippet()))
            modify_and_copy_python_file(project, "sub_stdout.py", n.get_runner_filename(), [])
        elif node.callback == "-led-":
            replace.append(repl("# !INSERT_SUBSCRIBER_IMPORT_HERE!", n.get_python_import_snippet()))
            modify_and_copy_python_file(project, "sub_led.py", n.get_runner_filename(), [])
        elif node.callback == "-user-supplied-":
            replace.append(repl("# !INSERT_SUBSCRIBER_IMPORT_HERE!", n.get_python_import_snippet()))
            modify_and_write_python_file_from_string(project, n.node.user_code, n.get_runner_filename(), [])
        else:
            print("Unknown Type of Entry Point!")
            return None, None
        modify_and_copy_python_file(project, "sub.py", node.node_name + "_sub", replace)
        filename = n.get_filename()
        entry_point = EntryPoint(filename, project.project_info.package_name, filename)
        launch_node = LaunchfileNode(project, node, filename)
        return entry_point, launch_node


def write_launch_file(project: Project, launch_nodes: [LaunchfileNode]):
    replacements = replace_std()

    launch_nodes_str = ""
    for ln in launch_nodes:
        if not ln:
            continue
        launch_nodes_str += ln.to_python() + "\n"

    replacements.append(repl("# !INSERT_LAUNCHFILE_NODES_HERE", launch_nodes_str))
    filename = path.join("..", "launch", project.project_info.package_name + ".launch")
    modify_and_copy_python_file(project, "launch.py", filename, replacements)


def register_entry_point(project: Project, e: EntryPoint):
    if e is None:
        return

    logging.info("registering entryPoint " + e.to_python())

    setup_py_path = path.join(get_package_root_dir(project), "setup.py")

    # read current config
    in_file = open(setup_py_path, "r")
    py_code = in_file.read()
    in_file.close()

    # insert snippet
    py_code = py_code.replace("'console_scripts': [", "'console_scripts': [\n" + e.to_python() + ",")

    out_file = open(setup_py_path, "w")
    out_file.write(py_code)
    out_file.close()


def register_launch_file(project: Project):
    logging.info("registering launch file")

    setup_py_path = path.join(get_package_root_dir(project), "setup.py")

    # read current config
    in_file = open(setup_py_path, "r")
    py_code = in_file.read()
    in_file.close()

    # insert snippets
    py_code = "import os\nfrom glob import glob\n" + py_code
    py_code = py_code.replace("data_files=[", "data_files=[\n" +
                              "(os.path.join('share', package_name), glob('launch/*.launch.py')),")

    out_file = open(setup_py_path, "w")
    out_file.write(py_code)
    out_file.close()
