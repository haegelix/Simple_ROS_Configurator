from ros2_ui.interfaces.ProjectStorage import ProjectStorage
from ros2_ui.interfaces.ros_interface_classes.entrypoint import EntryPoint
from ros2_ui.interfaces.ros_interface_classes.launchfile_node import LaunchfileNode
from ros2_ui.interfaces.template_files import copy_edit_template_file, register_entry_point, register_launch_file, \
    write_launch_file
from ros2_ui.domains.Project import Project
import ros2_ui.interfaces.ros2_cli as ros2_cli
from ros2_ui.interfaces.Log import logging


def project_get_list_use_case(project_storage: ProjectStorage) -> [Project]:
    return project_storage.list()


def project_load_one_use_case(project_storage: ProjectStorage, filename: str) -> [Project]:
    return project_storage.load_one(filename)


def project_save_use_case(project_storage: ProjectStorage, project: Project):
    return project_storage.save(project)


def check_ros2_workspace_exists_use_case(logger=logging):
    if not ros2_cli.workspace_exists():
        logger.info("Missing ROS2 workspace...")
        ros2_cli.create_workspace()
        logger.info("Missing ROS2 workspace... DONE!")


def project_create_ros2_package_use_case(project: Project, logger=logging):
    logger.info("Creating project '" + project.project_info.package_name + "'...")

    # check if ros2 workspace does exist
    check_ros2_workspace_exists_use_case(logger)

    # create ROS2 package
    ros2_cli.create_package(project, logger)

    # copy all project files to the ros2 workspace
    # and get all the entry points and launch file nodes along the way
    # todo split the two steps
    entrypoints: [EntryPoint]
    entrypoints = []
    launch_file_nodes: [LaunchfileNode]
    launch_file_nodes = []
    for n in project.pubs:
        e, l = copy_edit_template_file(n, project)
        entrypoints.append(e)
        launch_file_nodes.append(l)
    for n in project.subs:
        e, l = copy_edit_template_file(n, project)
        entrypoints.append(e)
        launch_file_nodes.append(l)

    # register entry points
    for e in entrypoints:
        register_entry_point(project, e)

    # write launch file
    write_launch_file(project, launch_file_nodes)

    # register launch file
    register_launch_file(project)

    # done
    logger.info("Creating project '" + project.project_info.package_name + "'... DONE!")


def project_build_use_case(project: Project, logger=logging):
    logger.info("Resolving and installing dependencies...")
    ros2_cli.resolve_dep(project)
    logger.info("Resolving and installing dependencies... DONE!")
    logger.info("Building the new package...")
    ros2_cli.build_package(project)
    logger.info("Building the new package... DONE!")

