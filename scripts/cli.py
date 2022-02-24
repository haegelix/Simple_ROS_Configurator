"""
This file is currently not included in installation due to refactoring the CLI.
"""
from ros2_ui.use_cases.Project import *
from ros2_ui.interfaces.ProjectStorage import ProjectStorage


storage = ProjectStorage()
p = project_load_one_use_case(storage, "test_package.json")
project_create_ros2_package_use_case(p)
project_build_use_case(p)
