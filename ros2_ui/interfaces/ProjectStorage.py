import json
from pathlib import Path
from os import path
import os

from ros2_ui.domains.Project import Project
from ros2_ui.settings import settings


# WARNING: Package name "TestPackage" does not follow the naming conventions. It should start with a lower case
# letter and only contain lower case letters, digits, underscores, and dashes.

class ProjectStorage:
    def __init__(self):
        self.projects_path = settings.projects_path

    def list(self) -> [str]:
        l = os.listdir(self.projects_path)
        return [i for i in l if i.endswith(".json")]

    def load_all(self) -> [Project]:
        projectfiles_paths = [path.join(self.projects_path, j) for j in self.list() if
                              j.endswith(".json")]
        project_files = [json.load(open(i)) for i in projectfiles_paths]
        return [Project.from_dict(i) for i in project_files]

    def load_one(self, filename: str) -> Project:
        """
        Loads a project from disk.
        :param filename: Filename of the project.
        :return: The project
        """
        projectfile_path = path.join(self.projects_path, filename)
        project_file = json.load(open(projectfile_path))
        return Project.from_dict(project_file)

    def list_projects(self) -> [str]:
        return [j for j in os.listdir(self.projects_path) if j.endswith(".json")]

    def save(self, project: Project):
        filename = project.project_info.package_name + ".json"
        projectfile_path = path.join(self.projects_path, filename)
        js = json.dumps(Project.to_dict(project), indent=2)
        out_file = open(projectfile_path, "w")
        out_file.write(js)
        out_file.close()
