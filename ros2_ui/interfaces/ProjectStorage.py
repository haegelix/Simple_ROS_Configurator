import json
from pathlib import Path
from os import path
import os

from ros2_ui.domains.Project import Project
from ros2_ui.settings import settings


class ProjectStorage:
    """
    Used to store or load projects on/from disk.
    """

    def __init__(self):
        """
        Initialize the ProjectStorage.
        """
        self.projects_path = settings.projects_path

    def list(self) -> [str]:
        """
        Get list of all projects stored on disk.
        :return: List of filenames.
        """
        dir_contents = os.listdir(self.projects_path)
        return [i for i in dir_contents if i.endswith(".json")]

    def load_one(self, filename: str) -> Project:
        """
        Loads a project from disk.
        :param filename: Filename of the project.
        :return: The project.
        """
        projectfile_path = path.join(self.projects_path, filename)
        project_file = json.load(open(projectfile_path))
        return Project.from_dict(project_file)

    def save(self, project: Project):
        """
        Save a project to disk.
        :param project: Project to be saved.
        :return: Nothing.
        """
        # retrieve filename (PACKAGE_NAME.json) & assemble path
        filename = project.project_info.package_name + ".json"
        projectfile_path = path.join(self.projects_path, filename)

        # dump project to json
        js = json.dumps(Project.to_dict(project), indent=2)

        # write and close file, overwrite of exists
        out_file = open(projectfile_path, "w")
        out_file.write(js)
        out_file.close()
