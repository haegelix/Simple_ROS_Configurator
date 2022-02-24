from os import path
from pathlib import Path

from ros2_ui.__init__ import __version__, source_link


class __Settings:
    version = __version__
    src_link = source_link
    log_path = path.join(Path.home(), ".ros2_ui", "LOG.log")
    projects_path = path.join(Path.home(), ".ros2_ui", "projects")


settings = __Settings()
