from os import path
from pathlib import Path

from ros2_ui.__init__ import *


class __Settings:
    version = v
    src_link = source_link
    log_path = path.join(Path.home(), ".ros2_ui", "LOG.log")


settings = __Settings()
