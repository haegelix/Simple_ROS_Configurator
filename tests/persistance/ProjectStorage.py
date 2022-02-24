import unittest
from ros2_ui.interfaces.ProjectStorage import ProjectStorage


class TestCaseProjectStorage(unittest.TestCase):
    def setUp(self) -> None:
        self.projectStorage = ProjectStorage()

    def test_load_all(self):
        print(self.projectStorage.load_all())


if __name__ == '__main__':
    unittest.main()
