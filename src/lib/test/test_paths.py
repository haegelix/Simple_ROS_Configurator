import json
import os
from unittest import TestCase
from src.lib.lib import paths


class TestPaths(TestCase):
    def setUp(self):
        os.makedirs("/tmp/test/src/test/", exist_ok=True)
        self.config = "{\"wspath\": \"/tmp/test/\"}"
        self.paths = paths.Paths(json.loads(self.config))

    def test_init(self):
        self.assertEqual(self.paths.ros_ws, "/tmp/test", "ROS WS wrong!")
        self.assertEqual(self.paths.ros_ws_src, "/tmp/test/src", "ROS WS source dir wrong!")
        self.assertEqual(self.paths.srosc, os.getcwd(), "Simple ROS WD is wrong")
        self.assertEqual(self.paths.srosc_configs, os.getcwd() + "/configs", "Simple ROS WD is wrong")

    def test_switch_to_configurator_dir(self):
        os.chdir("/")
        self.paths.switch_to_configurator_dir()
        self.assertEqual(os.getcwd(), self.paths.srosc, "Switching to SimpleROSConfigurator-Dir failed!")

    def test_switch_to_package_dir(self):
        os.chdir("/")
        self.paths.switch_to_package_dir("test")
        self.assertEqual(os.getcwd(), self.paths.ros_ws_src + "/test/", "Switching to Package Dir failed!")

    def test_switch_to_ws_source_dir(self):
        os.chdir("/")
        self.paths.switch_to_ws_source_dir()
        self.assertEqual(os.getcwd(), self.paths.ros_ws_src, "Switching to SimpleROSConfigurator-Dir failed!")

    def test_switch_to_ws_dir(self):
        os.chdir("/")
        self.paths.switch_to_ws_dir()
        self.assertEqual(os.getcwd(), self.paths.ros_ws, "Switching to SimpleROSConfigurator-Dir failed!")

    def tearDown(self) -> None:
        del self.paths
        del self.config
