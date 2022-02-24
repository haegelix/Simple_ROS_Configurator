import unittest
from ros2_ui.domains.Node import Publisher, Subscriber


class TestCaseNode(unittest.TestCase):
    def test_Node_Subscriber(self):
        init_dict = {
            "node_name": "test-node-name",
            "topic": "test-topic",
            "msg_type": "test-type",
            "callback": "test-callback"
        }
        sub = Subscriber.from_dict(init_dict)

        self.assertDictEqual(init_dict, sub.to_dict())

    def test_Node_Publisher(self):
        init_dict = {
            "node_name": "test-node-name",
            "topic": "test-topic",
            "msg_type": "test-type",
            "src": "test-src"
        }
        pub = Publisher.from_dict(init_dict)

        self.assertDictEqual(init_dict, pub.to_dict())


if __name__ == '__main__':
    unittest.main()
