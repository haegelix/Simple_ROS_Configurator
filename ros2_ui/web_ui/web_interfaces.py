import logging

from flask_socketio import SocketIO


class SocketIOHandler(logging.Handler):
    """A logging handler that emits records with SocketIO."""

    def __init__(self, socket: SocketIO, room: str, topic=str):
        """
        Constructor.
        :param socket: Socket to be used.
        :param room: Room to have the messages posted to.
        :param topic: Topic to be posting to.
        """
        super().__init__()
        self.socket = socket
        self.room = room
        self.topic = topic

    def emit(self, record):
        """
        Emits a log record.
        :param record:
        :return:
        """
        module = record.__dict__["module"]
        msg = self.format(record)
        if module == "_internal":
            return
        self.socket.emit(self.topic, msg, to=self.room, namespace="/package")
