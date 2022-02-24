#!/usr/bin/python3
# external modules
import json
import os

# flask & websocket modules
from logging.handlers import QueueListener
from queue import SimpleQueue

from flask import Flask, render_template, abort, redirect
from flask_socketio import SocketIO, emit, join_room, close_room

# ros modules
import rclpy
from rclpy.node import Node

# custom ros2_ui modules
from ros2_ui.interfaces.Log import add_logger, del_logger, get_queue_logger
from ros2_ui.settings import settings
from ros2_ui.domain.Project import Project
from ros2_ui.use_cases import Project as ProjectUseCases
from ros2_ui.interfaces import ProjectStorage
from ros2_ui.web_ui.web_interfaces import SocketIOHandler

ps = ProjectStorage.ProjectStorage()

app = Flask(__name__)
ros_node: Node
socket_ = SocketIO(app, async_mode=None)


# WEBSOCKET
@socket_.on('build', namespace='/package')
def build_package(message):
    q = SimpleQueue()
    logger = get_queue_logger(q, "build")
    handler = SocketIOHandler(socket_, "build", "build")
    listener = QueueListener(q, handler)
    listener.start()
    # prepare for redirecting the logs
    join_room("build")
    project = ProjectUseCases.project_load_one_use_case(ps, message['filename'])
    try:
        ProjectUseCases.project_create_ros2_package_use_case(project, logger)
        # run
        ProjectUseCases.project_build_use_case(project, logger)
    finally:
        pass

    # stop redirecting the log
    while not q.empty():
        pass
    listener.stop()

    del logger
    close_room("build")


@socket_.on('save_package', namespace='/package')
def save_package(message):
    project = Project.from_dict(message['data'])
    ProjectUseCases.project_save_use_case(ps, project)
    pass


@socket_.on('get_packages_list', namespace='/package')
def get_packages_list():
    emit('packages_list', {'list': ProjectUseCases.project_get_list_use_case(ps)})


@socket_.on('get_package', namespace='/package')
def get_package(message):
    project = ProjectUseCases.project_load_one_use_case(ps, message['filename'])
    emit('package', {'data': json.dumps(Project.to_dict(project), indent=2)})


# WEBSOCKET

@app.route('/')
def rootroute():
    return redirect("/ui/", 301)


@app.route('/ui/')
def uiroot():
    return redirect("/ui/home/", 301)


@app.route('/ui/<string:target>/')
def ui(target):
    uiargs = {
        "env": [],
        "version": ""
    }
    if target == "home":
        pass
    elif target == "env":
        for item, value in os.environ.items():
            uiargs["env"].append((item, value))
        uiargs["env"] = sorted(uiargs["env"])
        pass
    elif target == "projects":
        pass
    elif target == "info":
        uiargs["version"] = settings.version
        pass
    else:
        abort(404)
    return render_template('ui.html', component=target, uiargs=uiargs)


def main():
    global ros_node

    # if os.environ.get("FLASK_ENV") == "development":
    #    app.debug = True
    #    app.secret_key = '0123456789abcdef'
    #    toolbar = DebugToolbarExtension(app)

    rclpy.init()
    ros_node = rclpy.create_node("test")
    socket_.run(app, host='0.0.0.0', port=5000, debug=True)

# if __name__ == "__main__":
#     main()
