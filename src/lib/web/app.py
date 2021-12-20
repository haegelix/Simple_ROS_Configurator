#!/usr/bin/python3
import json
import os
import traceback

import rclpy
import io
import random
from lib.config import config

from flask import Flask, escape, request, render_template, abort, redirect, Response
from flask_debugtoolbar import DebugToolbarExtension
from rclpy.node import Node

from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure

# WEBSOCKET
from flask import Flask, render_template, session, copy_current_request_context
from flask_socketio import SocketIO, emit, disconnect
from threading import Lock

import lib.paths as paths
from lib.paths import get_packages_list

async_mode = None
# WEBSOCKET


app = Flask(__name__)
ros_node: Node
socket_ = SocketIO(app, async_mode=async_mode)


# @app.route('/')
# def hello():
#    name = request.args.get("name", "World")
#    return f'Hello, {escape(name)}!'

# WEBSOCKET
@app.route('/index')
def index():
    return render_template('index.html', sync_mode=socket_.async_mode)


@socket_.on('get_packages_list', namespace='/package')
def get_packages_list():
    emit('packages_list', {'list': get_packages_list(config.ignore_configs)})


@socket_.on('get_package', namespace='/package')
def get_package(message):
    cont = json.load(open(paths.get_srosc_ws_package_file_path(message['filename'])))
    emit('package', {'data': cont})


@socket_.on('my_broadcast_event', namespace='/test')
def test_broadcast_message(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    emit('my_response',
         {'data': message['data'], 'count': session['receive_count']},
         broadcast=True)


@socket_.on('disconnect_request', namespace='/test')
def disconnect_request():
    @copy_current_request_context
    def can_disconnect():
        disconnect()

    session['receive_count'] = session.get('receive_count', 0) + 1
    emit('my_response',
         {'data': 'Disconnected!', 'count': session['receive_count']},
         callback=can_disconnect)


@socket_.on('newpackage', namespace="/package")
def newpackage(message):
    try:
        obj = message['data']
        filename = message['filename']
        out_file = open(paths.get_srosc_ws_package_file_path(filename), "w")
        out_file.write(json.dumps(obj))
        out_file.close()
        emit('newpackage', {'status': 'done'})
    except:
        emit('newpackage', {'status': 'failed'})
        traceback.print_exc()


# WEBSOCKET

@app.route('/')
def rootroute():
    return redirect("/ui/", 301)


@app.route('/ui/')
def uiroot():
    return redirect("/ui/home/", 301)


@app.route('/graph.png')
def graphsvg():
    fig = create_figure()
    output = io.BytesIO()
    FigureCanvas(fig).print_png(output)
    return Response(output.getvalue(), mimetype='image/png')


def create_figure():
    fig = Figure()
    axis = fig.add_subplot(1, 1, 1)
    xs = range(500)
    ys = [random.randint(1, 150) for x in xs]
    axis.plot(xs, ys)
    return fig


@app.route('/ui/<string:target>/')
def ui(target):
    uiargs = {
        "env": []
    }
    if target == "home":
        pass
    elif target == "env":
        for item, value in os.environ.items():
            uiargs["env"].append((item, value))
        uiargs["env"] = sorted(uiargs["env"])
        pass
    elif target == "nodes":
        uiargs["nodes"] = ros_node.get_node_names_and_namespaces_with_enclaves()
        pass
    elif target == "topics":
        pass
    elif target == "test":
        pass
    elif target == "newpackage":
        pass
    elif target == "mypackages":
        pass
    else:
        abort(404)
    return render_template('ui.html', component=target, uiargs=uiargs)


testcontent = "old"


@app.route('/api/<string:target>', methods=['GET', 'POST'])
def api(target):
    global testcontent
    if target == "test":
        old = str(testcontent)
        testcontent = request.get_data()
        return old
    elif target == "start":
        pass
    elif target == "stop":
        pass
    elif target == "topics":
        topics = ros_node.get_topic_names_and_types()
        topics_sys = []
        topics_user = []
        for t in topics:
            if (t[0] == '/parameter_events') or (t[0] == '/rosout'):
                topics_sys.append(t)
            else:
                topics_user.append(t)
        ts = {
            "sys": topics_sys,
            "user": topics_user
        }
        return json.dumps(ts)
        pass
    else:
        abort(404)
    return 'api worx (with target ' + target + ')'


def main():
    global ros_node

    if os.environ.get("FLASK_ENV") == "development":
        app.debug = True
        app.secret_key = '0123456789abcdef'
        # toolbar = DebugToolbarExtension(app)

    rclpy.init()
    ros_node = rclpy.create_node("test")
    socket_.run(app, host='0.0.0.0', port=5000, debug=True)


if __name__ == "__main__":
    main()
