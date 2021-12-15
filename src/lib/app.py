#!/usr/bin/python3
import json
import os
import rclpy
import io
import random

from flask import Flask, escape, request, render_template, abort, redirect, Response
from flask_debugtoolbar import DebugToolbarExtension

from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure

app = Flask(__name__)

if os.environ.get("FLASK_ENV") == "development":
    app.debug = True
    app.secret_key = '0123456789abcdef'
    # toolbar = DebugToolbarExtension(app)

rclpy.init()
rosnode = rclpy.create_node("test")


# @app.route('/')
# def hello():
#    name = request.args.get("name", "World")
#    return f'Hello, {escape(name)}!'


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
        uiargs["nodes"] = rosnode.get_node_names_and_namespaces_with_enclaves()
        pass
    elif target == "topics":
        pass
    elif target == "test":
        pass
    else:
        abort(404)
    return render_template('ui.html', component=target, uiargs=uiargs)


@app.route('/api/<string:target>')
def api(target):
    if target == "test":
        pass
    elif target == "start":
        pass
    elif target == "stop":
        pass
    elif target == "topics":
        topics = rosnode.get_topic_names_and_types()
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
    app.run(host='0.0.0.0', port=5000)


if __name__ == "__main__":
    main()