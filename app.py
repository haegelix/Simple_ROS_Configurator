#!/usr/bin/python3
import os
import rclpy

from flask import Flask, escape, request, render_template, abort, redirect

app = Flask(__name__)
rclpy.init()

# rosnode = rclpy.

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
        pass
    elif target == "topics":
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
    else:
        abort(404)
    return 'api worx (with target ' + target + ')'
