#!/usr/bin/env python3
import os.path
import re
from pathlib import Path

import setuptools
import wget
from os.path import join

with open("ros2_ui/__init__.py", encoding="utf8") as f:
    version = re.search(r'__version__ = "(.*?)"', f.read()).group(1)

with open("README.md", "r") as fh:
    long_description = fh.read()

js_deps: [(str, str, str)]
js_deps = [("jQuery", "https://unpkg.com/jquery@3.6.0/dist/jquery.js", "jquery.js"),
           ("vis-network 1", "https://unpkg.com/vis-network@9.1.0/dist/vis-network.js", "vis-network.js"),
           ("vis-network 2", "https://unpkg.com/vis-network@9.1.0/dist/vis-network.js.map", "vis-network.js.map"),
           ("socket.io 1", "https://unpkg.com/socket.io-client@4.4.1/dist/socket.io.js", "socket.io.js"),
           ("socket.io 2", "https://unpkg.com/socket.io-client@4.4.1/dist/socket.io.js.map", "socket.io.js.map")]

for (title, url, filename) in js_deps:
    filepath = join('ros2_ui/web_ui/static/js-deps/', filename)
    if os.path.exists(filepath):
        print("Skipping:", title)
        continue
    print("Downloading:", title)
    wget.download(url, filepath)

os.makedirs(os.path.join(Path.home(), ".ros2_ui", "projects"), exist_ok=True)

setuptools.setup(
    name="ros2_ui",
    version=version,
    author="Tobias Haegele",
    author_email="haegelix@yahoo.de",
    description="A UI for ROS2 (robot operating system)",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/haegelix/ros2_ui",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Development Status :: 2 - Pre-Alpha",
        "Natural Language :: English"
    ],
    python_requires='>=3.6',
    scripts=['scripts/ros2_ui'],
    package_data={'ros2_ui': ['*', 'web_ui/*', 'web_ui/templates/*', 'web_ui/static/*', 'web_ui/static/css/*',
                              'web_ui/static/img/*', 'web_ui/static/js/*', 'web_ui/static/js-deps/*']},
    include_package_data=False,
    install_requires=[
        'Flask>=1.1.2',
        'dataclass-dict-convert>=1.6.3'
    ]
)
