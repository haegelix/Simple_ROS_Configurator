#!/bin/bash
BIN_DIR="/usr/local/bin/"
LIB_DIR="/usr/local/lib/srosc/"
CFG_DIR="/etc/srosc/"

# srosc
chmod +x ${BIN_DIR}srosc
chmod +x ${LIB_DIR}srosc.py

# runconfigs
chmod +x ${LIB_DIR}runconfigs.sh
chmod +x ${LIB_DIR}runconfigs.py

# text ui
chmod +x ${LIB_DIR}newconfig.py

# web ui
chmod +x ${LIB_DIR}runserver.sh
chmod +x ${LIB_DIR}runserver.py
chmod +x ${LIB_DIR}app.py

# config
chmod 644 ${CFG_DIR}config.json