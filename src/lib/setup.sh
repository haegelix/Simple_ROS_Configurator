#!/bin/bash
BIN_DIR="/usr/local/bin/"
LIB_DIR="/usr/local/lib/srosc/"
CFG_DIR="/etc/srosc/"

# srosc
chmod o+x ${BIN_DIR}srosc
chmod o+x ${LIB_DIR}srosc.py

# runconfigs
chmod o+x ${LIB_DIR}runconfigs.sh
chmod o+x ${LIB_DIR}runconfigs.py

# text ui
chmod o+x ${LIB_DIR}newconfig.py

# web ui
chmod o+x ${LIB_DIR}runserver.sh
chmod o+x ${LIB_DIR}runserver.py
chmod o+x ${LIB_DIR}app.py

# config
chmod 644 ${CFG_DIR}config.json