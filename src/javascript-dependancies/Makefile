.PHONY: all
all: build

.PHONY: build
build: jquery.js vis-network.js vis-network.js.map socket.io.js socket.io.js.map

## jquery.js
# download
jquery.js:
	@echo Downloading jquery
	@wget https://unpkg.com/jquery@3.6.0/dist/jquery.js

## vis-network
# download
vis-network.js vis-network.js.map:
	@echo Building vis-network
	@wget https://unpkg.com/vis-network@9.1.0/dist/vis-network.js
	@wget https://unpkg.com/vis-network@9.1.0/dist/vis-network.js.map

## socketio-client
# download
socket.io.js socket.io.js.map:
	@echo Downloading socket.io
	@wget https://unpkg.com/socket.io-client@4.4.1/dist/socket.io.js
	@wget https://unpkg.com/socket.io-client@4.4.1/dist/socket.io.js.map

# install
.PHONY: install
install: build
	# JavaScript
	@cp *.js ${LIB_DIR}/web/static/js/
	@cp *.js.map ${LIB_DIR}/web/static/js/
	# CSS
	@cp *.css ${LIB_DIR}/web/static/css/