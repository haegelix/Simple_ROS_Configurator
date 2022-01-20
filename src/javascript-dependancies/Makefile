.PHONY: all
all: build

.PHONY: build
build: jquery.js vis-network.js vis-network.js.map socket.io.js socket.io.js.map jquery-ui.js jquery-ui.css jquery-migrate.js

## jquery.js
# download
jquery.js:
	@echo Downloading jquery
	@wget https://unpkg.com/jquery@3.6.0/dist/jquery.js

## jquery-ui
# download
jquery-ui.js jquery-ui.css:
	@echo Downloading jquery-ui
	@wget https://unpkg.com/jquery-ui-css@1.11.5/jquery-ui.js
	@wget https://unpkg.com/jquery-ui-css@1.11.5/jquery-ui.css

## jquery-migrate
# download
jquery-migrate.js:
	@echo Downloading jquery-ui
	@wget https://unpkg.com/jquery-migrate@3.3.2/dist/jquery-migrate.js

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