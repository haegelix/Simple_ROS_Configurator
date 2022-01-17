# needed to suppress specific download for js-package not available on headless systems
export PUPPETEER_SKIP_DOWNLOAD=true

.PHONY: all
all: install

# jQuery
./jquery/.gitignore:
	@echo Cloning jQuery
	@git clone https://github.com/jquery/jquery.git

jquery.js: ./jquery/.gitignore
	@cd jquery/ && npm install && npm run build
	@cp jquery/dist/jquery.js jquery.js

# vis-network
./vis-network/.gitignore:
	@echo Cloning vis-network
	@git clone https://github.com/visjs/vis-network.git

vis-network.js: ./vis-network/.gitignore
	@cd vis-network/ && npm install && npm run build
	@cp vis-network/dist/vis-network.js vis-network.js

# install
.PHONY: install
install: jquery.js vis-network.js
	@cp jquery.js ${LIB_DIR}/web/static/js/
	@cp vis-network.js ${LIB_DIR}/web/static/js/
