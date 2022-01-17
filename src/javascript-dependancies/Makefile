# needed to suppress specific download for js-package not available on headless systems
export PUPPETEER_SKIP_DOWNLOAD=true

.PHONY: all
all: install


## JQUERY
# clone if not found
./jquery:
	@echo Cloning jQuery
	@git clone https://github.com/jquery/jquery.git

# attempt an update
./jquery/.git/FETCH_HEAD: | ./jquery
	@echo Updating jQuery
	@cd jquery/ && git pull

# if changed --> recompile
jquery.js: ./jquery/.git/FETCH_HEAD
	@echo Building jQuery
	@cd jquery/ && npm install && npm run build
	@cp jquery/dist/jquery.js jquery.js


## vis-network
# clone if not found
./vis-network:
	@echo Cloning vis-network
	@git clone https://github.com/visjs/vis-network.git

# attempt an update
./vis-network/.git/FETCH_HEAD: | ./vis-network
	@echo Updating vis-network
	@cd vis-network/ && git pull

# if changed --> recompile
vis-network.js: ./vis-network/.git/FETCH_HEAD
	@echo Building vis-network
	@cd vis-network/ && npm install && npm run build
	@cp vis-network/dist/vis-network.js vis-network.js


# install
.PHONY: install
install: jquery.js vis-network.js
	@cp jquery.js ${LIB_DIR}/web/static/js/
	@cp vis-network.js ${LIB_DIR}/web/static/js/
