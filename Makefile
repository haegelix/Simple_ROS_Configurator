# used directories
export BIN_DIR=/usr/local/bin/
export LIB_DIR=/usr/local/lib/srosc/
export CFG_DIR=/etc/srosc/
export LOG_DIR=/var/log/srosc/
export VAR_DIR=/var/local/srosc/

# used to pack a .deb-archive ready to get installed
.PHONY: pack
pack: build/
	@echo Packaging...
	@cd ./build/ && $(MAKE) pack

build/: src/
	@echo Delete build
	@rm -rf build/
	@echo Copying from src/ to build/
	@mkdir build
	@cp -r -u ./src/* ./build/
	@echo Start building...
	@cd ./build/ && $(MAKE) build

.PHONY: install
install: build/
	@cd ./build/ && $(MAKE) install

# delete all files that were shipped within the package
.PHONY: clean_install
clean_install:
	@rm -rvf ${CFG_DIR}
	@rm -rvf ${LOG_DIR}
	@rm -rvf ${LIB_DIR}
	@rm -rvf ${BIN_DIR}srosc

.PHONY: clean
clean:
	@echo Delete build
	@rm -rf build/
