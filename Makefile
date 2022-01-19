# used directories
export BIN_DIR=/usr/local/bin/
export LIB_DIR=/usr/local/lib/srosc/
export CFG_DIR=/etc/srosc/
export LOG_DIR=/var/log/srosc/

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

# delete all files that were shipped within the package
.PHONY: clean_install
clean_install:
	@rm -rvf ${CFG_DIR}
	@rm -rvf ${LOG_DIR}
	@rm -rvf ${LIB_DIR}
	@rm -rvf ${BIN_DIR}srosc

.PHONY: clean
clean:
	@cd ./build/ && find . ! -name .gitignore -delete
