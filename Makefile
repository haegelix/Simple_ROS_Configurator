# used directories
export BIN_DIR=/usr/local/bin/
export LIB_DIR=/usr/local/lib/srosc/
export CFG_DIR=/etc/srosc/
export LOG_DIR=/var/log/srosc/

# used to pack a .deb-archive ready to get installed
.PHONY: pack
pack: src/
	@echo Copying from src/ to build/
	@cp -r ./src/* ./build/
	@echo Packaging...
	@cd ./build/ && $(MAKE) pack

# delete all files that were shipped within the package
.PHONY: clean_install
clean_install:
	@rm -rvf ${CFG_DIR}
	@rm -rvf ${LOG_DIR}
	@rm -rvf ${LIB_DIR}
	@rm -rvf ${BIN_DIR}srosc

.PHONY: clean
clean:
	@cd ./build/ && rm -rvf !(.gitignore)