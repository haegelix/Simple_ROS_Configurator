# DIRS
BIN_DIR=/usr/local/bin/
LIB_DIR=/usr/local/lib/srosc/
CFG_DIR=/etc/srosc/
LOG_DIR=/var/log/srosc/

.PHONY: pack
pack:
	@cd ./src/ && make

.PHONY: clean_install
clean_install:
	@rm -rvf ${CFG_DIR}
	@rm -rvf ${LOG_DIR}
	@rm -rvf ${LIB_DIR}
	@rm -rvf ${BIN_DIR}srosc
