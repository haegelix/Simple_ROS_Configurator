BIN_DIR=/usr/local/bin/
LIB_DIR=/usr/local/lib/srosc/
CFG_DIR=/etc/srosc/

.PHONY: all
all:
	@echo Nothing to do. Call a specific target please.

.PHONY: checkroot
checkroot:
ifneq ($(shell id -u), 0)
	@echo "You must be root to do this"
	exit 2
endif

.PHONY: checkroot makedirs
makedirs:
	@echo Making dirs...
	mkdir ${LIB_DIR}
	mkdir ${CFG_DIR}

.PHONY: checkroot copyfiles
copyfiles:
	@echo Copying files...

.PHONY: setpermissions
setpermissions: checkroot
	@echo Setting permissions...
	sh ${LIB_DIR}setup.sh

.PHONY: install
install: checkroot makedirs copyfiles setpermissions
	@echo Installing...
