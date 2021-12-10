.PHONY: all
all:
	@echo Nothing to do. Call a specific target please.

.PHONY: checkroot
checkroot:
ifneq ($(shell id -u), 0)
	@echo "You must be root to do this"
	exit 2
endif

.PHONY: makedirs
makedirs:
	@echo Making dirs...
	mkdir /etc/srosc
	mkdir /usr/local/lib/srosc

.PHONY: copyfiles
copyfiles:
	@echo Copying files...

.PHONY: setpermissions
setpermissions:
	@echo Setting permissions...

.PHONY: install
install: checkroot makedirs copyfiles setpermissions
	@echo Installing...
