# package details
PKG_NAME=srosc
PKG_VERSION=0.2
PKG_ARCH=arm64
PKG_RELEASE=1
PKG_LICENSE="To Be Decided"
PKG_GROUP=srosc
PKG_SOURCE=srosc
PKG_ALT_SOURCE=
PACK_DIR=..
MAINTAINER="haegelix@yahoo.de"
PROVIDES=srosc
REQUIRES=dash
RECOMMENDS=
SUGGESTS=
CONFLICTS=
REPLACES=
INSTALL=no
MORE_FLAGS=--nodoc

# directories
BIN_DIR=/usr/local/bin/
LIB_DIR=/usr/local/lib/srosc/
CFG_DIR=/etc/srosc/
LOG_DIR=/etc/srosc/

PACK_SRC_DIR=./src/

.PHONY: all
all:
	@echo Nothing to do. Call a specific target please.

.PHONY: checkroot
checkroot:
ifneq ($(shell id -u), 0)
	@echo "Check for superuser privileges... FAILED"
	exit 2
endif
	@echo "Check for superuser privileges... DONE"

.PHONY: makedirs
makedirs: checkroot
	@echo Making dirs...
	@[ -d $(LIB_DIR) ] || mkdir ${LIB_DIR}
	@[ -d $(CFG_DIR) ] || mkdir ${CFG_DIR}
	@[ -d $(LOG_DIR) ] || mkdir ${LOG_DIR}
	@echo Making dirs... DONE

.PHONY: copyfiles
copyfiles: checkroot
	@echo Copying files...
	@cp ./src/bin/srosc ${BIN_DIR}
	@cp -r ./src/lib/* ${LIB_DIR}
	@cp -r ./src/etc/* ${CFG_DIR}
	@echo Copying files... DONE

.PHONY: createlogs
createlogs: checkroot
	@echo Creating log files...
	touch ${LOG_DIR}runconfigs.log
	touch ${LOG_DIR}newconfig.log
	@echo Creating log files... DONE

.PHONY: setpermissions
setpermissions: checkroot
	@echo Setting permissions...
	@# srosc
	@chmod o+x ${BIN_DIR}srosc
	@chmod o+x ${LIB_DIR}srosc.py
	@# runconfigs
	@chmod o+x ${LIB_DIR}runconfigs.sh
	@chmod o+x ${LIB_DIR}runconfigs.py
	@# text ui
	@chmod o+x ${LIB_DIR}newconfig.py
	@# web ui
	@chmod o+x ${LIB_DIR}runserver.sh
	@chmod o+x ${LIB_DIR}runserver.py
	@chmod o+x ${LIB_DIR}app.py
	@# config
	@chmod 644 ${CFG_DIR}config.json
	@# log
	@chmod 777 ${LOG_DIR}
	@chmod -R 777 ${LOG_DIR}*
	@echo Setting permissions... DONE

.PHONY: install
install: checkroot makedirs copyfiles setpermissions
	@echo Installing... DONE

.PHONY: pack
pack:
	cd ${PACK_SRC_DIR} && \
	checkinstall -y --pkgname=${PKG_NAME} --pkgversion=${PKG_VERSION} --pkgarch=${PKG_ARCH} \
	--pkgrelease=${PKG_RELEASE} --pkglicense=${PKG_LICENSE} --pkggroup=${PKG_GROUP} --pkgsource=${PKG_SOURCE} \
	--pkgaltsource=${PKG_ALT_SOURCE} --pakdir=${PACK_DIR} --maintainer=${MAINTAINER} --provides=${PROVIDES} \
	--requires=${REQUIRES} --recommends=${RECOMMENDS} --suggests=${SUGGESTS} --conflicts=${CONFLICTS} \
	--replaces=${REPLACES} --install=${INSTALL} ${MORE_FLAGS}