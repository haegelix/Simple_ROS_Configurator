# package details
PKG_NAME=srosc
PKG_VERSION=0.2
PKG_ARCH=arm64
PKG_RELEASE=1
PKG_LICENSE="To Be Decided"
PKG_GROUP=srosc
PKG_SOURCE=srosc
PKG_ALT_SOURCE=
PAKDIR=..
MAINTAINER="haegelix@yahoo.de"
PROVIDES=srosc
REQUIRES="dash npm git"
RECOMMENDS=
SUGGESTS=
CONFLICTS=
REPLACES=
INSTALL=no
MORE_FLAGS=--nodoc --backup

# just a dummy to catch all simple calls to "make" without given target
.PHONY: dummy
dummy:
	@echo Nothing to do. Call a specific target please.

# check for superuser privileges (this target is being used in many of the others)
.PHONY: checkroot
checkroot:
ifneq ($(shell id -u), 0)
	@echo "Check for superuser privileges... FAILED"
	exit 2
endif
	@echo "Check for superuser privileges... DONE"

# make all neccessary directories
.PHONY: makedirs
makedirs: checkroot
	@echo Making dirs...
	@[ -d $(LIB_DIR) ] || mkdir ${LIB_DIR}
	@[ -d $(CFG_DIR) ] || mkdir ${CFG_DIR}
	@[ -d $(LOG_DIR) ] || mkdir ${LOG_DIR}
	@echo Making dirs... DONE

# copy all files to the correct directories
.PHONY: copy_static_files
copy_static_files: checkroot
	@echo Copying files...
	@cp ./bin/srosc ${BIN_DIR}
	@cp -r ./lib/* ${LIB_DIR}
	@cp -r ./etc/* ${CFG_DIR}
	@echo Copying files... DONE

# install js dependancies
.PHONY: install_js_dependancies
install_js_dependancies:
	@cd ./javascript-dependancies/ && $(MAKE) install

# create empty logfiles
.PHONY: createlogs
createlogs: checkroot
	@echo Creating log files...
	@touch ${LOG_DIR}runconfigs.log
	@touch ${LOG_DIR}newconfig.log
	@echo Creating log files... DONE

# set all file permissions correctly (executables may be executable...)
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
	@# new workspace
	@chmod o+x ${LIB_DIR}newws.py
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

.PHONY: setversion
setversion: checkroot
	@echo Setting version file to current version.
	@echo ${PKG_VERSION} > ${LIB_DIR}.version

# TODO create systemctl unit file
# launch the whole install routine (mainly using other targets)
.PHONY: install
install: checkroot makedirs copy_static_files install_js_dependancies createlogs setpermissions setversion
	@echo Installing... DONE

# pack a debian package
.PHONY: pack
pack:
	checkinstall -y --pkgname=${PKG_NAME} --pkgversion=${PKG_VERSION} --pkgarch=${PKG_ARCH} \
	--pkgrelease=${PKG_RELEASE} --pkglicense=${PKG_LICENSE} --pkggroup=${PKG_GROUP} --pkgsource=${PKG_SOURCE} \
	--pkgaltsource=${PKG_ALT_SOURCE} --pakdir=${PAKDIR} --maintainer=${MAINTAINER} --provides=${PROVIDES} \
	--requires=${REQUIRES} --recommends=${RECOMMENDS} --suggests=${SUGGESTS} --conflicts=${CONFLICTS} \
	--replaces=${REPLACES} --install=${INSTALL} ${MORE_FLAGS}
