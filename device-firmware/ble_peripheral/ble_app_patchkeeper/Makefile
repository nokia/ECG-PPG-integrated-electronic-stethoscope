
# ========================================================================
#
# Makefile for the whole DFU build process.
#
# ========================================================================

DEBUG                   = 1
VERBOSE                 = 1
EMBUILD 		= emBuild
BUILD   		= Debug
SOFTDEVICE		= ../../../components/softdevice/s140/hex/s140_nrf52_6.1.1_softdevice.hex
FAMILY			= NRF52840
APPLICATION_VERSION	= 1
BOOTLOADER_VERSION	= 1
BL_SETTINGS_VERSION	= 2
BOOTLOADER_DIR		= ../../dfu/secure_bootloader
BOOTLOADER_BASENAME	= secure_bootloader_ble_s140_patchkeeper

ifeq (${DEBUG},1)
	CONFIG='Debug'
	BOOTLOADER_SUFFIX     = _debug
else
	CONFIG='Release'
	BOOTLOADER_SUFFIX     =
endif

ifeq (${VERBOSE},1)
	EMBUILD_VERBOSE=-verbose
else
	EMBUILD_VERBOSE=
endif


BOOTLOADER_HEX           = ${BOOTLOADER_DIR}/hex/${BOOTLOADER_BASENAME}${BOOTLOADER_SUFFIX}.hex
HEX_ZIP_SAVE_DIR		 = hex
APPLICATION_HEX          = ${HEX_ZIP_SAVE_DIR}/ble_app_patchkeeper_s140.hex

all: micro-ecc bootloader-gcc patchkeeper_app_dfu_package.zip merged_SD_Bootloader_Application.hex

clean-all: micro-ecc-clean bootloader-clean clean

clean:	app-clean hex-clean

# ========================================================================
#
# Micro-ECC Library
#
# This library is not included in the SDK and needs to be obtained from git.
# There is however a script included in the SDK that handles this - however it
# fails under OsX (and possibly Linux?) due to dos line endings. This is fixed
# by running dos2unix (and for traceability this is saved as a new file).
#
# ========================================================================

ifeq (${FAMILY},NRF52)
MICRO_ECC_LIBFILE = nrf52hf_armgcc/armgcc/micro_ecc_lib_nrf52.a
else ifeq (${FAMILY},NRF52832)
MICRO_ECC_LIBFILE = nrf52hf_armgcc/armgcc/micro_ecc_lib_nrf52.a
else ifeq (${FAMILY},NRF52840)
MICRO_ECC_LIBFILE = nrf52hf_armgcc/armgcc/micro_ecc_lib_nrf52.a
else ifeq (${FAMILY},NRF51)
MICRO_ECC_LIBFILE = nrf51_armgcc/armgcc/micro_ecc_lib_nrf51.a
else
	$(error Need to define family of MCU for Micro-ECC (e.g. NRF52))
endif

MICRO_ECC_SDK_DIR     = ../../../external/micro-ecc
MICRO_ECC_GIT_DIR     = micro-ecc

UNIX_BUILD_SCRIPT = build_all.unix.sh
ORIG_BUILD_SCRIPT = build_all.sh


${MICRO_ECC_SDK_DIR}/${UNIX_BUILD_SCRIPT} : ${MICRO_ECC_SDK_DIR}/${ORIG_BUILD_SCRIPT}
	dos2unix -n ${MICRO_ECC_SDK_DIR}/${ORIG_BUILD_SCRIPT} ${MICRO_ECC_SDK_DIR}/${UNIX_BUILD_SCRIPT}

${MICRO_ECC_SDK_DIR}/${MICRO_ECC_GIT_DIR} : ${MICRO_ECC_SDK_DIR}/${UNIX_BUILD_SCRIPT}
	cd ${MICRO_ECC_SDK_DIR} && source ${UNIX_BUILD_SCRIPT}


.PHONY:micro-ecc
micro-ecc: ${MICRO_ECC_SDK_DIR}/${MICRO_ECC_LIBFILE}
${MICRO_ECC_SDK_DIR}/${MICRO_ECC_LIBFILE}: ${MICRO_ECC_SDK_DIR}/${MICRO_ECC_GIT_DIR}

.PHONY:micro-ecc-clean
micro-ecc-clean:
	@rm -rf ${MICRO_ECC_SDK_DIR}/${MICRO_ECC_GIT_DIR}


# ========================================================================
#
# BootLoader
#
# This can be found in ../../dfu/secure_bootloader.
#
# Compiler options are emBuild (Segger's compiler using the ses project file)
# or gcc (uses the armgcc Makefile. Since everything else for the firmware
# uses the Segger compiler it makes sense to be consistent for this so a
# Makefile has been written to handle the compilation using Segger. However,
# this appears to generate a larger image has the attempt to mergehex fails
# due to conflicts. For this reason two targets have been written to select
# between armgcc and ses.
#
# There are debug and non-debug versions of the bootloader available. These
# are in separate directories: -
#
# 	patchkeeper_ble
# 	patchkeeper_ble_debug
#
# To compile and use the DEBUG version, set the DEBUG=1 flag in the call
# to this makefile.
#
# ========================================================================


.PHONY:bootloader
.PHONY:bootloader-gcc
.PHONY:bootloader-ses

bootloader: bootloader-gcc

bootloader-ses : ${MICRO_ECC_SDK_DIR}/${MICRO_ECC_LIBFILE}
	cd ${BOOTLOADER_DIR} && make DEBUG=${DEBUG} MICRO_ECC_SDK_DIR=${MICRO_ECC_SDK_DIR} MICRO_ECC_GIT_DIR=${MICRO_ECC_GIT_DIR} MICRO_ECC_LIBFILE=${MICRO_ECC_LIBFILE}

bootloader-gcc : ${MICRO_ECC_SDK_DIR}/${MICRO_ECC_LIBFILE}
	cd ${BOOTLOADER_DIR}/patchkeeper_ble${BOOTLOADER_SUFFIX}/armgcc && make
	mkdir -p ${BOOTLOADER_DIR}/hex
	cp ${BOOTLOADER_DIR}/patchkeeper_ble${BOOTLOADER_SUFFIX}/armgcc/_build/nrf52840_xxaa_s140.hex ${BOOTLOADER_HEX}

bootloader-clean :
	@cd ${BOOTLOADER_DIR} && make DEBUG=${DEBUG} clean \
		MICRO_ECC_SDK_DIR=${MICRO_ECC_SDK_DIR} \
		MICRO_ECC_GIT_DIR=${MICRO_ECC_GIT_DIR} \
		MICRO_ECC_LIBFILE=${MICRO_ECC_LIBFILE}


# ========================================================================
#
# patchkeeper Firmware Application
#
#
# patchkeeper_app_dfu_package.zip
#
# This is the Application compiled, signed, and package up ready for an
# Over-the-Air update.
#
# Note that the Firmware ID of the SoftDevice has to be defined.
# This can be found in the RELEASE NOTES for the appropriate SoftDevice.
#
# For SoftDevice 6.1.1 the Firmware ID is 0x00B7
#
# ========================================================================


.PHONY: patchkeeper
.PHONY: patchkeeper-ses
.PHONY: patchkeeper-gcc

patchkeeper : patchkeeper-ses

patchkeeper-ses:
	 cd patchkeeper/s140/ses && emBuild -rebuild -config ${CONFIG} -D BLE_DFU_ENABLED=1 ble_app_patchkeeper_s140.emProject ${EMBUILD_VERBOSE}
	 mkdir -p ${HEX_ZIP_SAVE_DIR}
	 cp -a patchkeeper/s140/ses/Output/${CONFIG}/Exe/ble_app_patchkeeper_s140.hex ${APPLICATION_HEX}
# option to build use armgcc
patchkeeper-gcc:
	cd patchkeeper/s140/armgcc && make
	mkdir -p ${HEX_ZIP_SAVE_DIR}
	cp -a patchkeeper/s140/armgcc/_build/nrf52840_xxaa.hex ${APPLICATION_HEX}

patchkeeper_app_dfu_package.zip:# ${APPLICATION_HEX}
	mkdir -p ${HEX_ZIP_SAVE_DIR}
	# cp -a patchkeeper/s140/ses/Output/${CONFIG}/Exe/ble_app_patchkeeper_s140.hex ${APPLICATION_HEX}
	nrfutil pkg generate --hw-version 52 --application-version 1 --application ${APPLICATION_HEX} --sd-req 0xb6 --key-file DFUProgramming/patchkeeper_private.key ${HEX_ZIP_SAVE_DIR}/patchkeeper_app_dfu_package.zip

app-clean:
	@rm -f ${HEX_ZIP_SAVE_DIR}/patchkeeper_app_dfu_package.zip
	@rm -rf patchkeeper/s140/ses/Output


# ========================================================================
#
# Initial Image
#
# SoftDevice + Bootloader + Bootloader Settings + patchkeeper FW Image
#
#
# This is the hex file combining everything for the initial DFU capable
# image to be programmed via nrfjprog
#
# ========================================================================


bootloader_Settings.hex:# ${APPLICATION_HEX} ${SOFTDEVICE}
	mkdir -p ${HEX_ZIP_SAVE_DIR}
	# cp -a patchkeeper/s140/ses/Output/${CONFIG}/Exe/ble_app_patchkeeper_s140.hex ${APPLICATION_HEX}
	nrfutil settings generate \
		--application ${APPLICATION_HEX} \
		--family ${FAMILY} \
		--application-version ${APPLICATION_VERSION} \
		--bootloader-version ${BOOTLOADER_VERSION} \
		--softdevice ${SOFTDEVICE} \
		--bl-settings-version ${BL_SETTINGS_VERSION} \
		--no-backup \
		${HEX_ZIP_SAVE_DIR}/bootloader_Settings.hex

bootloaderWithSettings.hex: bootloader_Settings.hex ${BOOTLOADER_HEX}
	mergehex -m ${BOOTLOADER_HEX} ${HEX_ZIP_SAVE_DIR}/bootloader_Settings.hex -o ${HEX_ZIP_SAVE_DIR}/bootloaderWithSettings.hex

merged_SD_Bootloader.hex: bootloaderWithSettings.hex
	mergehex -m ${SOFTDEVICE} ${HEX_ZIP_SAVE_DIR}/bootloaderWithSettings.hex -o ${HEX_ZIP_SAVE_DIR}/merged_SD_Bootloader.hex


merged_SD_Bootloader_Application.hex: merged_SD_Bootloader.hex
	# cp -a patchkeeper/s140/ses/Output/${CONFIG}/Exe/ble_app_patchkeeper_s140.hex ${APPLICATION_HEX}
	mergehex -m ${HEX_ZIP_SAVE_DIR}/merged_SD_Bootloader.hex ${APPLICATION_HEX} -o ${HEX_ZIP_SAVE_DIR}/merged_SD_Bootloader_Application.hex

hex-clean:
	@rm -f ${HEX_ZIP_SAVE_DIR}/*.hex

# ========================================================================
#
# Program the Initial Image
#
# ========================================================================

.PHONY: program
program: merged_SD_Bootloader_Application.hex
	nrfjprog --program ${HEX_ZIP_SAVE_DIR}/merged_SD_Bootloader_Application.hex --chiperase
	nrfjprog --reset


# ========================================================================
#
# Copy the patchkeeper_app_dfu_package to Cloud for access from other devices.
#
# TODO: Support for other cloud storage e.g. OneDrive.
# ========================================================================

OS := $(shell uname)

ifeq ($(OS), Darwin)
.PHONY:cloud
upload2cloud: patchkeeper_app_dfu_package.zip
	cp -a ${HEX_ZIP_SAVE_DIR}/patchkeeper_app_dfu_package.zip ~/Library/Mobile\ Documents/com~apple~CloudDocs
else
upload2cloud:
	@echo Not Copying to Cloud
endif

# ========================================================================
# HELP
# ========================================================================

.PHONEY: help
help:
	@echo 'make targets: -'
	@echo '     all                                      :  Prepare hex and zip files, not including program and upload2cloud '
	@echo '     micro-ecc                                :  Obtain and make the micro-ecc library. Only needs to be done once.'
	@echo '     bootloader                               :  Compile secure bootloader image'
	@echo '     patchkeeper                              :  Compile Application image with segger embedded studio'
	@echo '     patchkeeper-gcc                          :  Compile Application image with armgcc'
	@echo '     bootloader_Settings.hex                  :  Generate bootloader settings for current application'
	@echo '     bootloaderWithSettings.hex               :  Combine bootloader with bootloader settings.'
	@echo '     merged_SD_Bootloader.hex                 :  Combine bootloader with bootloader settings and SoftDevice'
	@echo '     merged_SD_Bootloader_Application.hex     :  Combine bootloader with bootloader settings, SoftDevice, and Application'
	@echo '     patchkeeper_app_dfu_package.zip          :  Generate the package for DFU update'
	@echo '     program                                  :  Program the Initial Image to nrf52 device'
	@echo '     upload2cloud                             :  Upload the initial Image to iClould on Mac computer'
	@echo '     clean                                    :  Clean volatile stuff'
	@echo '     clean-all                                :  Clean Everything including micro-ecc'
