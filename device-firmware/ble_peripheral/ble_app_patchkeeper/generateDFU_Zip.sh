#!/bin/bash
# © 2022 Nokia
# Licensed under the BSD 3-Clause License
# SPDX-License-Identifier: BSD-3-Clause
#


# active conda environment if used
source ~/.bash_profile
conda activate base

mkdir -p hex

# the foloowing commented out script is for combining bootloader, SoftDevice, and Application hex files together for initial upload to Patchkeeper with JLink. Future update of firmware can be done with nRFConnect app on computer or phone, to upload the DFU zip package.
# nrfutil settings generate \
# 	--application patchkeeper/s140/ses/Output/Debug/Exe/ble_app_patchkeeper_s140.hex \
# 	--family NRF52840 \
# 	--application-version 1 \
# 	--bootloader-version 1 \
# 	--softdevice ../../../components/softdevice/s140/hex/s140_nrf52_6.1.1_softdevice.hex \
# 	--bl-settings-version 2 \
# 	hex/bootloader_Settings.hex
#
# mergehex -m ../../dfu/secure_bootloader/hex/secure_bootloader_ble_s140_patchkeeper_debug.hex hex/bootloader_Settings.hex -o hex/bootloaderWithSettings.hex
# mergehex -m ../../../components/softdevice/s140/hex/s140_nrf52_6.1.1_softdevice.hex hex/bootloaderWithSettings.hex -o hex/merged_SD_Bootloader.hex
# mergehex -m hex/merged_SD_Bootloader.hex patchkeeper/s140/ses/Output/Debug/Exe/ble_app_patchkeeper_s140.hex -o hex/merged_SD_Bootloader_Application.hex
#
# echo "Softdevice Bootloader Application hex file created"

nrfutil pkg generate \
 	--hw-version 52 \
	--application-version 1 \
	--application patchkeeper/s140/ses/Output/Debug/Exe/ble_app_patchkeeper_s140.hex \
	--sd-req 0xb6 \
	--key-file DFUProgramming/patchkeeper_private.key \
	 hex/patchkeeper_app_dfu_package.zip
     # hex/patchkeeper_app_dfu_package_V2300_test.zip

echo "DFU zip package created: hex/patchkeeper_app_dfu_package_Vxxxx.zip"
# cp -a hex/patchkeeper_app_dfu_package_V2300_test.zip ~/Library/Mobile\ Documents/com~apple~CloudDocs
cp -a hex/patchkeeper_app_dfu_package.zip ~/Library/Mobile\ Documents/com~apple~CloudDocs

echo "copied zip package to iCould folder"
