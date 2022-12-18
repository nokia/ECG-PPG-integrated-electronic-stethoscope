#!/bin/bash
# © 2022 Nokia
# Licensed under the BSD 3-Clause License
# SPDX-License-Identifier: BSD-3-Clause
#


# active conda environment if used
source ~/.bash_profile
conda activate base

mkdir -p hex
nrfutil settings generate \
	--application patchkeeper/s140/ses/Output/Debug/Exe/ble_app_patchkeeper_s140.hex \
	--family NRF52840 \
	--application-version 1 \
	--bootloader-version 1 \
	--softdevice ../../../components/softdevice/s140/hex/s140_nrf52_6.1.1_softdevice.hex \
	--bl-settings-version 2 \
	hex/bootloader_Settings.hex

# mergehex -m ../../dfu/secure_bootloader/hex/secure_bootloader_ble_s140_patchkeeper_debug.hex hex/bootloader_Settings.hex -o hex/bootloaderWithSettings.hex
# mergehex -m ../../../components/softdevice/s140/hex/s140_nrf52_6.1.1_softdevice.hex hex/bootloaderWithSettings.hex -o hex/merged_SD_Bootloader.hex
# mergehex -m hex/merged_SD_Bootloader.hex patchkeeper/s140/ses/Output/Debug/Exe/ble_app_patchkeeper_s140.hex -o hex/merged_SD_Bootloader_Application.hex

mergehex -m ../../dfu/secure_bootloader/hex/secure_bootloader_ble_s140_patchkeeper_debug.hex hex/bootloader_Settings.hex ../../../components/softdevice/s140/hex/s140_nrf52_6.1.1_softdevice.hex -o hex/merged_SD_Bootloader.hex
mergehex -m hex/merged_SD_Bootloader.hex patchkeeper/s140/ses/Output/Debug/Exe/ble_app_patchkeeper_s140.hex -o hex/merged_SD_Bootloader_Application.hex
echo "Softdevice Bootloader Application hex file created"
