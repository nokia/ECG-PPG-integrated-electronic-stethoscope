#!/bin/bash
# © 2022 Nokia
# Licensed under the BSD 3-Clause License
# SPDX-License-Identifier: BSD-3-Clause
#


# Merge the Bootloader and the Softdevice

mergehex -m ../../../../components/softdevice/s140/hex/s140_nrf52_6.1.1_softdevice.hex ../../../dfu/secure_bootloader/patchkeeper_ble_debug/armgcc/_build/nrf52840_xxaa_s140.hex ../patchkeeper/s140/ses/Output/Debug/Exe/ble_app_patchkeeper_s140.hex -o merged_SD_Bootloader.hex

#Program the merged file

nrfjprog --program merged_SD_Bootloader.hex --chiperase

# Reboot the device so new program is run
nrfjprog --reset
