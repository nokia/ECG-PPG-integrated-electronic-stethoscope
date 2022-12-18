#!/bin/bash
# © 2022 Nokia
# Licensed under the BSD 3-Clause License
# SPDX-License-Identifier: BSD-3-Clause
#


# sd-req for SoftDevice 6.1.1 is 0xb6 for s140.
# s132_nrf52_6.1.1|0xB7|
# s140_nrf52_6.1.1|0xB6|
# active conda becuase nfrutil is installed under conda base environment
# comment these two lines if no conda installed(e.g. bash can find nrfutil)
source ~/.bash_profile
conda activate base

pushd ../patchkeeper/s140/ses
emBuild -rebuild -config 'Debug' -D BLE_DFU_ENABLED=1 ble_app_patchkeeper_s140.emProject -verbose
popd

nrfutil pkg generate --hw-version 52 --application-version 1 --application ../patchkeeper/s140/ses/Output/Debug/Exe/ble_app_patchkeeper_s140.hex --sd-req 0xb6 --key-file patchkeeper_private.key patchkeeper_app_dfu_package.zip

# Option - build the program with Segger, in which case the following command would be used...

# nrfutil pkg generate --hw-version 52 --application-version 1 --application ../patchkeeper/s140/ses/Output/Debug/Exe/ble_app_patchkeeper_s140.hex --sd-req 0xb6 --key-file patchkeeper_private.key patchkeeper_app_dfu_package.zip
