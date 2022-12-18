#!/bin/bash -l
# © 2022 Nokia
# Licensed under the BSD 3-Clause License
# SPDX-License-Identifier: BSD-3-Clause
#


source ~/.bash_profile
cd ../../..
./generateSoftDeviceBootloaderAppHex.sh

# the following command makes sure Segger will also generate zip file for uploading with DFU:
./generateDFU_Zip.sh
