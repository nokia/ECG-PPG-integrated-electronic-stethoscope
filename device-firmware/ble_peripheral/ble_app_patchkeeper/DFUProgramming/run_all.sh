# © 2022 Nokia
# Licensed under the BSD 3-Clause License
# SPDX-License-Identifier: BSD-3-Clause
#

sh ./01_uECC_build.sh
sh ./02_Bootloader_build.sh
sh ./03_DFU_build.sh
sh ./04_DFU_program.sh

cp -a patchkeeper_app_dfu_package.zip ~/Library/Mobile\ Documents/com~apple~CloudDocs
