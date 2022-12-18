#!/bin/bash
# © 2022 Nokia
# Licensed under the BSD 3-Clause License
# SPDX-License-Identifier: BSD-3-Clause
#



# This is used to generate private and public key pair.

# pushd DFUProgramming/
#generate private key
nrfutil keys generate patchkeeper_private.key
#generate public key
nrfutil keys display --key pk --format code patchkeeper_private.key --out_file dfu_public_key.c
# move public key to dfu folder
mv dfu_public_key.c ../../../dfu
