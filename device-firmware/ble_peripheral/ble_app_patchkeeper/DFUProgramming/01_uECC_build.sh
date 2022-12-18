#!/bin/bash
# © 2022 Nokia
# Licensed under the BSD 3-Clause License
# SPDX-License-Identifier: BSD-3-Clause
#



# Micro_ECC is part of the SDK but requires source code from https://github.com/kmackay/micro-ecc.git.
# The script in the SDK directory takes care of all of this.

pushd ../../../../external/micro-ecc/
dos2unix build_all.sh
sh build_all.sh
popd
