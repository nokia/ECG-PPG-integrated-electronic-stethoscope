#!/bin/bash
# © 2022 Nokia
# Licensed under the BSD 3-Clause License
# SPDX-License-Identifier: BSD-3-Clause
#


nrfjprog --program hex/merged_SD_Bootloader_Application.hex --chiperase
nrfjprog --reset
