DEBUG ?= 1
VERBOSE ?= 1

MICRO_ECC_SDK_DIR ?= ../../../external/micro-ecc
MICRO_ECC_GIT_DIR ?= micro-ecc
MICRO_ECC_LIBFILE ?= nrf52hf_armgcc/armgcc/micro_ecc_lib_nrf52.a

ifeq (${DEBUG},1)
	CONFIG='Release'
	SUFFIX = _debug
else
	CONFIG='Release'
	SUFFIX =
endif

ifeq (${VERBOSE},1)
	VERBOSE=-verbose
else
	VERBOSE=
endif

all: hex/secure_bootloader_ble_s140_patchkeeper${SUFFIX}.hex

hex/secure_bootloader_ble_s140_patchkeeper${SUFFIX}.hex:
	mkdir -p hex
	emBuild -rebuild -config ${CONFIG} patchkeeper_ble${SUFFIX}/ses/secure_bootloader_ble_s140_patchkeeper${SUFFIX}.emProject ${VERBOSE}
	cp patchkeeper_ble${SUFFIX}/ses/Output/${CONFIG}/Exe/secure_bootloader_ble_s140_patchkeeper${SUFFIX}.hex hex

clean:
	@rm -rf hex
	@find . -depth -name Output -exec rm -rf {} \;
