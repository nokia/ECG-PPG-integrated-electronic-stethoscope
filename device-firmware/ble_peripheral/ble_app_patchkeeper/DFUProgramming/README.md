# DFU Helper Scripts

These scripts were used to develop the DFU support for Gecko. They are no longer maintained and will are not used - however they have been included for reference.

They may be removed at any time.

# Buttonless DFU for patchkeeper

##Reference
[https://devzone.nordicsemi.com/nordic/nordic-blog/b/blog/posts/getting-started-with-nordics-secure-dfu-bootloader
]()

Note: The above tutorial instructs to flash the softdevice first, and then the bootloader. THIS DOES NOT WORK as the bootloader overwrited the softdevice. The correct way to do this is to use mergehex to merge the two hex files together. This is part of the nRF Command Line Tools (as is nrfjprog).

There is also a step-by-step tutorial (which uses mergehex) based on the above at
[https://github.com/gamnes/nRF52832-buttonless-dfu-development-tutorial]()


## Steps

Step A. Generate keys

Step B. Build the bootloader and merge with the softdevice.

Step C. Generate DFU .zip packet

Step D. Test DFU

Step E. Update new application firmware (Optional)

##Notes
Generated Keys are in dfu/patchkeeper\_public\_key.c and dfu/patchkeeper\_private.key in the workspace.

Bootloader is dfu/secure\_bootloader

ble\_peripheral/ble\_app\_buttonless_dfu is the current starter example. This is where the zip file is located.

## Step A. Generate Keys
In ble\_peripheral/ble\_app\_buttonless_dfu run the following nrfjprog command to generate the private key

`nrfutil keys generate patchkeeper_private.key
`

Generate the public key based on the private key

`nrfutil keys display --key pk --format code patchkeeper_private.key --out_file dfu_public_key.c
`

This step has already been done and the keys are included in this repository: -

		dfu/dfu\_public\_key.c
		DFUProgramming/patchkeeper\_private.key

## Step B. Build the BootLoader


This needs to be rebuilt using the private key generated above.

Scripts have been provided to do this.

1. Install micro\_ecc in the nRF5-SDK (01\_uECC\_build.sh)
2. Build the bootloader and merge with the softdevice (02\_Bootloader\_build.sh)
3. Build the DFU image from the required application (03\_DFU\_build.sh)
4.


## Step C. Generate the DFU .zip Packet

Build the new version of the application and then build the signed package.

For an example go to ble\_peripheral/ble\_app\_buttonless\_dfu and run the 03\_DFU\_build.sh script.

## Step D. Test DFU
Program the bootloader with 04\_DFU\_Program.sh

Use over the air update to program the application. This requires nRFConnect and does not work on Linux or OSX.
