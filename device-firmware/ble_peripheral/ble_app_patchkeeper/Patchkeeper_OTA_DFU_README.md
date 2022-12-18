# Patchkeeper Over-The-Air Device-Firmware-Update

This instruction was modified based on Geoko version (with nRF52840 chip and S140 softdevice).  
For an introduction to OTA DFU look at the README in `DFUProgramming` folder.

## Prepartion
### Step 1 - Generate Keys

Keys have already been generated and are in `DFUProgramming/patchkeeper_public_key.c` and `DFUProgramming/patchkeeper_private.key` in the workspace. Replacement keys can be generated with the following commands in the `DFUProgramming` folder: -

`nrfutil keys generate patchkeeper_private.key`

`nrfutil keys display --key pk --format code private.key --out_file dfu_public_key.c`

Then copy `dfu_public_key.c` file to `../../dfu` folder. 
One pair of keys have been generated for use in this repository. Don't update them unless absolutely necessary. 

### Step 2 - Install Micro-ECC

Go to the SDK Root directory and enter external/micro-ecc. Run build_all.sh or build_all.exe. On UNIX based systems (including OS-X) it may be necessary to run dos2unix on build_all.sh first.

### Step 3 - Build Bootloader

Bootloader is in `dfu/secure_bootloader` folder. To build the bootloader go to that directory and run make. Use the argument DEBUG=1 for the debug build.

## Complie Applicaiton with Segger Embedded Stuido
### Step 4 - Compile the Application
Compile the Application in Segger Embedded Studio using:

`patchkeeper/s140/ses/ble_app_patchkeeper_s140.emProject`

The application hex file should be located under:

`patchkeeper/s140/ses/Output/Debug/Exe/ble_app_patchkeeper_s140.hex`


### Step 5 - Modify Bootloader and Combine Images
The bootloader then needs to be modified with the details of the Application executable. For this, the `generateSoftDeviceAndBootloaderHex.sh` can be used. This generates the correct bootloader settings for the Application, and merges this resultant hex with the bootloader hex, and then with the softdevice hex producing a single hex file that can be loaded by Segger Embedded Studio (SES). It also creates the combined Application, Bootloader and Softdevice for programming with nrfjprog.

### Step 6 - Program Target Device 
WHen using SES for compiling and uploadining, it calls a script (`postLinkCommands.sh`) from the <project>/s140/ses directory which calls the `generateSoftDeviceAndBootloaderAppHex.sh` after Linking and before Loading. It will then upload to program the target device with the linked hex file `hex/merged_SD_Bootloader_Application.hex`. 

Another method to uplaod the linked hex file is through command line. Under project folder, run:
`nrfjprog --program hex/merged_SD_Bootloader_Application.hex --chiperase`

### Step 7 - Future Firmware updates 
Future firmware updates can be performed with nRFConnect App on computer or phone, to upload the DFU zip package. 
To generate the package, under the project folder as bove, run the script: 
`generateDFU_Zip.sh`. This will generate `patchkeeper_app_dfu_package.zip` package, and will also copy it to iCloud folder under Mac (directary needs to be modified on Windows/linux, or comment out `copy` command). 

with `nRF Connect` app, find the DFU function and follow instructions there to upload newly generated zip file. 


## Makefile Option for Complie Applicaiton
Above application compiling and programming steps are also wrapped up in a Makefile, the Application will be compile with with armgcc (use `make help` for detailed information).

When using `Makefile`, the compiled application will be copied to `hex` folder for combining uploading and generate DFU zip package.  

### Generate DFU Zip Package for firmware update
To generate the package to transmit over the DFU protocol run:

`make patchkeeper_app_dfu_package.zip` or 

`make upload2cloud`



