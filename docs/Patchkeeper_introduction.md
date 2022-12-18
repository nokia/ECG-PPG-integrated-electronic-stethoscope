# Introduction 

ECG-PPG-integrated-electronic-stethoscope is a device developed by Data and Device Group of in Nokia Bell Labs at Cambridge site. The device is called Patchkeeper and the name is widely used in this repository. 

## Devices details

Patchkeeper is a wearable health device specially developed for vital signal monitoring that can be used for both human and animals. It contains an electronic stethoscope, PPG, ECG and IMU sensors. The Patchkeeper has a size of 76x52x15 mm and weight of 56 grams. 

### MCU 
The MCU used in Patchkeeper is nRF52840 from Nordic Semiconductor, which contains a 64 MHz Cortex-M4 processor with FPU. The BLE functionality was not used in this study as all the data are saved in micro-SD card on PCB board. 

### Electronic stethoscope
The electronic stethoscope contains a metal chamber head attached with a diaphragm in a similar way as a traditional stethoscope. The metal air chamber has a small hole as an air outlet that is coupled directly to a bottom facing MEMS microphone on a PCB board. The coupling is airtight sealed with rubber ring. A second MEMS microphone is placed on the same PCB, but without coupling to the stethoscope head, which can be used as an noise cancellation purpose. Two microphones work as stereo pair and sound is recorded continuously into micro-SD card on the board. The MEMS microphone used is INMP621ACEZ from TDK InvenSense Inc. It is a PDM Digital output and wide dynamic range of about 111dB. 

### ECG
ADS1292R chip is used for ECG and respiration measurement. The Patchkeeper need to be attached with a soft patch electodes for ECG measurement, and the patch can be stick upon human body.

### PPG 
Maxim Integrated MAXM86161 Optical Bio-Sensor is used for for HR and SpO2 Measurement.  

### IMU
IMU sensor used is BMI160 from Bosch Sensortec. It is a small, low power, low noise 16-bit chip designed for mobile applications. It can provide highly accurate gyroscope and accelerometer data in real time. The sampling rate for IMU sensor in this study is 50 samples per second. 

### Battery
The Patchkeeper also contains a 400mAh Lithium polymer battery which can last more than 24 hours with continuous recording all sensor data.  

# Project development environment

## Software and hardare prepartion

The firmware for Patchkeeper was developed in Segger Embedded Studio with Nordic Semiconductor nRF5 SDK 15.3.0 with softdevice version s140. 
Segger Embedded Studio can be downloaded: https://www.segger.com/products/development-tools/embedded-studio/

nRF5 SDK 15.3.0 can be downloaded: https://www.nordicsemi.com/Products/Development-software/nRF5-SDK/Download#infotabs

JLink device and its drive (https://www.segger.com/downloads/jlink/) and nRF Command Line Tools are also required for compiling and uploading the firmware to device. 

## How to start guide
### Segger Embedded Studio
Download and Install Segger Embedded Studio and get a license for Nordic MCU as needed.  
 
### nRF5 SDK 15.3.0
Download above nRF5 SDK 15.3.0 and unzip the whole package of  nRF5_SDK_15.3.0_59ac345 in a working directory and copy device-firmware folder in this repository into it. The Seggar project file for Patchkeeper can be loaded from: device-firmware/ble_peripheral/ble_app_patchkeeper/patchkeeper/s140/ses/ble_app_patchkeeper_s140.emProject
#### Modifications to nRF5 SDK 15.3.0 files
The following code modifications are required for compiling in current version of Segger Embedded Studio 6.34: 
1. 	 components/libraries/uart/regart.c 
	Line 101 change: 
	`#if defined(__SES_VERSION) && (__SES_VERSION >= 34000)` to 
	`#if defined(__SES_VERSION) && (__SES_VERSION >= 34000) && !defined(__SEGGER_RTL_VERSION)`

2. components/libraries/bsp/bsp_config.h
	Line 89 and 90: 
	`#define ADVERTISING_LED_ON_INTERVAL            50 //200`
	`#define ADVERTISING_LED_OFF_INTERVAL           9950 //1800`
	This is to make the LED advertising less frequent. 

If there are more errors during compiling, you may want to switch Wegger Embedded Studio to early version published during 2019, for instatnce V5.40.

The project can also be complied with armgcc. the Makefile is located at: device-firmware/ble_peripheral/ble_app_patchkeeper/patchkeeper/s140/armgcc 
### Hardware programming interface
First timg programming of Patchkeeper need to go through SWD interface embedded within USB-C plug.SWD pin configuration can be found in the hardware schematic files. A customized interface conenction need to be made to programmer (JLink or others). 

### Other related instructions
 Other useful documrents and related instructions are located at:
 docs/
device-firmware/ble_peripheral/ble_app_patchkeeper

# Data parsing information

Sensor's data are saved on SD card and also steamed over BLE if 
connnected. All digital data are save in DLOG\_\*.BIN file, whcih can be parsed by procesed by python scripts for human and machine reading. Audio recodings are saved in ALOG\_\*.BIN file which can be imported and processed with Audacity as rawdata with setting of 16 bits stero at 16124Hz.Left channel of audio trae is from stethoscope right channel from the other on board microphone. 
