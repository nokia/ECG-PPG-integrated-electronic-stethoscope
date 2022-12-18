#  Firmware Version 2xxx for PatchkeeperV2.0
This version of firmware is for deployment of PatchkeeperV2.0, includes improved drives for PPG sensor, all sensors work in this version already, here is the list of all sensors:

Digital sensors:

- IMU (BMI160, I2C)  
- Microphones (pdm digital, two)
- ECG (ADS1292, SPI)
- PPG (MAXM86161, I2C)   

And four analog sensors:  

- GSR Sensor (analog)
- Pulse Sensor (analog)
- Strain Sensor (analog)
- Temperature Sensor (analog)
- Battery monitor (analog)

## Common sensor reading interval setting for V2xxx  


	PATCHKEEPER_DEFAULT_ADC_SAMPLE_INTERVAL_MS       5
	PATCHKEEPER_DEFAULT_ECG_SAMPLE_INTERVAL_MS       8  
	PATCHKEEPER_DEFAULT_GSR_SAMPLE_INTERVAL_MS       50
	PATCHKEEPER_DEFAULT_PULSE_SAMPLE_INTERVAL_MS     50
	PATCHKEEPER_DEFAULT_STRAIN_SAMPLE_INTERVAL_MS    50
	PATCHKEEPER_DEFAULT_TEMP_SAMPLE_INTERVAL_MS      1000
	PATCHKEEPER_DEFAULT_BATTERY_SAMPLE_INTERVAL_MS   10000
	PATCHKEEPER_DEFAULT_IMU_SAMPLE_INTERVAL_MS       50
	PATCHKEEPER_DEFAULT_AUDIO_SAMPLE_INTERVAL_MS     1000   
PPG timing information is embedded in buid number.

## Version variations
Version informaton embedded in following format:

```
// FIRMWARE VERSION information define at the beginning of sdk_config.h file.  
 #define FIRMWARE_VERSION_ID FIRMWARE_MAJOR_VERSION * 1000000 + \
                             FIRMWARE_MINOR_VERSION *   10000 + \
                             FIRMWARE_PATCH_NUM *         100 + \
                             FIRMWARE_BUILD_NUM
```
### V20xx (for board testing, ppg 50sps)
V2000 : Stream and save all sensors without saving full audio: IMU, ECG, PPG, GSR, Strain, Pluse, Temperature, Battery, Audio mean/peak.   
V2010 : Audio recording test, with IMU, PPG and battery.  
>V2011 : Turn on all sensors and saving full audio (audio mean/peak disabled automatically).    
>This version not working properly, too much interrupts/work for processor?).  
V2030: ECG sensor with default setting, 125sps, data saving buffer 10Kb, for optimisation.   
V2031: ECG sensor with 500sps, 10Kb buffer is a default for all V2xxxx.    
V2032: ECG sensor with 125sps, 10Kb buffer, all sensors are turned on.


## More firmware versioned with following format:
```
a. Major version: 2 : ppg with batch reading format n
b. Minor version: 0 : Board testing. 1:no audio. 2: Full audio saved, IMU, Battery
c. Patch number:  0 : no ECG. 1. With ECG. 2: ECG and analog sensors.
d. build number:  0 : PPG 50sps. 1: PPG 99sps. 2: PPG 199sps.  
```

### V2100 - 2102
V2100 : No audio, no ECG, no Analog sensors, with IMU, battery and PPG 50sps.   
V2101 : No audio, no ECG, no Analog sensors, with IMU, battery and PPG 99sps.  
V2102 : No audio, no ECG, no Analog sensors, with IMU, battery and PPG 199sps.

### V2110 - 2112 (Add ECG to V2100)
V2110 : No audio, ECG, IMU, battery and PPG 50sps.   
V2111 : No audio, ECG, IMU, battery and PPG 99sps.  
V2112 : No audio, ECG, IMU, battery and PPG 199sps.

### V2200 - 2202 (add full audio)
V2200 : Full audio saved, no ECG, with IMU, battery and PPG 50sps (same as above V2010).   
V2201 : Full audio saved, no ECG, with IMU, battery and PPG 99sps.  
V2202 : Full audio saved, no ECG, with IMU, battery and PPG 199sps.

### V2210 - 2212 (Add ECG to V2200)
PPG timestamp may be delays in some packets, need optimasation).   

V2210 : Full audio saved, with ECG, IMU, battery and PPG 50sps.  
V2211 : Full audio saved, with ECG, IMU, battery and PPG 99sps.   
V2212 : Full audio saved, with ECG, IMU, battery and PPG 199sps.   

### V2220 - 2222 (Add analog sensors, too nuch resouce required, need optimasation)
V2220 : Full audio saved, ECG, IMU, battery, PPG 50sps, with 4 analog sensors(same as V2011).  
V2221 : Full audio saved, ECG, IMU, battery, PPG 99sps, with 4 analog sensors.  
>V2222 : Full audio saved, ECG, IMU, battery, PPG 199sps, with 4 analog sensors.  
>Too much information to be processed, not produced).


# V3xxx New PPG format for HR and HRV (Blue LED only)
FIRMWARE_BUILD_NUM 1 is used for PPG sampling rate 99sps.
V3000:  testing Audio, ECG, PPG, IMU together.
V3001: Deployment, full audio, ECG 125sps, PPG 99sps, IMU 50 sps.
V3011: Use for collecting data from dog, PPG 99sps always on, IMU 50 sps, No ECG, Audio at default gain.
V3021: Add Temperature sensor on top of V3011.
