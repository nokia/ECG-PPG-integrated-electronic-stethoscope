import os
import sys
from struct import *
from pathlib import Path
import glob

from matplotlib import pyplot as plt
from matplotlib import ticker as ticker # NOQA
import matplotlib as mpl
import numpy as np
import time
import pandas as pd

mpl.style.use('fast')
mpl.rcParams['lines.linewidth'] = 0.5

if (sys.platform == "darwin"):
    sourcedir = "/Volumes/NO NAME/"
elif(sys.platform == "win32"):
    sourcedir = "D:/"
    # sourcedir = "C:/Users/hongweil/work/Dog_data_collection1/"

else:
    sourcedir = "/dev/sda1/"

fileprefix = ''

# sourcedir = "~/work/Nokia-VTT/data-processing-scripts/Patchkeeper/"
# filetimestamp='202011251755'
# fileprefix = filetimestamp + '_'

# sourcedir = "data/"
# filetimestamp=sys.argv[1]
# fileprefix = filetimestamp + '_'

filebasename = 'DLOG'
filesuffix = '.BIN'

# Sensor Packets
# Format is: -
#
#   16 bytes packet_info + subpackets
#   typedef struct {
#       packet_info_t             packet_info;
#       adc_sensor_data_buffer_t  data[PATCHKEEPER_TEMP_BUFFER_SIZE];
#   } patchkeeper_temp_data_t;

# current setup in Segger: PATCHKEEPER_TEMP_BUFFER_SIZE = 2

# subpackt: adc_sensor_data_buffer_t:
#   typedef struct {
#    uint32_t          ts_lo;
#    float_t           value;
#    } adc_sensor_data_buffer_t; // 12 bytes

# Where Packet header format is
##   #define SOP_BELLLABS 0x42656c6c4c616273
#   typedef struct {
#       uint32_t  sop; //lleB
#       uint32_t  timestamp_hi;
#       uint32_t  timestamp_lo;
#       uint8_t   logger_id;
#       uint16_t   length;
#       uint8_t  custom;
#   } packet_info_t;

#                  sop  ts_hi ts_lo lid len cus
PacketHeaderSize = 4 + 4 + 4 + 1 + 1 + 2

MagicString = b"Bell"[::-1]
MagicNumber = unpack('I', MagicString)

Config_ID: int = 0x00
IMU_ID: int = 0x2a
ECG_ID: int = 0x20
AUdio_ID: int = 0x04
Battery_ID: int = 0x05
GSR_ID: int = 0x22
STRAIN_ID: int = 0x24
PULSE_ID: int = 0x26
TEMP_ID: int = 0x28
PPG_ID: int = 0x2b
PPG_HR_ID: int = 0x2c
PPG_SPO2_ID: int = 0x2d

# Later versions of datalogger change the format
# e.g. coding of battery voltage is different.
# Select with version is used here.(add details about each version)

# All following defaults need to be set, as some sub-sequent file doesn't contains related information.
#
PrescalerValue = 0  # this was default, can be set to diffent in firmware, need to read from config_data
RTCInputFrequency = 32768 #same as above
Ticks_per_second: float = RTCInputFrequency / (PrescalerValue + 1)
DataPackingVersion: int = 3 #current default is 3, with 64 bits timstamp
FirmwareVersion: int = 3001 #current deault
FirmwareBuildNum: int = 1 #current default
#Number of digis after second decimal point, 3: ms, 6: us
TimeSavingPrecision = 6

# IMU Data Arrays

gyro_range = 2000
# Parameter Symbol Condition Min Typ Max Unit
# Range
# 125 °/s
# RFS250 250 °/s
# RFS500 500 °/s
# RFS1000 1,000 °/s
# RFS2000 2,000 °/s
gyro_sensitivity =32767/gyro_range


acce_sensitivity = 16384  #acce range ±2g
#
# Parameter Symbol Condition Min Typ Max Units
# #Resolution 16 bit
# S2g gFS2g, TA=25°C 15729 16384 17039 LSB/g
# S4g gFS4g, TA=25°C 7864 8192 8520 LSB/g
# S8g gFS8g, TA=25°C 3932 4096 4260 LSB/g
# S16g gFS16g, TA=25°C 1966 2048 2130 LSB/g


gx = []
gy = []
gz = []
ax = []
ay = []
az = []
imu_internal_ts: int = []
imu_ts: int = []

# Audio Sensor Data
audio_mean: int = []
audio_peak: int = []
audio_ts: int = []

# Battery Level
battery_percent: int = []
battery_level: float = []
battery_ts: int = []

# GSR Data Arrays
gsr_ts: int = []
gsr: float = []
# Strain Data Arrays
strain_ts: int = []
strain: float = []
# Pulse Data Arrays
pulse_ts: int = []
pulse: float = []
# Temperature Data Arrays
temperature_ts: int = []
temperature: float = []

# ECG Data Arrays
ecg_ts: int = []
ecg_ch1: int = []
ecg_ch2: int = []

# PPG Data Arrays
ppg_ts: int = []
ppg_green: int = []
ppg_infrared: int = []
ppg_red: int = []
ppg_ambient: int = []

# ppg_ts_packet: int =[]
# ppg_green_packet: int = []
# ppg_infrared_packet: int = []
# ppg_red_packet: int = []
# ppg_ambient_packet: int = []


def clear_data():
        # IMU Data Arrays
    gx.clear()
    gy.clear()
    gz.clear()
    ax.clear()
    ay.clear()
    az.clear()
    imu_internal_ts.clear()
    imu_ts.clear()

    # Analog Sensor Data Arrays
    gsr_ts.clear()
    gsr.clear()

    strain_ts.clear()
    strain.clear()

    pulse_ts.clear()
    pulse.clear()

    temperature_ts.clear()
    temperature.clear()
    # Audio Sensor Data
    audio_mean.clear()
    audio_peak.clear()
    audio_ts.clear()

    # Battery Level
    battery_percent.clear()
    battery_level.clear()
    battery_ts.clear()
    # ECG Data Arrays
    ecg_ts.clear()
    ecg_ch1.clear()
    ecg_ch2.clear()
    # PPG Data Arrays
    ppg_ts.clear()
    ppg_green.clear()
    ppg_infrared.clear()
    ppg_red.clear()
    ppg_ambient.clear()

    # ppg_ts_packet.clear()
    # ppg_green_packet.clear()
    # ppg_infrared_packet.clear()
    # ppg_red_packet.clear()
    # ppg_ambient_packet.clear()



def align32(size):
    offset = size % 4
    if offset:
        size += 4 - offset

    return size

def find_sop(SensorData_PacketHeader) :
    sop = 0
    skipped = 0
    finished = 0
    global read_offset

    while (sop != MagicNumber[0]):
        if (FirmwareVersion >= 3000):
            try:
                sop, timestamp_hi, timestamp_lo, logger_id, custom,length = unpack('IIIBBH', SensorData_PacketHeader)
                # the following is old version of the header before V2xxx:
                # the positions and sizes of length and custom are exhanged:
                # custom: used for package number sequence cycling 0-255 nowself.
                # length of data in the packet, now using two bytes at the end of the packet header, to enable holding data of very long 2^16 (much more than one BLE packet).
            except:
                print("Incomplete header.")
                finished = 1
                return -1

        else:
            try:
                sop, timestamp_hi, timestamp_lo, logger_id, length, custom = unpack('IIIBBH', SensorData_PacketHeader)
                # print("sop:",repr(sop))

            except:
                print("Incomplete header.")
                finished = 1
                return -1

        if (sop == MagicNumber[0]):
            # print("logger_id: ",hex(logger_id).zfill(2)," No. Package: ",custom)
            # print("logger_id: 0x"+str(hex(logger_id)[2:].zfill(2))," No. Package: "+str(custom))
            # print("SensorData_PacketHeader data length: ", repr(length))
            # print("read_offset: ", repr(read_offset))
            break
        try:
            SensorData_PacketHeader.pop(0)
            SensorData_PacketHeader.append(bytearray(f.read(1))[0])
            read_offset +=1
            skipped += 1
        except:
            print("No enough data SensorData.")
            finished = 1
            break
            # return -1

    if (logger_id == Config_ID):
        length = 32
        # config data has 32 bytes in total, the length and custom positions are swapped from V2xxx to V3xxx

    return (finished, skipped, sop, timestamp_hi, timestamp_lo, logger_id, custom,length)
    # return (finished, skipped, sop, timestamp_hi, timestamp_lo, logger_id, length, custom)

def process_config_data(SensorData,length):
    global PrescalerValue
    global RTCInputFrequency
    global Ticks_per_second
    global DataPackingVersion
    global FirmwareBuildNum
    global FirmwareVersion


    # print("Config Data Handler")
    PrescalerValue = (SensorData[3] << 24 | SensorData[2] << 16 << SensorData[1] << 8 | SensorData[0])
    RTCInputFrequency = (SensorData[5] << 8 | SensorData[4])
    DataPackingVersion = (SensorData[7] << 8 | SensorData[6])
    FirmwareVersion = (SensorData[27]<<24 | SensorData[26]<< 16 | SensorData[25]<< 8 | SensorData[24])
    FirmwareVersion = FirmwareVersion//1000000*1000+FirmwareVersion%1000000//10000*100+FirmwareVersion%10000//100*10+FirmwareVersion%10
    FirmwareBuildNum = FirmwareVersion%10

    print("Updating PrescalarValue to ", PrescalerValue)
    print("Updating RTC Input Frequency to ", RTCInputFrequency)
    print("Data Format Version(should be 3): ", DataPackingVersion)
    print("Firmware Version(simplified): ", FirmwareVersion)
    print("FirmwareBuildNum(0x): ", FirmwareBuildNum)

    Ticks_per_second = RTCInputFrequency / (PrescalerValue + 1)

    return

def process_imu_data(SensorData,length):
    # print("IMU Data Handler")
    gx.append(SensorData[1] << 8 | SensorData[0])
    gy.append(SensorData[3] << 8 | SensorData[2])
    gz.append(SensorData[5] << 8 | SensorData[4])
    ax.append(SensorData[7] << 8 | SensorData[6])
    ay.append(SensorData[9] << 8 | SensorData[8])
    az.append(SensorData[11] << 8 | SensorData[10])
    imu_internal_ts.append(SensorData[14] << 16 | SensorData[13] << 8 | SensorData[12])
    imu_ts.append(packet_time)

    imufile.write(str(packet_time)+
    ","+str(round(np.int16(SensorData[1] << 8 | SensorData[0])/gyro_sensitivity,6))+\
    ","+str(round(np.int16(SensorData[3] << 8 | SensorData[2])/gyro_sensitivity,6))+\
    ","+str(round(np.int16(SensorData[5] << 8 | SensorData[4])/gyro_sensitivity,6))+\
    ","+str(round(np.int16(SensorData[7] << 8 | SensorData[6])/acce_sensitivity,6))+\
    ","+str(round(np.int16(SensorData[9] << 8 | SensorData[8])/acce_sensitivity,6))+\
    ","+str(round(np.int16(SensorData[11] << 8 | SensorData[10])/acce_sensitivity,6))+\
    "\n")
    return

def process_gsr_data(SensorData,length):
    # print("GSR Data Handler")
    # print(SensorData)
    # print(timestamp_hi)
    timestamp_lo1, val1, timestamp_lo2, val2 = unpack('IfIf', SensorData)
    # gsr_time1 = (timestamp_hi<<32 | timestamp_lo1)
    gsr_time1 = (timestamp_hi * (2**32)) + timestamp_lo1
    gsr_time2 = (timestamp_hi * (2**32)) + timestamp_lo2
    gsr.append(val1)
    gsr.append(val2)
    gsr_ts.append(gsr_time1 / Ticks_per_second)
    gsr_ts.append(gsr_time2 / Ticks_per_second)
    # print('gsr_time1: ', gsr_time1/Ticks_per_second)
    # print('val1: ', val1)
    # print('gsr_time2: ', gsr_time2/Ticks_per_second)
    # print('val1: ', val2)

    gsrfile.write(str(round(gsr_time1/Ticks_per_second,TimeSavingPrecision))+\
    ","+str(val1)+\
    "\n"+ str(round(gsr_time2/Ticks_per_second,TimeSavingPrecision))+\
    ","+str(val2)+\
    "\n")

    # the above code can be replace with a loop, use unpack_from(fmt, buffer, offset=0)
    return

def process_strain_data(SensorData,length):
    timestamp_lo1, val1, timestamp_lo2, val2 = unpack('IfIf', SensorData)
    strain_time1 = (timestamp_hi * (2**32)) + timestamp_lo1
    strain_time2 = (timestamp_hi * (2**32)) + timestamp_lo2
    strain.append(val1)
    strain.append(val2)
    strain_ts.append(strain_time1 / Ticks_per_second)
    strain_ts.append(strain_time2 / Ticks_per_second)

    strainfile.write(str(round(strain_time1/Ticks_per_second,TimeSavingPrecision))+\
    ","+str(val1)+\
    "\n"+ str(round(strain_time2/Ticks_per_second,TimeSavingPrecision))+\
    ","+str(val2)+\
    "\n")
    return

def process_pulse_data(SensorData,length):
    timestamp_lo1, val1, timestamp_lo2, val2 = unpack('IfIf', SensorData)
    pulse_time1 = (timestamp_hi * (2**32)) + timestamp_lo1
    pulse_time2 = (timestamp_hi * (2**32)) + timestamp_lo2
    pulse.append(val1)
    pulse.append(val2)
    pulse_ts.append(pulse_time1 / Ticks_per_second)
    pulse_ts.append(pulse_time2 / Ticks_per_second)

    pulsefile.write(str(round(pulse_time1/Ticks_per_second,TimeSavingPrecision))+\
    ","+str(val1)+\
    "\n"+ str(round(pulse_time2/Ticks_per_second,TimeSavingPrecision))+\
    ","+str(val2)+\
    "\n")

    # the above code can be replace with a loop, use unpack_from(fmt, buffer, offset=0)
    return

# process temperature data as origina data was voltage measured by the circuits.
Rmatch = 2.74e3
Rbalance = 27.4e3
R_ratio = Rmatch/(Rmatch+Rbalance)
Gain = 2*(200e3/80.6e3)
Vref = 2.048
# Thermister_ratio = TemVoltage/Vref/Gain + R_ratio
# R_th  = Thermister_ratio*27.4e3/(1-Thermister_ratio)
Temp_df = pd.read_csv(r'KS103J34.csv')
# print (Temp_df)
# colname = Temp_df.columns[1]
# print (colname)

def process_temp_data(SensorData,length):

    # print("GSR Data Handler")
    # try:
    # print(SensorData)
    # print(timestamp_hi)
    timestamp_lo1, Temp_val1, timestamp_lo2, Temp_val2 = unpack('IfIf', SensorData)
    temp_time1 = (timestamp_hi * (2**32)) + timestamp_lo1
    temp_time2 = (timestamp_hi * (2**32)) + timestamp_lo2

    Thermister_ratio1 = Temp_val1/Vref/Gain + R_ratio
    R_th1  = Thermister_ratio1*Rbalance/(1-Thermister_ratio1)
    Thermister_ratio2 = Temp_val2/Vref/Gain + R_ratio
    R_th2  = Thermister_ratio2*Rbalance/(1-Thermister_ratio2)

    index1 =abs(Temp_df['Resistance(ohm)']-R_th1).idxmin(axis=0)
    index2 =abs(Temp_df['Resistance(ohm)']-R_th2).idxmin(axis=0)

    p = np.polyfit(Temp_df.iloc[index1-100:index1+100]['Resistance(ohm)'], Temp_df.iloc[index1-100:index1+100]['Temp(C)'], deg=3)
    pl = np.poly1d(p)
    val1 = pl(R_th1)
    val2 = pl(R_th2)

    temperature.append(val1)
    temperature.append(val2)
    temperature_ts.append(temp_time1 / Ticks_per_second)
    temperature_ts.append(temp_time2 / Ticks_per_second)

    tempfile.write(str(round(temp_time1/Ticks_per_second,TimeSavingPrecision))+ ","+ str(round(val1,2))+\
    "\n"   +       str(round(temp_time2/Ticks_per_second,TimeSavingPrecision))+ ","+ str(round(val2,2))+ "\n")

    return

def process_audio_data(SensorData,length):
    # print("Audio Data Handler")
    audio_mean.append(SensorData[1] << 8 | SensorData[0])
    audio_peak.append(SensorData[3] << 8 | SensorData[2])
    audio_ts.append(packet_time)
    # print("mean     : ", repr(hex(SensorData[1] << 8 | SensorData[0])))
    # print("peak     : ", repr(hex(SensorData[3] << 8 | SensorData[2])))

    audiofile.write(str(packet_time)+\
    ","+str(np.int16(SensorData[1] << 8 | SensorData[0]))+\
    ","+str(np.int16(SensorData[3] << 8 | SensorData[2]))+\
    "\n")
    return

def process_battery_data(SensorData,length):
    # global DataPackingVersion
    battery_percent.append(SensorData[0])
    if (DataPackingVersion>=3):
        battery_volt = 3.0 + SensorData[1]/200.0;
    else:
        battery_volt = (SensorData[1]+300.0)/100.0;
    battery_level.append(battery_volt)
    battery_ts.append(packet_time)

    batteryfile.write(str(packet_time)+\
    ","+str(np.int8(SensorData[0]))+\
    ','+str(battery_volt)+\
    "\n")
    return

def process_ecg_data(SensorData,length):

    # print("ecg data length:" + str(length))

        # for chunk in struct.iter_unpack('IBBBBBBBB',SensorData):      # Read byte chunks until empty byte returned
            # Do stuff with chunk
        #   size = calcsize(SensorData)
    for k in range(0, length//12):
        offset = 12*k
        timestamp_lo, status, sub_packet_count, ch1_0,ch1_1,ch1_2,ch2_0,ch2_1,ch2_2  = unpack_from('IBBBBBBBB', SensorData,offset)
        # timestamp_lo, status, sub_packet_count, np.int32(ch1_0),np.int32(ch1_1),np.int32(ch1_2),np.int32(ch2_0),\
        #   np.int32(ch2_1),np.int32(ch2_2) = unpack_from('IBBBBBBBB', SensorData,offset)
        ecg_time = round(((timestamp_hi * (2**32)) + timestamp_lo)/Ticks_per_second,TimeSavingPrecision)
        # print("ecg_time0" + str(ecg_time0))
        # ecg_time = round((packet_time) + k/500,7)
        # print("ecg_time:" + str(ecg_time))
        # print(np.int32(ch1_0))
        # # print(np.int32(ch1_0) <<24)

        # print(np.int32(ch1_1))
        # # print(np.int32(ch1_1) << 16)

        # print(np.int32(ch1_2))
        # # print(np.int32(ch1_2) << 8)

        # print((np.int32(ch1_0) << 24 | np.int32(ch1_1) << 16 | np.int32(ch1_2) << 8))
        # print((np.uint32(ch1_0) << 24 | np.uint32(ch1_1) << 16 | np.uint32(ch1_2) << 8))
        # print(np.int32((np.uint32(ch1_0) << 24 | np.uint32(ch1_1) << 16 | np.uint32(ch1_2) << 8)) >> 8)

        # print(np.int32(ch2_0))
        # print(np.int32(ch2_0) <<24)
        #
        # print(np.int32(ch2_1))
        # print(np.int32(ch2_1) << 16)
        #
        # print(np.int32(ch2_2))
        # print(np.int32(ch2_2) << 8)
        # print((np.int32(ch2_0) << 24 | np.int32(ch2_1) << 16 | np.int32(ch2_2) << 8))

        ch1_val = np.int32((np.uint32(ch1_0) << 24 | np.uint32(ch1_1) << 16 | np.uint32(ch1_2) << 8)) >> 8
        ch2_val = np.int32((np.uint32(ch2_0) << 24 | np.uint32(ch2_1) << 16 | np.uint32(ch2_2) << 8)) >> 8
        ecg_ts.append(ecg_time)
        ecg_ch1.append(ch1_val)
        ecg_ch2.append(ch2_val)

        ecgfile.write(str(ecg_time)+\
        ","+ str(ch1_val)+ "," + str(ch2_val) + "\n")

        # print(str(ecg_time)+\
            # ","+ str(ch1_val)+ "," + str(ch2_val) + "\n")
    # the above code can be replace with a loop, use unpack_from(fmt, buffer, offset=0)
    return


MAXM86161_REG_FIFO_DATA_MASK =   0x07FFFF
MAXM86161_REG_FIFO_RES    =      19
MAXM86161_REG_FIFO_TAG_MASK =    0x1F

def process_ppg_data(SensorData,length):
    val1:int = 0
    val2:int = 0
    val3:int = 0
    ppg_sps  = 0  #this value is currently embedded by the firmwarebuildNum()

    if FirmwareBuildNum == 0:
        ppg_sps = 50.027
    elif FirmwareBuildNum == 1:
        ppg_sps = 99.902
    elif FirmwareBuildNum == 2:
        ppg_sps = 199.805
    else:
        ppg_sps = 99.902

    ppg_data_length = len(SensorData)
    # print("ppg_data_length: ", repr(ppg_data_length))
    # print("ppg packet_time,",repr(packet_time))
    # print("No. of tag/val reading pairs: "+ repr(ppg_data_length))
    No_points_each_cycle = 3
    No_bytes_each_data = 3
    offset = 0;
    for k in range(0,(ppg_data_length//No_bytes_each_data)//No_points_each_cycle):
        # print("k: ", k)
        ppg_datapoint_time = packet_time + round(k/ppg_sps,TimeSavingPrecision)
        # print("start offest: " ,offset)
        for i in range(0,No_points_each_cycle):
            # print("offset: ", offset)
            block_buf = unpack_from('BBB', SensorData,offset)
            # print(SensorData)
            temp_data:int = block_buf[0] << 16 | block_buf[1] << 8 | block_buf[2]
            data_val:int = temp_data & MAXM86161_REG_FIFO_DATA_MASK
            tag:int = (temp_data >> MAXM86161_REG_FIFO_RES) & MAXM86161_REG_FIFO_TAG_MASK
            # print("k:tag:value : " + repr(k) + " : " + repr(tag)+" : "+ repr(data_val))

            # print(data_val)
            if tag == 1:
                val1 = data_val
            elif tag == 2:
                val2 = data_val
            elif tag == 3:
                val3 = data_val
            else:
                print("No correct tag,  point use previous value................................" )

                break

            offset += No_bytes_each_data

        ppg_ts.append(ppg_datapoint_time)
        ppg_green.append(val1)
        ppg_infrared.append(val2)
        ppg_red.append(val3)

        # print("length ppg_ts:", repr(len(ppg_ts)))
        # print("length ppg_green:", repr(len(ppg_green)))
        # print("length ppg_infrared:", repr(len(ppg_infrared)))
        # print("length ppg_red:", repr(len(ppg_red)))

        ppgfile.write(str(ppg_datapoint_time)+\
            ","+ str(val1)+ "," + str(val2) + "," + str(val3) + "\n")
    return

def process_ppg_hr_data(SensorData,length):
    val1:int = 0
    val2:int = 0
    # val3:int = 0
    ppg_sps  = 0  #this value is currently embedded by the firmwarebuildNum()

    if FirmwareBuildNum == 0:
        ppg_sps = 50.027
    elif FirmwareBuildNum == 1:
        ppg_sps = 99.902
    elif FirmwareBuildNum == 2:
        ppg_sps = 199.805
    else:
        ppg_sps = 99.902

    ppg_data_length = len(SensorData)
    # print("ppg_data_length: ", repr(ppg_data_length))
    # print("ppg packet_time:",repr(packet_time))
    # print("No. of tag/val reading pairs: "+ repr(ppg_data_length))
    No_points_each_cycle = 2
    No_bytes_each_data = 3
    No_cycles = (ppg_data_length//No_bytes_each_data)//No_points_each_cycle

    unpack_data_in_sequence = 0

    if unpack_data_in_sequence == 0:
    # unpack data in reversed sequence
        ppg_ts_packet: int =[]
        ppg_green_packet: int = []
        # ppg_infrared_packet: int = []
        # ppg_red_packet: int = []
        ppg_ambient_packet: int = []

        offset = (No_cycles-1) * (No_bytes_each_data*No_points_each_cycle)

        valid_pts = 1
        valid_ite = 0
        for k in range(0, No_cycles):
            if valid_pts == 1:
                ppg_datapoint_time = packet_time - round((valid_ite)/ppg_sps,TimeSavingPrecision)

            try:
                block_buf = unpack_from('BBBBBB', SensorData,offset)
                temp_data1:int = block_buf[0] << 16 | block_buf[1] << 8 | block_buf[2]
                data_val1:int = temp_data1 & MAXM86161_REG_FIFO_DATA_MASK
                tag1:int = (temp_data1 >> MAXM86161_REG_FIFO_RES) & MAXM86161_REG_FIFO_TAG_MASK
                temp_data2:int = block_buf[3] << 16 | block_buf[4] << 8 | block_buf[5]
                data_val2:int = temp_data2 & MAXM86161_REG_FIFO_DATA_MASK
                tag2:int = (temp_data2 >> MAXM86161_REG_FIFO_RES) & MAXM86161_REG_FIFO_TAG_MASK

                # print("ppg_datapoint_time: " +str(round(ppg_datapoint_time,TimeSavingPrecision)))
                # print("k:tag1:value1 : " +repr(k) + " : " + repr(tag1)+" : "+ repr(data_val1))
                # print("k:tag2:value2 : " +repr(k) + " : " + repr(tag2)+" : "+ repr(data_val2))

                if tag1 == 1 and tag2 == 2:
                    ppg_ts_packet.append(ppg_datapoint_time)
                    ppg_green_packet.append(data_val1)
                    ppg_ambient_packet.append(data_val2)
                    valid_pts = 1
                    valid_ite += 1
                else:
                    valid_pts = 0
            except:
                 print("Not enough data SensorData.")

            offset = offset - No_bytes_each_data*No_points_each_cycle

        ppg_ts_packet0 =  ppg_ts_packet[::-1]
        ppg_green_packet0 = ppg_green_packet[::-1]
        ppg_ambient_packet0 = ppg_ambient_packet[::-1]
        ppg_ts.extend(ppg_ts_packet0)
        ppg_green.extend(ppg_green_packet0)
        ppg_ambient.extend(ppg_ambient_packet0)

        for i in range(0,len(ppg_ts_packet0)):
            ppghrfile.write(str(ppg_ts_packet0[i])+\
                        ","+ str(ppg_green_packet0[i])+ "," + str(ppg_ambient_packet0[i]) + "\n")


    else:
    # unpack data in sequence
        offset = 0;
        for k in range(0, No_cycles):
            ppg_datapoint_time = packet_time - round((No_cycles-k-1)/ppg_sps,TimeSavingPrecision)
            # print("ppg_datapoint_time: " +str(round(ppg_datapoint_time,TimeSavingPrecision)))
            try:
                block_buf = unpack_from('BBBBBB', SensorData,offset)
                #unkack two readings at a time,with same timestamps
                temp_data1:int = block_buf[0] << 16 | block_buf[1] << 8 | block_buf[2]
                data_val1:int = temp_data1 & MAXM86161_REG_FIFO_DATA_MASK
                tag1:int = (temp_data1 >> MAXM86161_REG_FIFO_RES) & MAXM86161_REG_FIFO_TAG_MASK
                # print("k:tag1:value1 : " +repr(k) + " : " + repr(tag1)+" : "+ repr(data_val1))
                temp_data2:int = block_buf[3] << 16 | block_buf[4] << 8 | block_buf[5]
                data_val2:int = temp_data2 & MAXM86161_REG_FIFO_DATA_MASK
                tag2:int = (temp_data2 >> MAXM86161_REG_FIFO_RES) & MAXM86161_REG_FIFO_TAG_MASK
                # print("k:tag2:value2 : " +repr(k) + " : " + repr(tag2)+" : "+ repr(data_val2))

                if tag1 == 1 and tag2 == 2:
                    ppg_ts.append(ppg_datapoint_time)
                    ppg_green.append(data_val1)
                    ppg_ambient.append(data_val2)
                    ppghrfile.write(str(ppg_datapoint_time)+\
                    ","+ str(data_val1)+ "," + str(data_val2) + "\n")
            except:
                print("Not enough data SensorData.")
            offset += (No_bytes_each_data*No_points_each_cycle)


    return

def process_ppg_spo2_data(SensorData,length):
    val1:int = 0
    val2:int = 0
    val3:int = 0
    ppg_sps  = 0  #this value is currently embedded by the firmwarebuildNum()

    if FirmwareBuildNum == 0:
        ppg_sps = 50.027
    elif FirmwareBuildNum == 1:
        ppg_sps = 99.902
    elif FirmwareBuildNum == 2:
        ppg_sps = 199.805
    else:
        ppg_sps = 99.902

    ppg_data_length = len(SensorData)
    # print("ppg_data_length: ", repr(ppg_data_length))
    # print("ppg packet_time,",repr(packet_time))
    # print("No. of tag/val reading pairs: "+ repr(ppg_data_length))
    No_points_each_cycle = 3
    No_bytes_each_data = 3
    offset = 0;
    for k in range(0,(ppg_data_length//No_bytes_each_data)//No_points_each_cycle):
        # print("k: ", k)
        ppg_datapoint_time = packet_time + round(k/ppg_sps,TimeSavingPrecision)
        # print("start offest: " ,offset)
        for i in range(0,No_points_each_cycle):
            # print("offset: ", offset)
            block_buf = unpack_from('BBB', SensorData,offset)
            # print(SensorData)
            temp_data:int = block_buf[0] << 16 | block_buf[1] << 8 | block_buf[2]
            data_val:int = temp_data & MAXM86161_REG_FIFO_DATA_MASK
            tag:int = (temp_data >> MAXM86161_REG_FIFO_RES) & MAXM86161_REG_FIFO_TAG_MASK
            # print("tag:value : " + repr(tag)+" : "+ repr(data_val))

            # print(data_val)
            if tag == 1:
                val1 = data_val
            elif tag == 2:
                val2 = data_val
            elif tag == 3:
                val3 = data_val
            else:
                print("No correct tag,  point use previous value................................" )

                break

            offset += No_bytes_each_data

        ppg_ts.append(ppg_datapoint_time)
        ppg_infrared.append(val1)
        ppg_red.append(val2)
        ppg_ambient.append(val3)

        # print("length ppg_ts:", repr(len(ppg_ts)))
        # print("length ppg_green:", repr(len(ppg_green)))
        # print("length ppg_infrared:", repr(len(ppg_infrared)))
        # print("length ppg_red:", repr(len(ppg_red)))

        ppgfile.write(str(ppg_datapoint_time)+\
            ","+ str(val1)+ "," + str(val2) + "," + str(val3) + "\n")
    return


def process(logger_id,SensorData,length):
    switcher = {
        Config_ID: process_config_data,
        IMU_ID: process_imu_data,
        ECG_ID: process_ecg_data,
        GSR_ID: process_gsr_data,
        STRAIN_ID: process_strain_data,
        PULSE_ID: process_pulse_data,
        TEMP_ID: process_temp_data,
        AUdio_ID: process_audio_data,
        Battery_ID: process_battery_data,
        PPG_ID: process_ppg_data,
        PPG_HR_ID: process_ppg_hr_data,
        PPG_SPO2_ID: process_ppg_spo2_data
        }
    func = switcher.get(logger_id, lambda SensorData, length : "Invalid sensor")
    return func(SensorData,length)
    # if logger_id == GSR_ID:
    #     return process_gsr_data(packet_time,stamp_hi, SensorData)

def to_hex(x, pos):
    return '%x' % int(x)


def plot_all_data(imu_ts,gx,gy,gz,ax,ay,az,ppg_ts,ppg_green,ppg_red,ppg_infrared,ppg_ambient,ecg_ts,ecg_ch1,ecg_ch2,battery_ts,battery_percent,battery_level):

    gxarray = np.array(gx, np.int16)/gyro_sensitivity
    gyarray = np.array(gy, np.int16)/gyro_sensitivity
    gzarray = np.array(gz, np.int16)/gyro_sensitivity
    axarray = np.array(ax, np.int16)/acce_sensitivity
    ayarray = np.array(ay, np.int16)/acce_sensitivity
    azarray = np.array(az, np.int16)/acce_sensitivity

    gsrarray = np.array(gsr, np.float64)
    # strainarray = np.array(strain, np.float64)
    # pulsearray = np.array(pulse, np.float64)
    # temperaturearray = np.array(temperature, np.float64)

    audiomeanarray = np.array(audio_mean)
    audiopeakarray = np.array(audio_peak)

    batterypercentarray = np.array(battery_percent)
    batteryvoltarray = np.array(battery_level)

    ecg_ch1_array = np.array(ecg_ch1)#,np.int32)
    ecg_ch2_array = np.array(ecg_ch2)#,np.int32)


    # need improve,  ploting data was appended from file to next file ?
    plt.figure()
    plt.subplot(231)
    plt.plot(imu_ts, gxarray, 'r', imu_ts, gyarray, 'b', imu_ts, gzarray, 'g',linewidth = 0.75)
    plt.legend(['gx', 'gy', 'gz'],loc =1 )
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Vel.[dps]')

    plt.subplot(232)
    plt.plot(imu_ts, axarray, 'r', imu_ts, ayarray, 'b', imu_ts, azarray, 'g')
    plt.legend(['ax', 'ay', 'az'],loc =1)
    plt.xlabel('Time (s)')
    # plt.ylabel('Accelerometer')
    plt.ylabel('Acceleration [g]')

    # plt.subplot(233)
    # plt.plot(ppg_ts, ppg_green, 'g', ppg_ts, ppg_infrared, 'k', ppg_ts, ppg_red,'r')
    # plt.xlabel('Time (s)')
    # plt.ylabel('PPG readings')
    #
    plt.subplot(233)
    plt.plot(ppg_ts, ppg_green, 'g', ppg_ts, ppg_ambient, 'k')
    plt.xlabel('Time (s)')
    plt.ylabel('PPG readings')

    plt.subplot(234)
    plt.plot(ecg_ts, ecg_ch1, 'b' )
    plt.legend(['Respiration'],loc =1)
    plt.xlabel('Time (s)')
    plt.ylabel('Respiration Singal')


    plt.subplot(235)
    plt.plot(ecg_ts, ecg_ch2, 'r')
    plt.legend(['ECG'],loc =1)
    plt.xlabel('Time (s)')
    plt.ylabel('ECG Singal')


    plt.subplot(236)
    plt.plot(battery_ts, batteryvoltarray)
    plt.xlabel('Time (s)')
    plt.ylabel('Battery Voltage (V)')

    plt.show()

# Find out  how many filebasename files there are.
filesearchstring = sourcedir + filebasename + '_'+ '*' + filesuffix
print("Checking existence of " + filesearchstring)
filenames = glob.glob(filesearchstring)
print("Found " + str(len(filenames)) + " Data Files")
if (len(filenames)==0):
    print("Check existence of your SD card and its mounting directory, and make sure sourcedir is correct in this script.")
print(*filenames, sep='\n')
read_offset = 0

for file in filenames:
    total_bytes_skipped = 0
    total_packets = 0

    imu_data_present = 0
    gsr_data_present = 0
    strain_data_present = 0
    pulse_data_present = 0
    temp_data_present = 0
    audio_data_present = 0
    battery_data_present = 0
    ecg_data_present = 0
    ppg_data_present = 0
    ppg_hr_data_present = 0
    ppg_spo2_data_present = 0


    with open(file, "rb") as f:
        print("Processing File ", file)
        clear_data()
        time.sleep(1)

        index_start = file.find(filebasename)
        index_stop = file.find(filesuffix)
        _filenumberstring = file[index_start+4:index_stop]
        textfilenamebase=sourcedir + filebasename + fileprefix +  _filenumberstring

        imufile = open(textfilenamebase+"_imu.csv", "w")
        imufile.write("Time(s),gx(°/s),gy(°/s),gz(°/s),ax(g),ay(g),az(g)\n")

        gsrfile = open(textfilenamebase+"_gsr.csv", "w")
        gsrfile.write("Time(s),Voltage(V)\n")

        strainfile = open(textfilenamebase+"_strain.csv", "w")
        strainfile.write("Time(s),Voltage(V)\n")

        pulsefile = open(textfilenamebase+"_pulse.csv", "w")
        pulsefile.write("Time(s),Voltage(V)\n")

        tempfile = open(textfilenamebase+"_temperature.csv", "w")
        tempfile.write("Time(s),Temperature(oC)\n")

        audiofile = open(textfilenamebase+"_audio.csv", "w")
        audiofile.write("Time(s),audio_mean,audio_peak\n")

        batteryfile = open(textfilenamebase+"_battery.csv", "w")
        batteryfile.write("Time(s),BatteryPercentage(%),Votlage(V)\n")

        ecgfile = open(textfilenamebase+"_ecg.csv", "w")
        ecgfile.write("Time(s),channel1,channel2\n")

        ppgfile = open(textfilenamebase+"_ppg.csv", "w")
        ppgfile.write("Time(s),Green,Infrared,Red\n")
        ppghrfile = open(textfilenamebase+"_ppg_hr.csv", "w")
        ppghrfile.write("Time(s),Green,Ambient\n")
        ppgspo2file = open(textfilenamebase+"_ppg_spo2.csv", "w")
        ppgspo2file.write("Time(s),Infrared,Red,Ambient\n")

        while True:
            SensorData_PacketHeader = bytearray(f.read(PacketHeaderSize))
            read_offset += PacketHeaderSize
            bytes_read = len(SensorData_PacketHeader)

            if (bytes_read < PacketHeaderSize) :
                print("\n\n===============================================")
                print("End of Data file reached with " + repr(bytes_read) + " bytes left over")
                print("Total bytes skipped in data file is " + repr(total_bytes_skipped))
                print("Total packets processed in the file is " + repr(total_packets))

                print("===============================================")

                break

            (finished, skipped, sop, timestamp_hi, timestamp_lo, logger_id, custom, length) = find_sop(SensorData_PacketHeader)

            if (skipped) :
                print("WARNING: Skipped " + repr(skipped) + "bytes to find SOP")

            total_bytes_skipped += skipped

            # try:
            #     sop, timestamp_hi, timestamp_lo, logger_id, length, custom = unpack('<IIIBBH', SensorData_PacketHeader)
            # except:
            #     print("Incomplete header for final entry.")
            #     break
            if finished:
                break
            total_packets += 1

            if sop == 0 and timestamp_hi == 0 and timestamp_lo == 0 and logger_id == 0 and length == 0 and custom == 0:
                print("End of Data...")
                break
            else:
                timstamp = ((timestamp_hi<<32) | timestamp_lo) / Ticks_per_second
                packet_time = round(timstamp,TimeSavingPrecision)
                checkstring = 'StartOfPacket: ' + repr(hex(sop)) + ' MagicNumber: ' + repr(hex(MagicNumber[0]))
                # print(checkstring)
                # print('timestamp (s)        :' + repr(timstamp))
                # print('packet_time (s)      :' + refpr(packet_time))
                # print('logger_id            :' + repr(hex(logger_id)))
                # print('length               :' + repr(length))
                # print('custom               :' + repr(hex(custom)))
                # print('PacketDataHeader     :' + repr(SensorData_PacketHeader))

                align32_length = align32(length)
                # print('aligned length       :' + repr(align32_length))

                if (logger_id == PPG_ID) or (logger_id == PPG_HR_ID) or (logger_id == PPG_SPO2_ID) :
                    align32_length = length  # ppg data was not aligned to 4 bytes

                try:
                    SensorData = f.read(align32_length)
                    read_offset += align32_length
                except:
                    print("Not enough data in file for last entry.")
                    break
                # print('SensorData     :' + repr(SensorData))
                # print('Payload:')
                # for i in range(length):
                #     print("     ", i, " : ", repr(hex(SensorData[i])))
                #assert sop == MagicNumber[0]

                process(logger_id,SensorData,length)

                if logger_id == IMU_ID:
                    imu_data_present = 1
                elif  logger_id == GSR_ID:
                    gsr_data_present = 1
                elif  logger_id == STRAIN_ID:
                    strain_data_present = 1
                elif  logger_id == PULSE_ID:
                    pulse_data_present = 1
                elif  logger_id == TEMP_ID:
                    temp_data_present = 1
                elif  logger_id == AUdio_ID:
                    audio_data_present = 1
                elif  logger_id == Battery_ID:
                    battery_data_present = 1
                elif  logger_id == ECG_ID:
                    ecg_data_present = 1
                elif logger_id == PPG_ID :
                    ppg_data_present = 1
                elif logger_id == PPG_HR_ID:
                    ppg_hr_data_present = 1
                elif logger_id == PPG_SPO2_ID:
                    ppg_spo2_data_present = 1


        imufile.close()
        gsrfile.close()
        strainfile.close()
        pulsefile.close()
        tempfile .close()
        audiofile.close()
        batteryfile.close()
        ecgfile.close()
        ppgfile.close()
        ppghrfile.close()
        ppgspo2file.close()
        f.close()

        # remove unsued file because no corresponding data
        if imu_data_present == 0:
            os.remove(textfilenamebase+"_imu.csv")
        if gsr_data_present == 0:
            os.remove(textfilenamebase+"_gsr.csv")
        if strain_data_present == 0:
            os.remove(textfilenamebase+"_strain.csv")
        if pulse_data_present == 0:
            os.remove(textfilenamebase+"_pulse.csv")
        if temp_data_present == 0:
            os.remove(textfilenamebase+"_temperature.csv")
        if audio_data_present == 0:
            os.remove(textfilenamebase+"_audio.csv")
        if battery_data_present == 0:
            os.remove(textfilenamebase+"_battery.csv")
        if ecg_data_present == 0:
            os.remove(textfilenamebase+"_ecg.csv")
        if ppg_data_present   == 0:
            os.remove(textfilenamebase+"_ppg.csv")
        if ppg_hr_data_present   == 0:
            os.remove(textfilenamebase+"_ppg_hr.csv")
        if ppg_spo2_data_present   == 0:
            os.remove(textfilenamebase+"_ppg_spo2.csv")

        print("Finished Processing File: ", file)
        print("Firmware Version: ", FirmwareVersion)
        plot_all_data(imu_ts,gx,gy,gz,ax,ay,az,ppg_ts,ppg_green,ppg_red,ppg_infrared,ppg_ambient,ecg_ts,ecg_ch1,ecg_ch2,battery_ts,battery_percent,battery_level)
