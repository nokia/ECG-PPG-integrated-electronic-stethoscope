/*
 * © 2022 Nokia
 * Licensed under the BSD 3-Clause License
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdint.h> 
#include <string.h>

#include "sdk_config.h" 
#include "BMI160_if.h"
#include "maxm86161.h"
#include "nrf_drv_spi.h"
#include "patchkeeper_saadc.h"
#include "ff.h"
#include "diskio_blkdev.h"


// Need to define this type early as it is used to size the buffers.

//typedef struct {
//  uint32_t          ts_lo;
//  nrf_saadc_value_t value;
//  uint16_t          custom; // Word Aligns Packet - TODO: is this really needed?
//} adc_sensor_data_buffer_t;

typedef struct {
  uint32_t          ts_lo;
  float_t           value;
} adc_sensor_data_buffer_t; // 8 bytes

typedef uint8_t batt_data_buffer_t;

/*************************************
 ** @PATCHKEEPER Macros             **
 *************************************/
//#define CONFIG_DATA_VERSION             2 //Enable realtime set by ble master
#define CONFIG_DATA_VERSION             3 //Increase Timestamp Bitwidth
#define APP_BLE_CONN_CFG_TAG            1  /**< A tag identifying the SoftDevice BLE configuration. */

// define firmware version number
#define FIRMWARE_MAJOR_VERSION          3
#define FIRMWARE_MINOR_VERSION          1  // PPG alwasy on, 4G IMU range
#define FIRMWARE_PATCH_NUM              0  // 1:  for animal activities: no ECG, PPG always on
#define FIRMWARE_BUILD_NUM              1 


// FIRMWARE_BUILD_NUM  0: ppg sps = 50.027
// FIRMWARE_BUILD_NUM  1: ppg sps = 99.902 (defalut)
// FIRMWARE_BUILD_NUM  2: ppg sps = 199.805
// FIRMWARE_BUILD_NUM  other: ppg sps = 99.902 (default)


// ADS1292 Macros.
//COMMAND                  DESCRIPTION FIRST BYTE SECOND BYTE
//System Commands
#define ADS1292R_CMD_WAKEUP    0x02  // Wake-up from standby mode 0000 0010 (02h)
#define ADS1292R_CMD_STANDBY   0x04  // Enter standby mode 0000 0100 (04h)
#define ADS1292R_CMD_RESET     0x06  // Reset the device 0000 0110 (06h)
#define ADS1292R_CMD_START     0x08  // Start or restart (synchronize) conversions 0000 1000 (08h)
#define ADS1292R_CMD_STOP      0x0a  // Stop conversion 0000 1010 (0Ah)
#define ADS1292R_CMD_OFFSETCAL 0x1a  // Channel offset calibration 0001 1010 (1Ah)

//Data Read Commands
#define ADS1292R_CMD_RDATAC    0x10  // Enable Read Data Continuous mode. 0001 0000 (10h) This mode is the default mode at power-up.(1)
#define ADS1292R_CMD_SDATAC    0x11  // Stop Read Data Continuously mode 0001 0001 (11h)
#define ADS1292R_CMD_RDATA     0x12  // Read data by command; supports multiple read back. 0001 0010 (12h)

//Register Read Commands
#define ADS1292R_CMD_RREG      0x20  // Read n nnnn registers starting at address r rrrr 001r rrrr (2xh)(2) 000n nnnn(2)
#define ADS1292R_CMD_WREG      0x40  // Write n nnnn registers starting at address r rrrr 010r rrrr (4xh)(2) 000n nnnn(2)


#define ADS1292R_REG_ID          0x00 
// Global Settings Across Channels
#define ADS1292R_REG_CONFIG1     0x01
#define ADS1292R_REG_CONFIG2     0x02
#define ADS1292R_REG_LOFF        0x03
// Channel-Specific Settings
#define ADS1292R_REG_CH1SET      0x04
#define ADS1292R_REG_CH2SET      0x05
#define ADS1292R_REG_RLD_SENS    0x06
#define ADS1292R_REG_LOFF_SENS   0x07
#define ADS1292R_REG_LOFF_STAT   0x08
// GPIO and Other Registers
#define ADS1292R_REG_RESP1       0x09
#define ADS1292R_REG_RESP2       0x0A
#define ADS1292R_REG_GPIO        0x0B


#define SOP_BELLLABS 0x42656c6c


#define PATCHKEEPER_IMU_DATA_LENGTH_BYTES 16 
#define PATCHKEEPER_IMU_DATA_LENGTH_HALF_WORDS PATCHKEEPER_IMU_DATA_LENGTH_BYTES/2 
#define PATCHKEEPER_IMU_ID  0x2a 


#define PATCHKEEPER_ECG_TEST_MASK_BIT       (1 << 0)
#define PATCHKEEPER_GSR_TEST_MASK_BIT       (1 << 1)
#define PATCHKEEPER_STRAIN_TEST_MASK_BIT    (1 << 2)
#define PATCHKEEPER_PULSE_TEST_MASK_BIT     (1 << 3)
#define PATCHKEEPER_TEMP_TEST_MASK_BIT      (1 << 4)
#define PATCHKEEPER_IMU_TEST_MASK_BIT       (1 << 5)

#define PATCHKEEPER_ANALOG_TEST_MASK_BITS   PATCHKEEPER_GSR_TEST_MASK_BIT || PATCHKEEPER_STRAIN_TEST_MASK_BIT || PATCHKEEPER_PULSE_TEST_MASK_BIT || PATCHKEEPER_TEMP_TEST_MASK_BIT


#define PATCHKEEPER_NUM_AGGREGATED_ADS1292_PACKET    19 //(244-16)/12=19, use 12 bytes for struct data alignment
#define PATCHKEEPER_ECG_DATA_LENGTH PATCHKEEPER_NUM_AGGREGATED_ADS1292_PACKET*sizeof(ads1292r_packet_t)  //19x12
#define PATCHKEEPER_ECG_ID                      0x20 


#define PATCHKEEPER_BATT_BUFFER_SIZE            1 // multiple of this not implemented
#define PATCHKEEPER_BATT_DATA_LENGTH_BYTES      PATCHKEEPER_BATT_BUFFER_SIZE * sizeof(batt_data_buffer_t)  //1*1?
#define PATCHKEEPER_BATT_ID                     0x05


#define PATCHKEEPER_GSR_BUFFER_SIZE             2
#define PATCHKEEPER_GSR_DATA_LENGTH_BYTES       PATCHKEEPER_GSR_BUFFER_SIZE * sizeof(adc_sensor_data_buffer_t)  //2x8
#define PATCHKEEPER_GSR_ID                      0x22

#define PATCHKEEPER_STRAIN_BUFFER_SIZE          2
#define PATCHKEEPER_STRAIN_DATA_LENGTH_BYTES    PATCHKEEPER_STRAIN_BUFFER_SIZE * sizeof(adc_sensor_data_buffer_t)  //2x8
#define PATCHKEEPER_STRAIN_ID                   0x24

#define PATCHKEEPER_PULSE_BUFFER_SIZE           2
#define PATCHKEEPER_PULSE_DATA_LENGTH_BYTES     PATCHKEEPER_PULSE_BUFFER_SIZE * sizeof(adc_sensor_data_buffer_t)  //2x8
#define PATCHKEEPER_PULSE_ID                    0x26

#define PATCHKEEPER_TEMP_BUFFER_SIZE            2
#define PATCHKEEPER_TEMP_DATA_LENGTH_BYTES      PATCHKEEPER_TEMP_BUFFER_SIZE * sizeof(adc_sensor_data_buffer_t)  //2x8
#define PATCHKEEPER_TEMP_ID                     0x28

#define PATCHKEEPER_PPG_ID                      0x2B
#define PATCHKEEPER_PPG_HR_ID                   0x2C
#define PATCHKEEPER_PPG_SPO2_ID                 0x2D

#define PATCHKEEPER_DEFAULT_ECG_MODE              STREAMING
#define PATCHKEEPER_DEFAULT_GSR_MODE              STREAMING
#define PATCHKEEPER_DEFAULT_STRAIN_MODE           STREAMING
#define PATCHKEEPER_DEFAULT_PULSE_MODE            STREAMING
#define PATCHKEEPER_DEFAULT_TEMP_MODE             STREAMING
#define PATCHKEEPER_DEFAULT_BATTERY_MODE          STREAMING
#define PATCHKEEPER_DEFAULT_IMU_MODE              STREAMING
#define PATCHKEEPER_DEFAULT_PPG_MODE              STREAMING

#define PATCHKEEPER_DEFAULT_IMU_SAMPLE_INTERVAL_MS        20  //50
#define PATCHKEEPER_DEFAULT_AUDIO_SAMPLE_INTERVAL_MS      1000 
#define PATCHKEEPER_DEFAULT_BATTERY_SAMPLE_INTERVAL_MS    10000

#define PATCHKEEPER_DEFAULT_GSR_SAMPLE_INTERVAL_MS        50
#define PATCHKEEPER_DEFAULT_PULSE_SAMPLE_INTERVAL_MS      50
#define PATCHKEEPER_DEFAULT_STRAIN_SAMPLE_INTERVAL_MS     50
#define PATCHKEEPER_DEFAULT_TEMP_SAMPLE_INTERVAL_MS       15000  // changed from 1s to 15s

#if BOARD_HAS_ANA_SENSOR == 1 
#define PATCHKEEPER_DEFAULT_ADC_SAMPLE_INTERVAL_MS        5000  //changed to 5s as only for slow Temperature sensor
#else
#define PATCHKEEPER_DEFAULT_ADC_SAMPLE_INTERVAL_MS         5000
#endif

#define PATCHKEEPER_DEFAULT_ECG_SAMPLE_INTERVAL_MS        8 //this is not used for now

//#define PATCHKEEPER_DEFAULT_PPG_SAMPLE_INTERVAL_MS        50 //ppg sampling rate is set in: default_maxim_config

#define PATCHKEEPER_DEFAULT_PPG_TIMER_MS                    5000  //5 seconds, smallest timer settings for on/off control
#define PATCHKEEPER_DEFAULT_PPG_ON_COUNTS_OF_TIMER          15 //6  //30 seconds
#define PATCHKEEPER_DEFAULT_PPG_DUTYCYCLE_COUNTS_OF_TIMER   12 //60 //5 mins


#define MEMORY_SIZE_KB(n) (n) * 1024

//#if BOARD_HAS_PDM_MIC == 1

typedef int16_t audio_memory_t;

#define PATCHKEEPER_AUDIO_ID  0x04
#define PATCHKEEPER_AUDIO_RESULT_BUFFER_SIZE          1
#define PATCHKEEPER_AUDIO_RESULT_DATA_LENGTH_BYTES    PATCHKEEPER_AUDIO_RESULT_BUFFER_SIZE*2*sizeof(audio_memory_t)

// PDM, once it has a buffer, will continue until it has filled the buffer. 
// To limit this to 500ms, the buffer needs to hold only 500ms worth of mono
// audio data at 16.125kHz i.e. 8062 samples at 16 bit/sample -> 16124 
//#define AUDIO_MEMORY_SIZE_BYTES         MEMORY_SIZE_KB(32) // we need two buffers for continuous saving

// The above 32K memory size was for 500ms worth of mono
//For 500ms worth of stereo of Patchkeeper V2.0, we need 64k memory
#define AUDIO_MEMORY_SIZE_BYTES         MEMORY_SIZE_KB(64) 


#define AUDIO_MEMORY_NUM_BUFFERS        2  //we still use two buffers in alternatvie mode  for V2.0 
#define AUDIO_MEMORY_BUFFER_SIZE        AUDIO_MEMORY_SIZE_BYTES/AUDIO_MEMORY_NUM_BUFFERS  //this is doubled for V2.0 as well
static audio_memory_t   audiodata_memory[AUDIO_MEMORY_NUM_BUFFERS] [AUDIO_MEMORY_BUFFER_SIZE/sizeof(audio_memory_t)]; 


#define LOGGER_CONFIG_DATA_LENGTH_BYTES 32
#define LOGGER_CONFIG_ID           0x00
#define DATALOG_MEMORY_SIZE_BYTES MEMORY_SIZE_KB(8)
typedef uint8_t datalog_memory_t; 

//#endif // BOARD_HAS_PDM_MIC  

/*************************************
 ** @PATCHKEEPER Types              **
 *************************************/
typedef enum {TEMPERATURE, GSR, STRAIN, PULSE} ina338_channel_t;
typedef ina338_channel_t test_t;

typedef struct packet_info_struct { 
    uint32_t  sop; 
    uint32_t  timestamp_hi; // If we represent this as 64-bit we have alignment issues with padding added
    uint32_t  timestamp_lo; // So send as two 32-bit numbers and put together in the app. 
    uint8_t   logger_id; 
    uint8_t   custom;     // Makes the structure end 32-bit aligned.length; 
    uint16_t  length;     
} packet_info_t; 
 
typedef struct { 
    packet_info_t   packet_info; 
    int16_t         data[PATCHKEEPER_IMU_DATA_LENGTH_HALF_WORDS]; // 4*8 bytes
} imu_data_t; 

// The SPIM driver does not allow us to offset the incoming read from a transfer
// So when we read the registers, the first two bytes need to be discarded. 
// Simplest way to deal with this is to use a structure that contains the entries 
// for the extra dummy reads.
typedef struct {
  uint8_t   dummy_read_0;
  uint8_t   dummy_read_1;
  uint8_t   id;
  uint8_t   config1;
  uint8_t   config2;
  uint8_t   loff;
  uint8_t   ch1set;
  uint8_t   ch2set;
  uint8_t   rld_sens;
  uint8_t   loff_sens;
  uint8_t   loff_stat;
  uint8_t   resp1;
  uint8_t   resp2;
  uint8_t   gpio;
} ads1292_registers_t;

// Read Data from SPI
typedef struct {
  uint8_t status[3];
  uint8_t data0[3]; 
  uint8_t data1[3];
} ads1292_rdata_t;

typedef struct { 
    uint32_t    timestamp_lo;
    uint8_t     status;
    uint8_t     packet_count;  //for memory allocation alignment
    uint8_t     channel1_data[3];
    uint8_t     channel2_data[3];
} ads1292r_packet_t; //12 bytes

typedef struct { 
    packet_info_t                   packet_info; 
    ads1292r_packet_t               ads1992_packet[PATCHKEEPER_NUM_AGGREGATED_ADS1292_PACKET]; 
} ecg_data_t; 

typedef struct {  
    packet_info_t                   packet_info; 
    nrf_saadc_value_t               data[PATCHKEEPER_GSR_DATA_LENGTH_BYTES]; 
} gsr_data_t; 

typedef struct {  
    packet_info_t                   packet_info; 
    nrf_saadc_value_t               data[PATCHKEEPER_STRAIN_DATA_LENGTH_BYTES]; 
} strain_data_t; 
 
typedef struct {  
    packet_info_t                   packet_info; 
    nrf_saadc_value_t               data[PATCHKEEPER_PULSE_DATA_LENGTH_BYTES];  
} pulse_data_t; 

typedef struct {  
    packet_info_t                   packet_info; 
    nrf_saadc_value_t               data[PATCHKEEPER_TEMP_DATA_LENGTH_BYTES]; 
} temp_data_t; 

typedef struct {
    packet_info_t                   packet_info;
    audio_memory_t                  data[PATCHKEEPER_AUDIO_RESULT_DATA_LENGTH_BYTES];
} audio_data_t;

typedef struct { 
    ecg_data_t     ecg_data; 
    gsr_data_t     gsr_data; 
    temp_data_t    temp_data; 
    strain_data_t  strain_data; 
    pulse_data_t   pulse_data;
} patchkeeper_data_t; 

typedef enum {OFF, STREAMING, ON_DEVICE_PROCESSING} ble_mode_t;
typedef uint32_t sample_interval_ms_t;

typedef struct {
  ble_mode_t              mode;
  sample_interval_ms_t  sample_interval_ms;
} sensor_config_t;

typedef struct {
  sensor_config_t ecg_sensor;
  sensor_config_t gsr_sensor;
  sensor_config_t strain_sensor;
  sensor_config_t pulse_sensor;
  sensor_config_t temp_sensor;
  sensor_config_t imu_sensor;
  sensor_config_t battery;
} patchkeeper_sensor_config_t;

typedef uint32_t (*ble_send_data_t)(uint8_t *start_ptr, uint8_t *end_ptr);

typedef void (*ble_start_advertising_t)(bool);

typedef struct {
    packet_info_t   packet_info;
    int8_t          data[LOGGER_CONFIG_DATA_LENGTH_BYTES];
} config_data_t;

 typedef struct {
  ble_send_data_t           ble_send_data;
  ble_start_advertising_t   ble_advertising_start;
} patchkeeper_init_context_t;

typedef enum {UNINITIALISED, IDLE, RUNNING} m_sensor_timer_state_t;

typedef struct { 
    packet_info_t             packet_info; 
    adc_sensor_data_buffer_t  data[PATCHKEEPER_GSR_BUFFER_SIZE]; 
} patchkeeper_gsr_data_t; 

typedef struct { 
    packet_info_t             packet_info; 
    adc_sensor_data_buffer_t  data[PATCHKEEPER_STRAIN_BUFFER_SIZE]; 
} patchkeeper_strain_data_t; 

typedef struct { 
    packet_info_t             packet_info; 
    adc_sensor_data_buffer_t  data[PATCHKEEPER_PULSE_BUFFER_SIZE]; 
} patchkeeper_pulse_data_t; 

typedef struct { 
    packet_info_t             packet_info; 
    adc_sensor_data_buffer_t  data[PATCHKEEPER_TEMP_BUFFER_SIZE]; 
} patchkeeper_temp_data_t; 

typedef struct { 
    packet_info_t             packet_info; 
    batt_data_buffer_t       data[PATCHKEEPER_BATT_BUFFER_SIZE]; 
} patchkeeper_batt_data_t; 


/*************************************
 ** @PATCHKEEPER Functions          **
 *************************************/
uint8_t patchkeeper_init(patchkeeper_init_context_t);
void patchkeeper_spi_handler (nrf_drv_spi_evt_t const* , void*);
uint8_t patchkeeper_spi_init(void);

extern void config_datalogger_handler(void);
void Swap_and_Save_Buffer(void);

void patchkeeper_config_ecg(uint8_t mode, uint8_t sample_interval_ms);
void patchkeeper_config_gsr(uint8_t mode, uint8_t sample_interval_ms);
void patchkeeper_config_strain(uint8_t mode, uint8_t sample_interval_ms);
void patchkeeper_config_pulse(uint8_t mode, uint8_t sample_interval_ms);
void patchkeeper_config_temp(uint8_t mode, uint8_t sample_interval_ms);
void patchkeeper_config_imu(uint8_t mode, uint8_t sample_interval_ms);

void patchkeeper_imu_datalogger_handler (void*);
//void patchkeeper_gsr (void*);

void patchkeeper_ads1292_init(void);
void patchkeeper_ads1292_interrupt_init(void);

void patchkeeper_ads1292_standby(void);
void patchkeeper_ads1292_powerdown(void);

void patchkeeper_ads1292_spi_cmd(uint8_t command);
void patchkeeper_ads1292_spi_regr(ads1292_registers_t* p_registers);
void patchkeeper_ads1292_spi_regw(ads1292_registers_t* p_registers);

void patchkeeper_ads1292_rdatac (void);
void patchkeeper_ads1292_drdy_handler(nrf_drv_gpiote_pin_t, nrf_gpiote_polarity_t);
void patchkeeper_ecg_send_data(void);
void patchkeeper_ecg_stop(void);
void patchkeeper_ecg_trigger(void);

static bool patchkeeper_ppg_maxm86161_init(void);

void  disable_INA338_all(void);
void  enable_INA338_channel(ina338_channel_t);
void  patchkeeper_enable_INA338_all(void);

void patchkeeper_read_temperature(void*);

void patchkeeper_adc_init(void);
void patchkeeper_start_sampling_adc(void);
void patchkeeper_stop_sampling_adc(void);

uint32_t read_temp_vds(void);
uint32_t read_gsr_vds(void);
uint32_t read_strain_vds(void);
uint32_t read_pulse_vds(void);

// IMU Driver
void patchkeeper_imu_init(void);
void patchkeeper_imu_trigger(void);
void patchkeeper_imu_isr(nrf_drv_gpiote_pin_t, nrf_gpiote_polarity_t);
void patchkeeper_imu_timer_init (void);
void patchkeeper_imu_timer_trigger (void);

// GSR Driver
void patchkeeper_gsr_init (void);
void patchkeeper_gsr_trigger (void);
void patchkeeper_gsr_isr(void*);

// Strain Driver
void patchkeeper_strain_init (void);
void patchkeeper_strain_trigger (void);
void patchkeeper_strain_isr(void*);

// Pulse Driver
void patchkeeper_pulse_init (void);
void patchkeeper_pulse_trigger (void);
void patchkeeper_pulse_isr(void*);

// Temp Driver
void patchkeeper_temp_init (void);
void patchkeeper_temp_trigger (void);
void patchkeeper_temp_isr(void*);

// Batt Driver
void patchkeeper_batt_init (void);
void patchkeeper_batt_trigger (void);
void patchkeeper_batt_isr(void*);
float patchkeeper_convert_adc_voltage_to_vbatt (float);

#if BOARD_HAS_PDM_MIC==1 
void pdm_init();
void pdm_start();
void patchkeeper_audio_handler (nrfx_pdm_evt_t const * const p_evt); 
//void patchkeeper_audio_pdm_ctrl(uint8_t); 
void patchkeeper_audio_timer_config (void);
void patchkeeper_audio_timer_handle(void*);

#endif // BOARD_HAS_PDM_MIC 

#if USE_SDCARD == 1
#include "nrf_block_dev_sdc.h"
#include "app_sdcard.h"


#define SDC_SECTOR_SIZE             512     ///< Size of a single SD card block in bytes.


#define FILENAME_BASENAME "DLOG_"
#define FILENAME_BASENAME_AUDIO "ALOG_"
#define FILENAME_SUFFIX "bin"
#define MAX_FILESIZE_4GB 0xffffffff /* Corresponds to 4GB-1 */
#define MAX_FILESIZE_2GB   0x80000000 /* Corresponds to 2GB   */
#define MAX_FILESIZE_128MB 0x08000000 /* Corresponds to 128MB   */
#define MAX_FILESIZE_256MB 0x10000000 /* Corresponds to 256MB   */
#define MAX_FILESIZE_512MB 0x20000000 /* Corresponds to 512MB   */
#define MAX_FILESIZE_1GB   0x40000000 /* Corresponds to 1024MB   */

#define MAX_FILESIZE_512KB 0x00080000 /* for testing*/
//#define MAX_FILESIZE MAX_FILESIZE_512KB

#define MAX_FILESIZE MAX_FILESIZE_512MB
//#define MAX_FILESIZE MAX_FILESIZE_128MB


extern uint32_t m_last_reset_reason;
enum reset_type { PowerOnReset=0,
                  ResetPin=1,
                  WatchdogReset=2,
                  SoftwareReset=4,
                  CPULockupReset=8,
                  GPIOWakeUp=0x10000,
                  LPCompWakeUp=0x20000,
                  DebugWakeUp=0x40000,
                  NFCWakeUp=0x80000
                  };;

/** 
 * @brief  SDC block device definition 
 * */ 
NRF_BLOCK_DEV_SDC_DEFINE( 
        m_block_dev_sdc, 
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE, 
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN) 
         ), 
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00") 
); 


static int Open_SDCard();
static int List_SDCard();

extern void error_led_flash_loop(void);
static void datalogger_wait_func(void);
void Swap_and_Save_Buffer(void);
void Save_Audio_Buffer(audio_memory_t *);
static FRESULT sdcard_write(datalog_memory_t *, datalog_memory_t *);
static FRESULT sdcard_write_audio_trace(datalog_memory_t *, datalog_memory_t *);
//static FRESULT sdcard_write_audio_trace_small_files(datalog_memory_t *, datalog_memory_t *);

#endif

#if BOARD_HAS_PPG_SENSOR == 1

//#define PATCHKEEPER_CONFIG_PPG     0x2b

typedef struct { 
    packet_info_t             packet_info; 
    maxm86161_ppg_sample_t    data; 
} patchkeeper_ppg_sample_t; 

typedef struct { 
    packet_info_t             packet_info; 
    maxm86161_ppg_data_t      data[128]; 
} patchkeeper_ppg_data_t; 

typedef struct { 
    packet_info_t             packet_info; // 16 bytes
    uint8_t                   data[3*128]; 
} patchkeeper_ppg_bytes_t; // 400 bytes


static maxm86161_device_config_t default_maxim_config = {
    60,   //(max 128) number of data points available before interrupt
// 128-this numbr 60 = how many new samples can be written to the FIFO before the interrupt is asserted.
    {   //LED sequence config: maxm86161_ledsq_cfg_t
#if (PROX_SELECTION & PROX_USE_IR)
        0x02,//LED2 - IR
        0x01,//LED1 - green
        0x03,//LED3 - RED
#elif (PROX_SELECTION & PROX_USE_RED)
        0x03,//LED3 - RED
        0x02,//LED2 - IR
        0x01,//LED1 - green
#else // default use GREEN
        0x01,//LED1 - green
        0x02,//LED2 - IR
        0x03,//LED3 - RED
#endif
        0x00, 
        0x00,
        0x00,
    },
    {  // LED drive current range config: 31, 63,92,124mA for 00,01,10,11
        0x02,
        0x01,
        0x01
    },
        { // LED brightness config, drive current
        0x20,// green
        0x15,// IR
        0x10,// LED
    },  
    { // PPG sensor configure: maxm86161_ppg_cfg_t
        MAXM86161_PPG_CFG_ALC_DISABLE,                 //< Enable/disable ambient light correction ALC
        MAXM86161_PPG_CFG_OFFSET_NO,              //< Enable/disable dark current measurement
        MAXM86161_PPG_CFG_TINT_117p3_US,          //< Set the pulse width of the LED drivers and the integration time of PPG ADC
        MAXM86161_PPG_CFG_ADC_RANGE_16k,          //< Set the ADC range of the sensor
        MAXM86161_PPG_CFG_SMP_AVG_1,              //< Set the number of exposure per sample
        MAXM86161_PPG_CFG_SMP_RATE_P1_50sps,       //< Set the effective sampling rate of the PPG sensor
         // change sample rate need to change FIRMWARE_BUILD_NUM (50sps: 0) for correct python script data parsing
        MAXM86161_PPG_LED_SETLNG_12us,            //< Set LED settling time
        MAXM86161_PPG_DIGIFILT_CDM,               //< ppg DIG_FILT_SEL 
    },
    {  // interupt configure: maxm86161_int_t
        MAXM86161_INT_ENABLE,//full_fifo
        MAXM86161_INT_DISABLE,//data_rdy
        MAXM86161_INT_DISABLE,//alc_ovf
#ifdef PROXIMITY
        MAXM86161_INT_ENABLE,//proximity
#else
        MAXM86161_INT_DISABLE,
#endif
        MAXM86161_INT_DISABLE,//led_compliant
        MAXM86161_INT_DISABLE,//die_temp
        MAXM86161_INT_DISABLE,//pwr_rdy
        MAXM86161_INT_DISABLE,//sha
    }
};


static maxm86161_device_config_t maxim_config_3leds_99sps = {
    60,   //(max 128)how many new samples can be written to the FIFO before the interrupt is asserted.)
    {   //LED sequence config: maxm86161_ledsq_cfg_t
#if (PROX_SELECTION & PROX_USE_IR)
        0x02,//LED2 - IR
        0x01,//LED1 - green
        0x03,//LED3 - RED
#elif (PROX_SELECTION & PROX_USE_RED)
        0x03,//LED3 - RED
        0x02,//LED2 - IR
        0x01,//LED1 - green
#else // default use GREEN
        0x01,//LED1 - green
        0x02,//LED2 - IR
        0x03,//LED3 - RED
#endif
        //0x09, // DIRECT AMBIENT exposure
        0x00,
        0x00,
        0x00,
    },
    {  // LED drive current range config: 31, 63,92,124mA for binary 00,01,10,11
        0x02,
        0x01,
        0x01,
    },
    { // LED brightness config, drive current
        0x20,// green
        0x15,// IR
        0x10,// LED
    },
    { // PPG sensor configure: maxm86161_ppg_cfg_t
        MAXM86161_PPG_CFG_ALC_ENABLE,               //< Enable/disable ambient light correction ALC
        MAXM86161_PPG_CFG_OFFSET_NO,            //< Enable/disable dark current measurement
        MAXM86161_PPG_CFG_TINT_117p3_US,        //< Set the pulse width of the LED drivers and the integration time of PPG ADC
        MAXM86161_PPG_CFG_ADC_RANGE_16k,        //< Set the ADC range of the sensor
        MAXM86161_PPG_CFG_SMP_AVG_1,            //< Set the number of exposure per sample
        MAXM86161_PPG_CFG_SMP_RATE_P1_99sps,   //< Set the effective sampling rate of the PPG sensor
         // change sample rate need to change FIRMWARE_BUILD_NUM (99sps: 1) for correct python script data parsing
        MAXM86161_PPG_LED_SETLNG_12us,            //< Set LED settling time
        MAXM86161_PPG_DIGIFILT_CDM,               //< ppg DIG_FILT_SEL 
    },
    {  // interupt configure: maxm86161_int_t
        MAXM86161_INT_ENABLE,//full_fifo
        MAXM86161_INT_DISABLE,//data_rdy
        MAXM86161_INT_DISABLE,//alc_ovf
#ifdef PROXIMITY
        MAXM86161_INT_ENABLE,//proximity
#else
        MAXM86161_INT_DISABLE,
#endif
        MAXM86161_INT_DISABLE,//led_compliant
        MAXM86161_INT_DISABLE,//die_temp
        MAXM86161_INT_DISABLE,//pwr_rdy
        MAXM86161_INT_DISABLE,//sha
    }
};

static maxm86161_device_config_t default_spo2_config = {
    60,   //(max 128)
    {   //LED sequence config: maxm86161_ledsq_cfg_t
#if (PROX_SELECTION & PROX_USE_IR)
        0x02,//LED2 - IR
        0x01,//LED1 - green
        0x03,//LED3 - RED
#elif (PROX_SELECTION & PROX_USE_RED)
        0x03,//LED3 - RED
        0x02,//LED2 - IR
        0x01,//LED1 - green
#else // default use GREEN
        0x02,//LED2 - IR
        0x01,//LED1 - RED
        0x09,// DIRECT AMBIENT exposure
#endif
        0x00,
        0x00,
        0x00,
    },
    {  // LED drive current range config: 31, 63,92,124mA for binary 00,01,10,11
        0x02,
        0x01,
        0x01,
    },
    { // LED brightness config, drive current
        0x20,// green
        0x15,// IR
        0x10,// LED
    },
    { // PPG sensor configure: maxm86161_ppg_cfg_t
        MAXM86161_PPG_CFG_ALC_ENABLE,               //< Enable/disable ambient light correction ALC
        MAXM86161_PPG_CFG_OFFSET_NO,            //< Enable/disable dark current measurement
        MAXM86161_PPG_CFG_TINT_117p3_US,        //< Set the pulse width of the LED drivers and the integration time of PPG ADC
        MAXM86161_PPG_CFG_ADC_RANGE_16k,        //< Set the ADC range of the sensor
        MAXM86161_PPG_CFG_SMP_AVG_1,            //< Set the number of exposure per sample
        MAXM86161_PPG_CFG_SMP_RATE_P1_50sps,   //< Set the effective sampling rate of the PPG sensor
        // 50sps : FIRMWARE_BUILD_NUM =0 for correct python script data parsing
        MAXM86161_PPG_LED_SETLNG_12us,            //< Set LED settling time
        MAXM86161_PPG_DIGIFILT_CDM,               //< ppg DIG_FILT_SEL 
     },
    {  // interupt configure: maxm86161_int_t
        MAXM86161_INT_ENABLE,//full_fifo
        MAXM86161_INT_DISABLE,//data_rdy
        MAXM86161_INT_DISABLE,//alc_ovf
#ifdef PROXIMITY
        MAXM86161_INT_ENABLE,//proximity
#else
        MAXM86161_INT_DISABLE,
#endif
        MAXM86161_INT_DISABLE,//led_compliant
        MAXM86161_INT_DISABLE,//die_temp
        MAXM86161_INT_DISABLE,//pwr_rdy
        MAXM86161_INT_DISABLE//sha
    }
};


static maxm86161_device_config_t default_green_led_config = {
    60,   //(max 128) number of data points available before interrupt
// 128-this numbr = how many new samples can be written to the FIFO before the interrupt is asserted.
    {   //LED sequence config: maxm86161_ledsq_cfg_t
#if (PROX_SELECTION & PROX_USE_IR)
        0x02,//LED2 - IR
        0x01,//LED1 - green
        0x03,//LED3 - RED
#elif (PROX_SELECTION & PROX_USE_RED)
        0x03,//LED3 - RED
        0x02,//LED2 - IR
        0x01,//LED1 - green
#else // default use GREEN
        0x01,//LED1 - green
        0x09,//direct ambient light 
        0x00,
#endif
        0x00, 
        0x00,
        0x00,
    },
    {  // LED drive current range config: 31, 63,92,124mA for 00,01,10,11
        0x02,
        0x01,
        0x01
    },
        { // LED brightness config, drive current
        0x20,// green
        0x15,// IR
        0x10,// LED
    },  
    { // PPG sensor configure: maxm86161_ppg_cfg_t
        MAXM86161_PPG_CFG_ALC_ENABLE,             //< Enable/disable ambient light correction ALC
        MAXM86161_PPG_CFG_OFFSET_NO,              //< Enable/disable dark current measurement
        MAXM86161_PPG_CFG_TINT_117p3_US,          //< Set the pulse width of the LED drivers and the integration time of PPG ADC
        MAXM86161_PPG_CFG_ADC_RANGE_16k,          //< Set the ADC range of the sensor
        MAXM86161_PPG_CFG_SMP_AVG_1,              //< Set the number of exposure per sample
        MAXM86161_PPG_CFG_SMP_RATE_P1_99sps,       //< Set the effective sampling rate of the PPG sensor
        // change sample rate need to change FIRMWARE_BUILD_NUM (99sps: 1) for correct python script data parsing
        MAXM86161_PPG_LED_SETLNG_12us,            //< Set LED settling time
        MAXM86161_PPG_DIGIFILT_FDM,               //< ppg DIG_FILT_SEL 
    },
    {  // interupt configure: maxm86161_int_t
        MAXM86161_INT_ENABLE,//full_fifo
        MAXM86161_INT_DISABLE,//data_rdy
        MAXM86161_INT_DISABLE,//alc_ovf
#ifdef PROXIMITY
        MAXM86161_INT_ENABLE,//proximity
#else
        MAXM86161_INT_DISABLE,
#endif
        MAXM86161_INT_DISABLE,//led_compliant
        MAXM86161_INT_DISABLE,//die_temp
        MAXM86161_INT_DISABLE,//pwr_rdy
        MAXM86161_INT_DISABLE//sha
    }
};



static bool patchkeeper_ppg_maxm86161_init(void);
void patchkeeper_ppg_config(ble_mode_t mode, uint8_t ppg_sample_rate);
static void patchkeeper_ppg_maxm86161_run(void);
static void patchkeeper_ppg_maxm86161_pause(void);
void patchkeeper_ppg_irq_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
void patchkeeper_ppg_irq_handler_read_bytes(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
void patchkeeper_ppg_timeout_handler();
void patchkeeper_ppg_timer_init (void);
#endif 
