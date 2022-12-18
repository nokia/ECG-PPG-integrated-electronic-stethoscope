/*
 * © 2022 Nokia
 * Licensed under the BSD 3-Clause License
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "nordic_common.h"
#include "nrf.h"
#include "math.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrfx_pdm.h"
#include "nrf_rtc.h"

#include "BMI160_if.h"      // Accelerometer
#include "maxm86161.h"      //PPG

#include "I2C.h"            // I2C
#include "ADC.h"            // ADC

#include "app_timer.h"
//#include "SDcard.h"
#include "patchkeeper_saadc.h"
#include "patchkeeper.h"
#include "timestamp.h"
#include "ble_advertising.h"


extern uint8_t m_ble_connected;
extern ble_advertising_t    m_advertising;
patchkeeper_init_context_t  m_main_context;
patchkeeper_sensor_config_t m_patchkeeper_sensor_config;
patchkeeper_adc_data_t      m_adc_data;

extern uint16_t  m_write_offset;
extern datalog_memory_t *m_p_datalog_mem;
extern int Open_SDCard();
extern int List_SDCard();
extern void error_led_flash_loop(void);
static void datalogger_wait_func(void);
void Swap_and_Save_Buffer(void);
void Save_Audio_Buffer(audio_memory_t *);
static FRESULT sdcard_write(datalog_memory_t *, datalog_memory_t *);
static FRESULT sdcard_write_audio_trace(datalog_memory_t *, datalog_memory_t *);

//patchkeeper_data_t          m_patchkeeper_data;

uint8_t m_spi_busy = 0;
uint8_t m_ads1292_rdatac_active = 0;
uint8_t m_ads1292_powered = 0;


uint8_t m_resp_ch = 0;
ads1292_registers_t m_ads1292_registers;
ads1292_rdata_t     m_ads1292_rdata;



void patchkeeper_config_set_defaults(void) {

  m_patchkeeper_sensor_config.ecg_sensor.mode             =  PATCHKEEPER_DEFAULT_ECG_MODE;
  m_patchkeeper_sensor_config.gsr_sensor.mode             =  PATCHKEEPER_DEFAULT_GSR_MODE;
  m_patchkeeper_sensor_config.pulse_sensor.mode           =  PATCHKEEPER_DEFAULT_PULSE_MODE;
  m_patchkeeper_sensor_config.strain_sensor.mode          =  PATCHKEEPER_DEFAULT_STRAIN_MODE;
  m_patchkeeper_sensor_config.temp_sensor.mode            =  PATCHKEEPER_DEFAULT_TEMP_MODE;
  m_patchkeeper_sensor_config.imu_sensor.mode             =  PATCHKEEPER_DEFAULT_IMU_MODE;
  m_patchkeeper_sensor_config.battery.mode                =  PATCHKEEPER_DEFAULT_BATTERY_MODE;

  m_patchkeeper_sensor_config.ecg_sensor.sample_interval_ms      =  PATCHKEEPER_DEFAULT_ECG_SAMPLE_INTERVAL_MS;
  m_patchkeeper_sensor_config.gsr_sensor.sample_interval_ms      =  PATCHKEEPER_DEFAULT_GSR_SAMPLE_INTERVAL_MS;
  m_patchkeeper_sensor_config.pulse_sensor.sample_interval_ms    =  PATCHKEEPER_DEFAULT_PULSE_SAMPLE_INTERVAL_MS;
  m_patchkeeper_sensor_config.strain_sensor.sample_interval_ms   =  PATCHKEEPER_DEFAULT_STRAIN_SAMPLE_INTERVAL_MS;
  m_patchkeeper_sensor_config.temp_sensor.sample_interval_ms     =  PATCHKEEPER_DEFAULT_TEMP_SAMPLE_INTERVAL_MS;
  m_patchkeeper_sensor_config.imu_sensor.sample_interval_ms      =  PATCHKEEPER_DEFAULT_IMU_SAMPLE_INTERVAL_MS;
  m_patchkeeper_sensor_config.battery.sample_interval_ms         =  PATCHKEEPER_DEFAULT_BATTERY_SAMPLE_INTERVAL_MS;
 
 };


/*************************************
 ** @nRF5 Driver Instances          **
 *************************************/
static const nrf_drv_spi_t m_spi_master_0 = NRF_DRV_SPI_INSTANCE(0);


/**
 * @brief Function for switching the SPI clock to high speed mode.
 */
__STATIC_INLINE void set_spi_hispeed(void)
{
#ifdef SPI_PRESENT
    nrf_spi_frequency_set(m_spi_master_0.u.spi.p_reg, NRF_SPI_FREQ_2M);
#else
    nrf_spim_frequency_set(m_spi_master_0.u.spim.p_reg, NRF_SPI_FREQ_2M);
#endif
}

/*************************************
 ** @Patchkeeper Drivers (API)      **
 *************************************/

uint8_t patchkeeper_init(patchkeeper_init_context_t main_context) {

  m_main_context = main_context;

  // Set the default config
  patchkeeper_config_set_defaults();

  // Configure the GPIO ports
  ret_code_t err_code;
  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);
  nrf_gpio_cfg_output(ADS_START);
  nrf_gpio_cfg_output(ADS_PWDN);

  //ADC sampling related sensor pins
  nrf_gpio_cfg_output(BAT_MON_EN);
  nrf_gpio_cfg_output(PULSE_AMP_EN);
  nrf_gpio_cfg_output(GSR_AMP_EN);
  nrf_gpio_cfg_output(STRAIN_AMP_EN);
  nrf_gpio_cfg_output(TEMP_AMP_EN);

  // Configure and Enable the ADC
  patchkeeper_adc_init();

  //I2C bus
  I2C_init();

#if USE_SDCARD == 1
  // Write configuration information
  // (Although BLE not connected - it puts this as the first bit of information in the SDCard
  // datalog.
  config_datalogger_handler();

  nrf_gpio_cfg_input(SD_DETECTION, GPIO_PIN_CNF_PULL_Disabled);
 // disable ADS1292 ECG
 // nrf_gpio_cfg_output(CS_ADS);
 // nrf_gpio_pin_set(CS_ADS);   // _set() set the pin to HIGH  and // _clear() set the pin to LOW 
// End of disable ADS1292 ECG

  uint8_t sd_card = nrf_gpio_pin_read(SD_DETECTION);
  NRF_LOG_DEBUG("sd_card: %d",sd_card);
  nrf_delay_ms(10);


// check if SD card inserted, record data if there is SD card
if (sd_card == 1) {
  err_code = Open_SDCard(); 
    NRF_LOG_DEBUG("Open_SDCard error code: %d",err_code);
  if (err_code) { 
    error_led_flash_loop(); 
  } 
 
  err_code = List_SDCard(); 
 
  if (err_code) { 
    error_led_flash_loop(); 
  } 
  } else {
  // RED led indication for missing SD Card
   bsp_board_leds_off();
   bsp_board_led_invert(LED_RED);
   nrf_delay_ms(2000);
   bsp_board_leds_off();
   }
#endif  //end of USE_SDCARD == 1


#if BOARD_HAS_BMI160 == 1
  // Configure the IMU driver
  patchkeeper_imu_init();
#endif

  // Clear the ADC Data Buffer.
  memset(&m_adc_data,0,sizeof(m_adc_data));

  // Setup the battery config. This uses the same system as the analogue sensors
  // but we don't allow the Client App to change it - so set it here.
  m_patchkeeper_sensor_config.battery.mode = STREAMING;
  m_patchkeeper_sensor_config.battery.sample_interval_ms = PATCHKEEPER_DEFAULT_BATTERY_SAMPLE_INTERVAL_MS;

  // Start sampling the ADCs
  // TODO: Power Saving - Control this more.
  patchkeeper_start_sampling_adc();

  // TODO: If we want to stop sampling the ADCs at any point use
  // patchkeeper_stop_sampling_adc();

#if BOARD_HAS_BATMON == 1
  // Initialise the Battery . This also requires the ADCs to be running.
  patchkeeper_batt_init();
  // Enable the Battery Measurement Circuitry
  patchkeeper_batt_trigger();
#endif

#if BOARD_HAS_ADS1292 == 1
  // Configure and Initialise the SPI0 Driver
  patchkeeper_spi_init();
  // Initialise the ADS1292
   patchkeeper_ads1292_init();
  // Configure and enable the DRDY interrupt handler
  patchkeeper_ads1292_interrupt_init();

  //Configure and Initialise ADS1292R (inside patchkeeper_ecg_trigger())
  // NOTE! Do not need to use BOARD_HAS_ADS1292 any more as we can control
  // this default streaming by setting PATCHKEEPER_DEFAULT_ECG_MODE to OFF
  patchkeeper_ecg_trigger();

  set_spi_hispeed();
#endif

  // Start the ADC Sensors and IMU Based on PATCHKEEPER_DEFAULT_XXX_MODE

#if BOARD_HAS_BMI160 == 1
  patchkeeper_imu_trigger();
    NRF_LOG_DEBUG("imu trigger timer start");
#endif

// using default config above
//  patchkeeper_config_gsr(ble_mode_t mode, uint8_t sample_interval_ms);
//  patchkeeper_config_pulse();
//  patchkeeper_config_strain();
//  patchkeeper_config_temp();

#if BOARD_HAS_ANA_SENSOR == 1
  // Enable all analogue sensors
  // TODO: Power Saving Here...
  patchkeeper_enable_INA338_all();
#endif

  // Initialise the ADC Based Sensor Sampling
#if BOARD_HAS_GSR_SENSOR == 1
    patchkeeper_gsr_init();
    patchkeeper_gsr_trigger();
#endif

#if BOARD_HAS_STRAIN_SENSOR == 1
    patchkeeper_strain_init();
    patchkeeper_strain_trigger();
#endif

#if BOARD_HAS_PULSE_SENSOR == 1
    patchkeeper_pulse_init();
    patchkeeper_pulse_trigger();
#endif

#if BOARD_HAS_TEMP_SENSOR == 1
    patchkeeper_temp_init();
    patchkeeper_temp_trigger();
#endif

//#endif

#if BOARD_HAS_PDM_MIC == 1
  // Configure PDM for Audio Capture
   //NRF_LOG_DEBUG("PDM mic init");
  pdm_init();
  NRF_LOG_DEBUG("PDM mic init");
  nrf_delay_ms(100);
  
  pdm_start();
  NRF_LOG_DEBUG("PDM mic started");
  nrf_delay_ms(500);
  patchkeeper_audio_timer_config();
#endif // end of BOARD_HAS_PDM_MIC


#if BOARD_HAS_PPG_SENSOR == 1

  if (! patchkeeper_ppg_maxm86161_init()){
  NRF_LOG_DEBUG("PPG sensor initilised!");
  patchkeeper_ppg_maxm86161_run();
  NRF_LOG_DEBUG("PPG sensor start running...");

  } else {
  NRF_LOG_DEBUG("PPG sensor can't be initilised!");
    patchkeeper_ppg_maxm86161_pause();
  };


#endif

  return 0;
}



void patchkeeper_config_ecg(ble_mode_t mode, uint8_t sample_interval_ms){

  // TODO: This currently does not control the sampling rate!
  // This needs to control the ADS1292.
  // Update values.

#if BOARD_HAS_ADS1292 == 1
  m_patchkeeper_sensor_config.ecg_sensor.mode = mode;
  m_patchkeeper_sensor_config.ecg_sensor.sample_interval_ms = sample_interval_ms;

  // The above mode and sample interval are not implemented yet!
  // Once implemented, the changes can be made with patchkeeper_ecg_trigger too Trigger the Changes.

  if (mode == STREAMING) {
    patchkeeper_ecg_trigger();
  } else {
    patchkeeper_ecg_stop();
  }
#endif
};



/***************************************************
 * IMU Sensor
 ***************************************************/

void patchkeeper_config_imu(ble_mode_t mode, uint8_t sample_interval_ms){

  // Update values.
  m_patchkeeper_sensor_config.imu_sensor.mode = mode;
  m_patchkeeper_sensor_config.imu_sensor.sample_interval_ms = sample_interval_ms;

  // Trigger the Changes.
  #if BOARD_HAS_BMI160 == 1
    patchkeeper_imu_trigger();
  #endif
};


#if BOARD_HAS_BMI160 == 1
/**
 * IMU  Timeout Handler
 */

// Get data direct from sensor rather than use driver as driver converts into a structure that
// wastes storage, and then needs to be converted into bytes for the SDCard writes. Better to
// keep it in bytes as straight out of BMI160
// Format is gx0,gx1,gy0,gy1,gz0,gz1,ax0,ax1,ay0,ay1,az0,az1,s0,s1,s2 which is 15 bytes but
// this then gets aligned to 32 bit boundary -> 16 bytes.


void patchkeeper_imu_timeout_handler(void* p_void_context) {
//  patchkeeper_context_t* p_context = (patchkeeper_context_t*) p_void_context;
  imu_data_t                imu_data;
  static uint8_t            packet_count = 0;
  ts64_t                    timestamp = get_timestamp();
  uint16_t                  size = sizeof(imu_data_t);

//#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
//  if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
//      Swap_and_Save_Buffer();
//  }
//#endif //PATCHKEEPER_BUFFER_SENSOR_DATA

//  watchdog_refresh();

#if BOARD_HAS_BMI160 == 0
  return;
#endif

  imu_data.packet_info.sop            = SOP_BELLLABS;
  imu_data.packet_info.timestamp_lo   = timestamp.lo;
  imu_data.packet_info.timestamp_hi   = timestamp.hi;
  imu_data.packet_info.logger_id      = PATCHKEEPER_IMU_ID;
  imu_data.packet_info.length         = PATCHKEEPER_IMU_DATA_LENGTH_BYTES;
  imu_data.packet_info.custom         = packet_count++;

#if USE_DUMMY_SENSORS == 0

  BMI160_Get_Data(imu_data.data);

  // Clear the MSByte of the TimeStamp as this is only 24-bit into a 32-bit field
  *((int8_t*) imu_data.data+15) = 0;
#else
  for (int i=0; i<PATCHKEEPER_IMU_DATA_LENGTH_HALF_WORDS; i++) {
    imu_data.data[i] = (i<<8) + i;
  }
#endif

#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
  if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
      Swap_and_Save_Buffer();
  }
//#endif //PATCHKEEPER_BUFFER_SENSOR_DATA
//#if PATCHKEEPER_BUFFER_SENSOR_DATA==1 
  //ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES); 
  memcpy((m_p_datalog_mem+m_write_offset), &imu_data,  size); 
  m_write_offset += size;
#endif //PATCHKEEPER_BUFFER_SENSOR_DATA


#if BLE_STREAM_IMMEDIATE==1
  if (m_ble_connected == 1) {
    m_main_context.ble_send_data((unsigned char*) &imu_data, (unsigned char*) &imu_data+size);
  }
#endif
}


/*************************************
 ** @IMU Timer                      **
 *************************************/
 APP_TIMER_DEF(m_imu_timer);

 m_sensor_timer_state_t m_imu_timer_state = UNINITIALISED;

void patchkeeper_imu_timer_init (void) {

  if (m_imu_timer_state == UNINITIALISED) {
    app_timer_create(&m_imu_timer,APP_TIMER_MODE_REPEATED,&patchkeeper_imu_timeout_handler);

    m_imu_timer_state = IDLE;
  }
}

void patchkeeper_imu_timer_trigger (void) {
  if ((m_imu_timer_state == IDLE) && (m_patchkeeper_sensor_config.imu_sensor.mode == STREAMING)) {
    app_timer_start(m_imu_timer, APP_TIMER_TICKS(m_patchkeeper_sensor_config.imu_sensor.sample_interval_ms), NULL);
    m_imu_timer_state = RUNNING;
  }

  if ((m_imu_timer_state == RUNNING) && (m_patchkeeper_sensor_config.imu_sensor.mode == OFF)) {
    app_timer_stop(m_imu_timer);
    m_imu_timer_state = IDLE;
  }

};

/**
 * IMU trigger based on timer
 */

void patchkeeper_imu_trigger(void){
  patchkeeper_imu_timer_trigger();
}


/*****************************************************************************
 * IMU hardware interrupt, used to stop device: stop ble and save data to SD card
 *****************************************************************************/

void patchkeeper_imu_isr(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    NRF_LOG_DEBUG("IMU1 Interrupt Detected");

//    watchdog_refresh();

#if USE_BLE_COMMS == 1
     m_main_context.ble_advertising_start(1);
#endif

#if USE_SDCARD == 1
    // Write data to SDCard so we can switch off and/or remove card.
    NRF_LOG_DEBUG("Flushing Data to SDCard.");
    Swap_and_Save_Buffer();
#endif

    NRF_LOG_FLUSH();
}


/**
 * IMU init
 */

void patchkeeper_imu_init(void) {
  ret_code_t err_code;

  //BMI160 IMU
//#if BOARD_HAS_BMI160 == 1
  BMI160_Turn_On();

  // If using BLE, use INT1 interrupt to restart advertising
  // If using SDCard, use INT1 to force datapurge to SDCard.

  // Enable INT1 as interrupt using GPIOTE
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
  //in_config.pull = NRF_GPIO_PIN_PULLUP; 
  err_code = nrf_drv_gpiote_in_init(IMU_INT1, &in_config, patchkeeper_imu_isr);
  APP_ERROR_CHECK(err_code);
  nrf_drv_gpiote_in_event_enable(IMU_INT1, true);
  nrf_delay_ms(50);
//#endif

  // Initialise the IMU Timer
  patchkeeper_imu_timer_init();

// TODO: Remove? Do we want this running from start of day?
//  patchkeeper_imu_trigger();

}

#endif



#if BOARD_HAS_ADS1292 == 1

/*************************************
 ** ADS1292 Related functions       **
 *************************************/

/**
 * SPI0 interface for Patchkeeper
 */
 //

void patchkeeper_spi_handler (nrf_drv_spi_evt_t const *p_event, void *p_trans) {
  if (p_event->type == NRF_DRV_SPI_EVENT_DONE) {
//    NRF_LOG_DEBUG("SPI0Done");
    m_spi_busy = 0;
  }
}

uint8_t patchkeeper_spi_init(void) {
  ret_code_t err_code;

  nrf_drv_spi_config_t patchkeeper_spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  patchkeeper_spi_config.mosi_pin   = SPI_MOSI;
  patchkeeper_spi_config.miso_pin   = SPI_MISO;
  patchkeeper_spi_config.sck_pin    = SPI_SCK;
  patchkeeper_spi_config.ss_pin     = CS_ADS;
  patchkeeper_spi_config.frequency  = NRF_DRV_SPI_FREQ_1M;
  patchkeeper_spi_config.mode       = NRF_DRV_SPI_MODE_1;

  err_code = nrf_drv_spi_init(&m_spi_master_0, &patchkeeper_spi_config, &patchkeeper_spi_handler, NULL);

  if (err_code != NRF_SUCCESS)
  {
      // Initialization failed. Take recovery action.
      NRF_LOG_ERROR("SPI0 Initialisation FAILED!!!");
//      ASSERT(0);
  }
  return err_code;
}


/*************************************
 ** @ADS1292 Drivers                **
 *************************************/

/**
 * patchkeeper_ads1292_spi_cmd
 * SPI command for ADS1292
 */
void patchkeeper_ads1292_spi_cmd(uint8_t command){
  m_spi_busy = 1;
  nrf_drv_spi_transfer(&m_spi_master_0, &command, 1, NULL, 0);
  while (m_spi_busy) {};
}

/*
 * patchkeeper_ads1292_powerdown
 * Turn on ADS1292
 */
void patchkeeper_ads1292_powerup (void) {
  // Uses ADS1292R through SPI.
//  static uint8_t command[10];

  // Hold START low on the ADS1292R
  nrf_gpio_pin_clear(ADS_START);
  nrf_delay_ms(10);
  // Power up the ADS1292R
  nrf_gpio_pin_set(ADS_PWDN);
  nrf_delay_ms(100);

  // Reset the ADS1292R
  nrf_gpio_pin_clear(ADS_PWDN);
  nrf_delay_ms(100);
  nrf_gpio_pin_set(ADS_PWDN);
  nrf_delay_ms(500);
}

/*
 * patchkeeper_ads1292_powerdown
 * Turn off ADS1292
 */
void patchkeeper_ads1292_powerdown (void) {

  // Power down the ADS1292R
  nrf_gpio_pin_clear(ADS_PWDN);

  NRF_LOG_DEBUG("ADS1292R Powered Down?...");
  m_ads1292_powered = 0;

};

/*
 * patchkeeper_ads1292_powerdown
 * Put ADS1292 in standby mode
 */
void patchkeeper_ads1292_standby (void) {
  // Put the ADS1292R into STANDBY Mode
  m_ads1292_rdatac_active = 0;
  patchkeeper_ads1292_spi_cmd(ADS1292R_CMD_STANDBY);
  NRF_LOG_DEBUG("ADS1292R Sleeping...");
//  m_ads1292_powered = 0;
}


/*
 *  * patchkeeper ads1292r read register with spi
 * Reads all the register and places into a structure to replicate the register map
 */
void patchkeeper_ads1292_spi_regr(ads1292_registers_t* p_registers){
  uint8_t command[2];

//  uint8_t* p_data = &(p_registers->id); // there are two dummy bytes added for correct reading
  command[0] = ADS1292R_CMD_RREG;
  command[1] = sizeof(ads1292_registers_t) -1 ;


  m_spi_busy = 1;
  nrf_drv_spi_transfer(&m_spi_master_0, command, 2, (uint8_t*) p_registers, sizeof(ads1292_registers_t));
  while (m_spi_busy) {};
}


/*
 * patchkeeper ads1292r write register with spi
 * Writes registers
 */
void patchkeeper_ads1292_spi_regw(ads1292_registers_t* p_registers){
  uint8_t command[2];

  // We don't want to transmit the 2 dummy entries
  uint8_t* p_data = &(p_registers->id);
  uint8_t transfer_size = sizeof(ads1292_registers_t) - 2;
  command[0] = ADS1292R_CMD_WREG;
  command[1] = transfer_size;

  m_spi_busy = 1;
  nrf_drv_spi_transfer(&m_spi_master_0, command, 2, p_data, transfer_size);
  while (m_spi_busy) {};
}

/*
 * patchkeeper ads1292r write all registers with spi
 * Writes all registers in one go
 */
void patchkeeper_ads1292_spi_regw_all(ads1292_registers_t* p_registers){
  uint8_t transfer_size = sizeof(ads1292_registers_t);
  uint8_t command[transfer_size]; // total bytes to send for write registers

  command[0] = 0x40; //write reg from address 00h, total 12 register to be written, and id has to included due to struct
  command[1] = 0x0b; // total 12 register to be written, decial (12 - 1) = 0x0b bytes to send
  p_registers->dummy_read_0 = command[0];
  p_registers->dummy_read_1 = command[1];
  memcpy(&command, p_registers, (uint8_t) sizeof(ads1292_registers_t));
//  uint8_t* p_data = &(p_registers->id);ƒ

  m_spi_busy = 1;
  ret_code_t err_code;
   err_code =  nrf_drv_spi_transfer(&m_spi_master_0, command, transfer_size, NULL,0);
//  nrf_drv_spi_transfer(&m_spi_master_0, (uint8_t*) p_registers, transfer_size, NULL,0);
  while (m_spi_busy) {};
    NRF_LOG_DEBUG("ads1292r spi reg write done");


}


/*
* patchkeeper_ads1292_init
* ADS1292R Intialisation
*/

void patchkeeper_ads1292_init (void) {
  ret_code_t err_code;
  if (!m_ads1292_powered) {
      NRF_LOG_DEBUG("m_ads1292_powered: %d", m_ads1292_powered );
    patchkeeper_ads1292_powerup();
      NRF_LOG_DEBUG("m_ads1292_powered up" );
    m_ads1292_powered = 1;
    m_ads1292_registers.id        = 0x73;
    m_ads1292_registers.config1   = 0b00000000; //0x00; //Set sampling rate to 125 SPS
    m_ads1292_registers.config2   = 0b10100000; //0xa0; //Lead-off comp off, test signal disabled
    m_ads1292_registers.loff      = 0b00010000; //0x10; ///Lead-off defaults
    //m_ads1292_registers.ch1set    = 0b01000000; //0x40  //Ch 1 enabled, gain 4, connected to electrode in,
    //m_ads1292_registers.ch1set    = 0b00000000; //0x00  //Ch 1 enabled, gain 6, connected to electrode in, was 0100=gain 4
    m_ads1292_registers.ch1set    = 0b00100000; //0x20  //Ch 1 enabled, gain 2, connected to electrode in, was 0100=gain 4

    //m_ads1292_registers.ch2set    = 0b00010000; //0x10  //Ch 2 enabled, gain 1, connected to electrode in
    m_ads1292_registers.ch2set    = 0b00000000; //0x00  //Ch 2 enabled, gain 6, connected to electrode in
    //m_ads1292_registers.ch2set    = 0b01100000; //0x60  //Ch 2 enabled, gain 12, connected to electrode in
    m_ads1292_registers.rld_sens  = 0b00101100; //0x2C   //RLD settings: fmod/16, RLD enabled, RLD inputs from Ch2 only
    //m_ads1292_registers.rld_sens  = 0b00100011; //0x23   //RLD settings: fmod/16, RLD enabled, RLD inputs from Ch1 only

    m_ads1292_registers.loff_sens = 0b00000000; //0x00; //LOFF settings: all disabled, this is defalut
    m_ads1292_registers.loff_stat = 0b00000000; //0x00  //OFF Settings default,
    
    //m_ads1292_registers.resp1     = 0b11110010; //0xf2  //Respiration: MOD/DEMOD turned on , phase 135 , internal clock
    //m_ads1292_registers.resp1     = 0b11101010; //0xEA  //Respiration: MOD/DEMOD turned on , phase 112.5 , internal clock
    m_ads1292_registers.resp1     = 0b11110110; //0xEA  //Respiration: MOD/DEMOD turned on , phase 146.25 , internal clock

//    m_ads1292_registers.resp1     = 0b00110010; //  //Respiration: MOD/DEMOD turned off , phase 135degree, internal
    m_ads1292_registers.resp2     = 0b00000011; //0x03  //Respiration: default, Calib OFF, respiration freq 32kHz, RLDREF internal
    m_ads1292_registers.gpio      = 0b00000110; //0x06  //default

    patchkeeper_ads1292_spi_regw_all(&m_ads1292_registers);
    patchkeeper_ads1292_spi_regr(&m_ads1292_registers);
    NRF_LOG_DEBUG("m_ads1292_registers.id:0x%x\n",m_ads1292_registers.id);

   }
}



/********************************************************************
* patchkeeper ads1292 read data continously
* Prepare ADS1292R and put it into RDATAC Mode (Read Data Continuous)
********************************************************************/

void patchkeeper_ads1292_rdatac (void) {

  // Wake Up the ADS1292R
  patchkeeper_ads1292_spi_cmd(ADS1292R_CMD_WAKEUP);
  nrf_delay_ms(100);

  // Send SDATAC Command: Stop Read Data Continuously mode
  patchkeeper_ads1292_spi_cmd(ADS1292R_CMD_SDATAC);
  nrf_delay_ms(100);

  // Check that the ADS1292R is present and functioning.
  // Read the registers and check REG_ID Reg
  memset(&m_ads1292_registers, 0, (uint8_t) sizeof(ads1292_registers_t));
  patchkeeper_ads1292_spi_regr(&m_ads1292_registers);
  NRF_LOG_INFO("m_ads1292_registers.id:0x%x\n",m_ads1292_registers.id);
  ASSERT(m_ads1292_registers.id == 0x73);
 // ASSERT(m_ads1292_registers.config1 == 0x02); // check default sampling rate: 0x02: 500

#if NBL_LOG_ECG == 1
//  NRF_LOG_INFO("m_ads1292_registers.dummy_read_0:0x%x\n",m_ads1292_registers.dummy_read_0);
//  NRF_LOG_INFO("m_ads1292_registers.dummy_read_1:0x%x\n",m_ads1292_registers.dummy_read_1);
  NRF_LOG_INFO("m_ads1292_registers.id:0x%x\n",m_ads1292_registers.id);
  NRF_LOG_INFO("m_ads1292_registers.config1:0x%x\n",m_ads1292_registers.config1);
  NRF_LOG_INFO("m_ads1292_registers.config2:0x%x\n",m_ads1292_registers.config2);
  NRF_LOG_INFO("m_ads1292_registers.loff:0x%x\n",m_ads1292_registers.loff);
  NRF_LOG_INFO("m_ads1292_registers.ch1set:0x%x\n",m_ads1292_registers.ch1set);
  NRF_LOG_INFO("m_ads1292_registers.ch2set:0x%x\n",m_ads1292_registers.ch2set);
  NRF_LOG_INFO("m_ads1292_registers.rld_sens:0x%x\n",m_ads1292_registers.rld_sens);
  NRF_LOG_INFO("m_ads1292_registers.loff_sens:0x%x\n",m_ads1292_registers.loff_sens);
  NRF_LOG_INFO("m_ads1292_registers.loff_stat:0x%x\n",m_ads1292_registers.loff_stat);
  NRF_LOG_INFO("m_ads1292_registers.resp1:0x%x\n",m_ads1292_registers.resp1);
  NRF_LOG_INFO("m_ads1292_registers.resp2:0x%x\n",m_ads1292_registers.resp2);
  NRF_LOG_INFO("m_ads1292_registers.gpio:0x%x\n",m_ads1292_registers.gpio);

#endif

  // Put the ADS1292R into RDATAC Mode: read data continously
  m_ads1292_rdatac_active = 1;
  patchkeeper_ads1292_spi_cmd(ADS1292R_CMD_RDATAC);
  NRF_LOG_DEBUG("ADS1292 RDATAC MODE");

  // Send the START Command
  patchkeeper_ads1292_spi_cmd(ADS1292R_CMD_START);
};


/*
* patchkeeper_ads1292_drdy_handler
* ADS1292R Data Ready (drdy) handle
*/

void patchkeeper_ads1292_drdy_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action){

#if BOARD_HAS_ADS1292 == 0
  return;
#endif
  ts64_t                    timestamp = get_timestamp();

  static uint8_t           packet_count = 0;
  static uint8_t            sub_packet_count = 0;

  static ecg_data_t         ecg_data;
  unsigned char*            start_ptr = (unsigned char*)  &ecg_data;
  unsigned char*            end_ptr =   (unsigned char*)  &ecg_data +  sizeof(ecg_data_t);
  uint16_t   size = sizeof(ecg_data_t);

//#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
//  if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
//      Swap_and_Save_Buffer();
//      }
//#endif // PATCHKEEPER_BUFFER_SENSOR_DATA


  if (packet_count == 0)
  {
    ecg_data.packet_info.sop                      = SOP_BELLLABS;
    ecg_data.packet_info.logger_id                = PATCHKEEPER_ECG_ID;
    ecg_data.packet_info.length                   = PATCHKEEPER_ECG_DATA_LENGTH; //12 x 19 = 228 
  }
  if (sub_packet_count == 0)
  {
    ecg_data.packet_info.timestamp_hi             = timestamp.hi;
    ecg_data.packet_info.timestamp_lo             = timestamp.lo;
  }

    //NRF_LOG_DEBUG("m_ads1292_rdatac_active = %d\n",m_ads1292_rdatac_active);
  if (m_ads1292_rdatac_active)
  {
    m_spi_busy = 1;
    nrf_drv_spi_transfer(&m_spi_master_0, NULL, 0, (uint8_t*) &m_ads1292_rdata, sizeof(m_ads1292_rdata));
    while (m_spi_busy) {};
  }

  ecg_data.ads1992_packet[sub_packet_count].timestamp_lo          = timestamp.lo;
  ecg_data.ads1992_packet[sub_packet_count].status                = m_resp_ch << 5 || (m_ads1292_rdata.status[0] && 0x1f); 
  // this put m_resp_ch into the third bit from left to status, which still contains first 4 bits from left of ads1292 status
  ecg_data.ads1992_packet[sub_packet_count].packet_count          = sub_packet_count;
  ecg_data.ads1992_packet[sub_packet_count].channel1_data[0]      = m_ads1292_rdata.data0[0];
  ecg_data.ads1992_packet[sub_packet_count].channel1_data[1]      = m_ads1292_rdata.data0[1];
  ecg_data.ads1992_packet[sub_packet_count].channel1_data[2]      = m_ads1292_rdata.data0[2];
  ecg_data.ads1992_packet[sub_packet_count].channel2_data[0]      = m_ads1292_rdata.data1[0];
  ecg_data.ads1992_packet[sub_packet_count].channel2_data[1]      = m_ads1292_rdata.data1[1];
  ecg_data.ads1992_packet[sub_packet_count].channel2_data[2]      = m_ads1292_rdata.data1[2];

  if (sub_packet_count < (PATCHKEEPER_NUM_AGGREGATED_ADS1292_PACKET-1))
  {
    sub_packet_count++;
  }
  else
  {
    sub_packet_count = 0;
    ecg_data.packet_info.custom  = packet_count++;
 

#if NBL_LOG_ECG == 1
  uint32_t  temp1 = 0;
  uint32_t  temp2 = 0;
  int32_t ch1_reading = 0;
  int32_t ch2_reading = 0;
  //m_ads1292_rdata.data0[], 1[], MSB first, are 24 bits signed intergal, need to be converted 32 bits signed intergal
  temp1 = m_ads1292_rdata.data0[0] << 16 | m_ads1292_rdata.data0[1] << 8 | m_ads1292_rdata.data0[2];
  temp1 = temp1 << 8; 
  ch1_reading = (signed long) temp1;
  ch1_reading = (signed long) (ch1_reading >> 8);
  NRF_LOG_DEBUG("channel1 reading: %d",ch1_reading);
  
  temp2 = m_ads1292_rdata.data0[3] << 16 | m_ads1292_rdata.data0[4] << 8 | m_ads1292_rdata.data0[5];
  temp2 = temp2 << 8; 
  ch2_reading = (signed long) temp2;
  ch2_reading = (signed long) (ch2_reading >> 8);
  NRF_LOG_DEBUG("channel2 reading: %d",ch2_reading);
#endif


#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
  if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
      Swap_and_Save_Buffer();
      }
//#endif // PATCHKEEPER_BUFFER_SENSOR_DATA

//#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
  //ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES);
  memcpy((m_p_datalog_mem+m_write_offset), start_ptr, size);
  m_write_offset += size;
#endif  //PATCHKEEPER_BUFFER_SENSOR_DATA

#if BLE_STREAM_IMMEDIATE==1
    if (m_ble_connected == 1)
    {
      m_main_context.ble_send_data((uint8_t *) start_ptr, (uint8_t *) end_ptr);
#if NBL_LOG_BLE_SEND == 1
    NRF_LOG_DEBUG("ECG Packet %d complete", packet_count);
#endif
    }
#endif
//    NRF_LOG_DEBUG("Number of packet_count: %d",packet_count);
  }

//#define DEBUG_ECG_PRINT_DATA
#ifdef DEBUG_ECG_PRINT_DATA
  NRF_LOG_DEBUG("ts=0x%04x",timestamp.lo);
  NRF_LOG_DEBUG("ch1_data1=%5d ch1_data2=%5d ch1_data3=%5d ch2_data1=%5d ch2_data2=%5d ch2_data3=%5d",
    m_ads1292_rdata.data0[0],
    m_ads1292_rdata.data0[1],
    m_ads1292_rdata.data0[2],
    m_ads1292_rdata.data1[0],
    m_ads1292_rdata.data1[1],
    m_ads1292_rdata.data1[2]);
#endif

}



void patchkeeper_ads1292_interrupt_init() {
  ret_code_t err_code;

  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
  //in_config.pull = NRF_GPIO_PIN_NOPULL; 
  err_code = nrf_drv_gpiote_in_init(ADS_DRDY, &in_config, patchkeeper_ads1292_drdy_handler);
  APP_ERROR_CHECK(err_code);
  nrf_drv_gpiote_in_event_enable(ADS_DRDY, true);
  nrf_delay_ms(50);
}



/*************************************
 ** @ECG Measurement                **
 *************************************/

/*
 * @patchkeeper_ecg_start
 * Puts the ADS1292 into RDATAC_Mode
 */
void patchkeeper_ecg_start(void) {
  patchkeeper_ads1292_rdatac();
}

/*
 * @patchkeeper_ecg_stop
 * Puts the ADS1292 into Standby
 */
void patchkeeper_ecg_stop(void) {
  patchkeeper_ads1292_standby();
}

/*****************************************
 * @patchkeeper ecg trigger command
 * Trigger ECG if enabled in config: to start ecg for streaming or process on device
 *****************************************/
void patchkeeper_ecg_trigger () {
  //patchkeeper_ads1292_init(); // Does nothing if it is already initialised.
   // needed for control ads1292 over BLE in the future. 

  if (m_patchkeeper_sensor_config.ecg_sensor.mode != OFF) {
    patchkeeper_ecg_start();
    NRF_LOG_DEBUG("ECG started");
  } else {
    patchkeeper_ecg_stop();
    NRF_LOG_DEBUG("ECG Disabled");
  }
}

/*************************************
 ** @Respiration Measurement        **
 *************************************/
//void patchkeeper_trigger_resp (void* p_data) {};

#endif






#if BOARD_HAS_PPG_SENSOR == 1
/*************************************
 ** PPG MAXM86161 Measurement related **
 *************************************/

/*state of maxm86161 device*/
bool ppg_turn_on = false;
bool ppg_reset_request = false;
ble_mode_t ble_stream_ppg = PATCHKEEPER_DEFAULT_PPG_MODE; 

//uint32_t patchkeeper_ppg_dutycycle_interval_ms = 300000;  //5mins
//uint64_t ppg_on_time_ms = 30000; //  30 seconds
//uint32_t patchkeeper_ppg_dutycycle_interval_ms = PATCHKEEPER_DEFAULT_PPG_DUTYCYCLE_INTERVAL_MS;  
uint16_t patchkeeper_ppg_on_counts_of_timer = PATCHKEEPER_DEFAULT_PPG_ON_COUNTS_OF_TIMER;
uint16_t patchkeeper_ppg_dutycycle_counts_of_timer = PATCHKEEPER_DEFAULT_PPG_DUTYCYCLE_COUNTS_OF_TIMER;
//uint64_t ppg_on_time_ms = PATCHKEEPER_DEFAULT_PPG_ON_TIME_MS; 
uint16_t ppg_timer_cycle_count = 0; 

//ts64_t  ppg_measurement_timer; 
//uint8_t ppg_measurement_timer_run = 1; 


 maxm86161_device_config_t device_config;


APP_TIMER_DEF(m_ppg_timer);
m_sensor_timer_state_t m_ppg_timer_state = UNINITIALISED;

void patchkeeper_ppg_timer_init (void) {

  if (m_ppg_timer_state == UNINITIALISED) {
    app_timer_create(&m_ppg_timer, APP_TIMER_MODE_REPEATED,&patchkeeper_ppg_timeout_handler);
    m_ppg_timer_state = IDLE;
  }
}


void patchkeeper_ppg_timer_trigger (void) {
  if ((m_ppg_timer_state == IDLE) && ((ble_stream_ppg == STREAMING) || (ble_stream_ppg == ON_DEVICE_PROCESSING))) {
    app_timer_start(m_ppg_timer, APP_TIMER_TICKS(PATCHKEEPER_DEFAULT_PPG_TIMER_MS), NULL);
    //app_timer_start(m_ppg_timer, APP_TIMER_TICKS(patchkeeper_ppg_dutycycle_interval_ms), NULL);

    m_ppg_timer_state = RUNNING;
    //NRF_LOG_DEBUG("Starting PPG Timer");
  }

  if ((m_ppg_timer_state == RUNNING) && (ble_stream_ppg == OFF)  ) {
    app_timer_stop(m_ppg_timer);
    m_ppg_timer_state = IDLE;
  }

  if ((m_ppg_timer_state == IDLE) && (ble_stream_ppg == OFF)) {
    NRF_LOG_DEBUG("PPG Timer Disabled");
  }

};


void patchkeeper_ppg_timeout_handler(){
    if (ppg_reset_request == true){
          //maxm86161_software_reset();
          maxm86161_init_device(device_config);
          ppg_reset_request = false;
          patchkeeper_ppg_maxm86161_run();
      }

//    ppg_timer_cycle_count++;
//#if NBL_LOG_PPG == 1
//    NRF_LOG_DEBUG("ppg_timer_cycle_count: %d,",ppg_timer_cycle_count);
//#endif

//    if (ppg_reset_request == true && ppg_timer_cycle_count < patchkeeper_ppg_on_counts_of_timer){
//          //maxm86161_software_reset();
//          maxm86161_init_device(device_config);
//          ppg_reset_request = false;
//          patchkeeper_ppg_maxm86161_run();
//      }

// if (ppg_timer_cycle_count == patchkeeper_ppg_on_counts_of_timer) {
//      patchkeeper_ppg_maxm86161_pause();
//#if NBL_LOG_PPG == 1
//      NRF_LOG_DEBUG("ppg_timer_cycle_count: %d, stopping PPG...",ppg_timer_cycle_count);
//      //ppg_measurement_timer_run = 1;    
//#endif
//    } else if (ppg_timer_cycle_count == patchkeeper_ppg_dutycycle_counts_of_timer) {
//      if (ppg_reset_request){
//          maxm86161_init_device(device_config);
//          ppg_reset_request = false;
//          }
//      patchkeeper_ppg_maxm86161_run();

//#if NBL_LOG_PPG == 1
//      NRF_LOG_DEBUG("ppg_timer_cycle_count: %d, restarting PPG...",ppg_timer_cycle_count);

//      //ppg_measurement_timer_run = 1;    
//#endif

//      ppg_timer_cycle_count = 0;

//    }

}


static bool patchkeeper_ppg_maxm86161_init(void){
     //maxm86161_shutdown_device(true);
    ret_code_t err_code;    
    // Enable PPG_LDO_PIN
    nrf_gpio_cfg_output(PPG_LDO_EN);

    nrf_gpio_pin_clear(PPG_LDO_EN); 
    nrf_delay_ms(50);
    nrf_gpio_pin_set(PPG_LDO_EN);   
    nrf_delay_ms(100);                       
  // Enable PPG_INT as interrupt using GPIOTE
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
  //"false" means that gpiote will use the PORT event, which is low power,
  //i.e. does not add any noticable current consumption (<<1uA).
  //Setting this to "true" will make the gpiote module use GPIOTE->IN events which 
  //add ~8uA for nRF52 and ~1mA for nRF51.
  in_config.pull = NRF_GPIO_PIN_NOPULL; 
  //in_config.sense = NRF_GPIOTE_POLARITY_HITOLO,
  //in_config.is_watcher = false,
  //in_config.hi_accuracy = true,
  //in_config.skip_gpio_setup = false,
  err_code = nrf_drv_gpiote_in_init(PPG_INT, &in_config, patchkeeper_ppg_irq_handler_read_bytes);
  APP_ERROR_CHECK(err_code);
  nrf_drv_gpiote_in_event_enable(PPG_INT, true);
  nrf_delay_ms(50);
  uint8_t part_id;
       
   if (!maxm86161_identify_part(&part_id))
      {
      return 1;
      }

      
#if PPG_FOR_HR_ONLY == 1
    device_config =  default_green_led_config;
 // change sample rate in config need to change FIRMWARE_BUILD_NUM (99sps: 1) for correct python script data parsing

#elif PPG_FOR_SPO2_ONLY == 1
     device_config =  default_spo2_config;
#else
     device_config = default_maxim_config; // default 50sps, low led power
    // device_config = maxim_config_3leds_99sps;
 // change sample rate in config need to change FIRMWARE_BUILD_NUM (50sps: 0) for correct python script data parsing

#endif

  err_code = maxm86161_init_device(device_config);

  //maxm86161_i2c_write_to_register(MAXM86161_REG_LED_RANGE1, 0x3f);

  //uint8_t register_reading = 0;
  //register_reading = maxm86161_i2c_read_from_register(MAXM86161_REG_LED1_PA);
  //NRF_LOG_DEBUG("register_reading LED1_PA: 0x%02x", register_reading);
  //register_reading = maxm86161_i2c_read_from_register(MAXM86161_REG_LED2_PA);
  //NRF_LOG_DEBUG("register_reading LED2_PA: 0x%02x", register_reading);
  //register_reading = maxm86161_i2c_read_from_register(MAXM86161_REG_LED3_PA);
  //NRF_LOG_DEBUG("register_reading LED3_PA: 0x%02x", register_reading);
  //register_reading = maxm86161_i2c_read_from_register(MAXM86161_REG_LED_PILOT_PA);
  //NRF_LOG_DEBUG("register_reading pilot_PA: 0x%02x", register_reading);
  //register_reading = maxm86161_i2c_read_from_register(MAXM86161_REG_LED_SEQ1);
  //NRF_LOG_DEBUG("MAXM86161_REG_LED_SEQ1: 0x%02x", register_reading);
  //register_reading = maxm86161_i2c_read_from_register(MAXM86161_REG_LED_RANGE1);
  //NRF_LOG_DEBUG("MAXM86161_REG_LED_RANGE1: 0x%02x", register_reading);
  
  maxm86161_shutdown_device(true);
  patchkeeper_ppg_timer_init();
  patchkeeper_ppg_timer_trigger();
    return err_code;
}


/**************************************************************************//*
 * @brief
 *  Start the device's autonomous measurement operation.
 *  The device can be configure over BLE.
 *
 * @return
 *  Returns error status.
 *****************************************************************************/
void patchkeeper_ppg_config(ble_mode_t mode, uint8_t ppg_sample_rate){
      
    ret_code_t err_code;    
    uint8_t part_id;
    maxm86161_device_config_t device_config = default_maxim_config;
    device_config.ppg_cfg.smp_rate = ppg_sample_rate;
    if (!maxm86161_identify_part(&part_id))
      {
      //return 1;
      }
    ble_stream_ppg = mode; 
    err_code = maxm86161_init_device(device_config);

  switch(mode){
    case (OFF):
     patchkeeper_ppg_maxm86161_pause();
      break;

    case (STREAMING): 
     patchkeeper_ppg_maxm86161_run();
     break;
     
    case (ON_DEVICE_PROCESSING): 
     patchkeeper_ppg_maxm86161_run();

     break;

     default: break;
     }
};

/**************************************************************************//*
 * @brief
 *  Start the device's autonomous measurement operation.
 *  The device must be configured before calling this function.
 *
 * @return
 *  Returns error status.
 *****************************************************************************/
static void patchkeeper_ppg_maxm86161_run(void)
{
  //maxm86161_flush_fifo();
  maxm86161_shutdown_device(false);
}

/**************************************************************************//**
 * @brief
 *  Pause the device's autonomous measurement operation.
 *  HRM must be running before calling this function.
 *
 * @return
 *  Returns error status.
 *****************************************************************************/
static void patchkeeper_ppg_maxm86161_pause(void)
{
  maxm86161_shutdown_device(true);
  NRF_LOG_DEBUG("maxm86161 shut down");
}


/**************************************************************************//**
 * @brief GPIO Interrupt handler for event pins.
 *****************************************************************************/
void patchkeeper_ppg_irq_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  patchkeeper_ppg_sample_t    ppg_data;
  static uint8_t           packet_count = 0;
  static uint64_t           packet_count_1 = 0;
  ts64_t                    timestamp = get_timestamp();
  uint16_t                  size =   sizeof(patchkeeper_ppg_sample_t);
  uint16_t                   length = sizeof(maxm86161_ppg_sample_t);
  
  //NRF_LOG_DEBUG("ppg_data size: %d", size);

//#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
//  if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
//  //if (m_ble_connected || (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES)) {
//      Swap_and_Save_Buffer();
//  }
//#endif //PATCHKEEPER_BUFFER_SENSOR_DATA

  //SEGGER_RTT_printf(0,"PPG irq!\n");

   //memset(ppg_data.data.ppg0, 0, sizeof(ppg_data.data));

  ppg_data.packet_info.sop            = SOP_BELLLABS;
  ppg_data.packet_info.timestamp_lo   = timestamp.lo;
  ppg_data.packet_info.timestamp_hi   = timestamp.hi;
  ppg_data.packet_info.logger_id      = PATCHKEEPER_PPG_ID;
  ppg_data.packet_info.length         = length;
  ppg_data.packet_info.custom         = packet_count++;

  packet_count_1++;
  if(packet_count_1 > 1) {  // discard first 2 packets due to data processing for some reason

  uint8_t int_status;
  bool data_ready;

  int_status =  maxm86161_i2c_read_from_register(MAXM86161_REG_IRQ_STATUS1);
 #if NBL_LOG_PPG == 1
    NRF_LOG_DEBUG("maxm86161 int_status: 0x%x",int_status);
 #endif

  if(int_status & MAXM86161_INT_1_FULL) {
      data_ready = maxm86161_read_samples_in_fifo(&ppg_data.data);

#if NBL_LOG_PPG == 1
      NRF_LOG_DEBUG("maxm86161 data_ready: %d",data_ready);

      if(data_ready){
       NRF_LOG_DEBUG("%lu,%lu,%lu", ppg_data.data.ppg0, ppg_data.data.ppg1, ppg_data.data.ppg2);
      }
 #endif

    }



#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
  if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
  //if (m_ble_connected || (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES)) {
      Swap_and_Save_Buffer();
  }
  //ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES); 
  memcpy((m_p_datalog_mem+m_write_offset), &ppg_data,  size); 
  m_write_offset += size;
#endif //PATCHKEEPER_BUFFER_SENSOR_DATA


#if BLE_STREAM_IMMEDIATE==1 

  if (m_ble_connected == 1 && ble_stream_ppg == STREAMING) {
    uint16_t mod =  size%4;
  if (mod != 0) {size += 4-mod;}  // this is to ensure sending 32bits aligned data over BLE
    m_main_context.ble_send_data((unsigned char*) &ppg_data, (unsigned char*) &ppg_data+size);
  }
#endif
    
    //NRF_LOG_DEBUG("ppg_measurement_timer_run: %d",ppg_measurement_timer_run);

 //if (ppg_measurement_timer_run == 1) {
 //      ppg_measurement_timer = timestamp;
 //      ppg_measurement_timer_run = 0;
 //   } else{
   
 //     uint64_t timepassed =ts64_to_uint64(&timestamp)-ts64_to_uint64(&ppg_measurement_timer);
 //     //NRF_LOG_DEBUG("time passed: %d",timepassed);

 //     if (timepassed > 32768*ppg_on_time_ms/1000) {
      
 //       patchkeeper_ppg_maxm86161_pause();

 //     }
 //}

  }

}


  //uint32_t  last_ppg_point_timestamp;
/**************************************************************************//**
 * @brief GPIO Interrupt handler for event pins.
 * Stream and save all rawdata from PPG to speedup processing 
 *****************************************************************************/
void patchkeeper_ppg_irq_handler_read_bytes(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  ret_code_t err_code;
  uint8_t data_cnt = 0;
  uint8_t valid_data_cnt = 0;
  uint8_t ovf_cnt = 0;
  patchkeeper_ppg_bytes_t   ppg_data;
  static uint8_t            packet_count = 0;
  static uint64_t           packet_count_1 = 0;

  ts64_t                    timestamp = get_timestamp();
  uint16_t                  size =   0;
  uint16_t                  length = 0; 
  
  memset(&ppg_data, 0, sizeof(patchkeeper_ppg_bytes_t));
  //NRF_LOG_DEBUG("ppg_data size: %d", sizeof(patchkeeper_ppg_bytes_t)); 
  //NRF_LOG_DEBUG("ppg_data.data size: %d", sizeof(ppg_data.data));
  
  ppg_data.packet_info.sop            = SOP_BELLLABS;  
  ppg_data.packet_info.timestamp_hi   = timestamp.hi;
  ppg_data.packet_info.timestamp_lo   = timestamp.lo;
  ppg_data.packet_info.logger_id      = PATCHKEEPER_PPG_ID;
  ppg_data.packet_info.custom         = packet_count++;
  ppg_data.packet_info.length         = length;

  memset(ppg_data.data, 0, sizeof(ppg_data.data));

#if PPG_FOR_HR_ONLY == 1
    ppg_data.packet_info.logger_id    = PATCHKEEPER_PPG_HR_ID;
#endif

#if PPG_FOR_SPO2_ONLY == 1
    ppg_data.packet_info.logger_id    = PATCHKEEPER_PPG_SPO2_ID;
#endif

  packet_count_1++;
  //NRF_LOG_DEBUG(" ppg  packet_count: %d",packet_count);
  //NRF_LOG_DEBUG(" ppg  packet_count_1: %d",packet_count_1);
if (packet_count_1 > 1) {  // discard first 2 packets due to data processing for some reason

  uint8_t int_status;

  int_status =  maxm86161_i2c_read_from_register(MAXM86161_REG_IRQ_STATUS1);
#if NBL_LOG_PPG == 1
  NRF_LOG_DEBUG("maxm86161 int_status: 0x%x",int_status);
#endif

  if(int_status & MAXM86161_INT_1_FULL) {
    err_code = maxm86161_read_all_bytes_in_fifo(&ovf_cnt,&data_cnt,ppg_data.data);
    //nrf_delay_ms(5);

    //NRF_LOG_DEBUG("start ppg read data_cnt: %d",data_cnt);
    uint8_t *block_buf = ppg_data.data;
    for (uint8_t i = 0; i < data_cnt; i++){
    uint32_t temp_data = 0x000000;
    temp_data = (block_buf[i*3 + 0] << 16 | block_buf[i*3+1] << 8 | block_buf[i*3+2] );
    uint32_t data_val = temp_data & MAXM86161_REG_FIFO_DATA_MASK;
    uint8_t tag = (temp_data >> MAXM86161_REG_FIFO_RES) & MAXM86161_REG_FIFO_TAG_MASK;

    valid_data_cnt++;
    if(tag == 0) {
      NRF_LOG_DEBUG("invalid tag and value, need to reset device");
      ppg_reset_request = true;
      patchkeeper_ppg_maxm86161_pause();

      //nrf_delay_ms(2);
      break;
    //NRF_LOG_DEBUG("cnt:%d, i:%d, tag:%d, val:%d", data_cnt,i,tag,data_val);
      };
    };
  };
   
  length = 3 * valid_data_cnt;  // total number of valid bytes, each data point has three bytes
  ppg_data.packet_info.length = length;
  size = sizeof(packet_info_t) + length;
 NRF_LOG_DEBUG("size of ppg data: %d",size);

#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
  if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
      Swap_and_Save_Buffer();
  }
  
  memcpy((m_p_datalog_mem+m_write_offset), &ppg_data, size); 
  m_write_offset += size;
  
#endif //PATCHKEEPER_BUFFER_SENSOR_DATA


#if BLE_STREAM_IMMEDIATE==1 
  if (m_ble_connected == 1 && ble_stream_ppg == STREAMING) {
   uint16_t mod =  size%4;
  if (mod != 0) {size += 4-mod;}  // this is to ensure sending 32bits aligned data over BLE
   //NRF_LOG_DEBUG("size of send ble ppg data: %d",size);
    m_main_context.ble_send_data((unsigned char*) &ppg_data, (unsigned char*) &ppg_data+size);
  }
#endif
 
//#if NBL_LOG_PPG == 1
//    NRF_LOG_DEBUG("ppg_measurement_timer_run: %d",ppg_measurement_timer_run);
//#endif

// if (ppg_measurement_timer_run == 1) {
//       ppg_measurement_timer = timestamp;
//       ppg_measurement_timer_run = 0;
//    } else{
   
//      uint64_t timepassed =ts64_to_uint64(&timestamp)-ts64_to_uint64(&ppg_measurement_timer);
//#if NBL_LOG_PPG == 1
//      NRF_LOG_DEBUG("ppg time passed: %d",timepassed);
//      NRF_LOG_DEBUG("ppg on time: %d",32768*ppg_on_time_ms/1000);
//#endif

//      if (timepassed > 32768*ppg_on_time_ms/1000) {
//         NRF_LOG_DEBUG("pausing ppg...");

//        patchkeeper_ppg_maxm86161_pause();
//        //nrf_delay_us(100);

//      }
//    }

  }

}



#endif

/*************************************
 ** @SAADC Based Measurements       **
 *************************************/
// SAADC when sampled, samples all channels
// It therefore makes sense to use a single
// app_timer for all sensors.
// Start by measuring everything on each
// call.

APP_TIMER_DEF(m_sensor_timer);

void patchkeeper_adc_init(void) {

// Configure and Enable the SAADC HW
patchkeeper_saadc_init((nrf_saadc_value_t *) &m_adc_data);
// Create a timer to sample the ADCs
// But don't start it yet.
app_timer_create(&m_sensor_timer, APP_TIMER_MODE_REPEATED, patchkeeper_saadc_read_adc);
}

// Start sampling ADC in every 10 ms
void patchkeeper_start_sampling_adc(void) {
//    uint16_t adc_oversample = (int) pow(2.0, NRF_SAADC->OVERSAMPLE);
  app_timer_start(m_sensor_timer, APP_TIMER_TICKS(PATCHKEEPER_DEFAULT_ADC_SAMPLE_INTERVAL_MS), &m_adc_data);
}

// Stop sampling ADC
void patchkeeper_stop_sampling_adc (void) {
  app_timer_stop(m_sensor_timer);
}

void saadc_measurements_report(void){

  NRF_LOG_DEBUG("GSR    ADC reads 0x%04x", m_adc_data.gsr);
  NRF_LOG_DEBUG("STRAIN ADC reads 0x%04x", m_adc_data.strain);
  NRF_LOG_DEBUG("PULSE  ADC reads 0x%04x", m_adc_data.pulse);
  NRF_LOG_DEBUG("TEMP   ADC reads 0x%04x", m_adc_data.temp);
  NRF_LOG_DEBUG("BATT   ADC reads 0x%04x", m_adc_data.batt);

}



/*************************************
 ** @GSR Measurement                **
 *************************************/

APP_TIMER_DEF(m_gsr_timer);
m_sensor_timer_state_t m_gsr_timer_state = UNINITIALISED;
patchkeeper_gsr_data_t m_patchkeeper_gsr_data;


void patchkeeper_config_gsr(ble_mode_t mode, uint8_t sample_interval_ms){

  // Update values.
  m_patchkeeper_sensor_config.gsr_sensor.mode = mode;
  m_patchkeeper_sensor_config.gsr_sensor.sample_interval_ms = sample_interval_ms;

  // Trigger the Changes.
  patchkeeper_gsr_trigger();
};

void patchkeeper_gsr_init (void) {

  if (m_gsr_timer_state == UNINITIALISED) {
    app_timer_create(&m_gsr_timer, APP_TIMER_MODE_REPEATED,&patchkeeper_gsr_isr);
    m_gsr_timer_state = IDLE;
  }
}

void patchkeeper_gsr_trigger (void) {
  if ((m_gsr_timer_state == IDLE) && (m_patchkeeper_sensor_config.gsr_sensor.mode == STREAMING)) {
    app_timer_start(m_gsr_timer, APP_TIMER_TICKS(m_patchkeeper_sensor_config.gsr_sensor.sample_interval_ms), NULL);
    m_gsr_timer_state = RUNNING;
    NRF_LOG_DEBUG("Starting GSR Timer");
  }

  if ((m_gsr_timer_state == RUNNING) && (m_patchkeeper_sensor_config.gsr_sensor.mode == OFF)) {
    app_timer_stop(m_gsr_timer);
    m_gsr_timer_state = IDLE;
  }

  if ((m_gsr_timer_state == IDLE) && (m_patchkeeper_sensor_config.gsr_sensor.mode == OFF)) {
    NRF_LOG_DEBUG("GSR Timer Disabled");
  }

};


void patchkeeper_gsr_isr(void* p_dummy) {

  // Data is in m_adc_data.gsr;
  static uint8_t item_count = 0;
  static uint8_t packet_count = 0;
  uint16_t   size = sizeof(patchkeeper_gsr_data_t);


  ts64_t timestamp = get_timestamp();

  m_patchkeeper_gsr_data.data[item_count].ts_lo = timestamp.lo;
  m_patchkeeper_gsr_data.data[item_count].value =
  patchkeeper_get_voltage_from_adc_value(m_adc_data.gsr,PATCHKEEPER_GSR_ADC_GAIN);

  item_count++;

  NRF_LOG_DEBUG("gsr item count: %d", item_count);
  NRF_LOG_DEBUG("GSR Sensor Voltage value:"
  NRF_LOG_FLOAT_MARKER,NRF_LOG_FLOAT(m_patchkeeper_gsr_data.data[item_count].value));

  if (item_count == PATCHKEEPER_GSR_BUFFER_SIZE) {


    m_patchkeeper_gsr_data.packet_info.sop            = SOP_BELLLABS;
    m_patchkeeper_gsr_data.packet_info.timestamp_lo   = timestamp.lo;
    m_patchkeeper_gsr_data.packet_info.timestamp_hi   = timestamp.hi;
    m_patchkeeper_gsr_data.packet_info.logger_id      = PATCHKEEPER_GSR_ID;
    m_patchkeeper_gsr_data.packet_info.length         = PATCHKEEPER_GSR_DATA_LENGTH_BYTES;
    m_patchkeeper_gsr_data.packet_info.custom         = packet_count++;

    uint8_t* p_start = (uint8_t*) &m_patchkeeper_gsr_data;
    uint8_t* p_end   = p_start + sizeof(patchkeeper_gsr_data_t);

#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
  if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
      Swap_and_Save_Buffer();
      }

  //ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES);
  memcpy((m_p_datalog_mem+m_write_offset), p_start, size);
  m_write_offset += size;
#endif  //PATCHKEEPER_BUFFER_SENSOR_DATA

#if BLE_STREAM_IMMEDIATE==1
  if (m_ble_connected == 1) {
    m_main_context.ble_send_data(p_start, p_end);
}
#endif

    item_count = 0;
  }
}


/*************************************
 ** @STRAIN Measurement            **
 *************************************/

APP_TIMER_DEF(m_strain_timer);

m_sensor_timer_state_t m_strain_timer_state = UNINITIALISED;
patchkeeper_strain_data_t m_patchkeeper_strain_data;

void patchkeeper_config_strain(ble_mode_t mode, uint8_t sample_interval_ms){

  // Update values.
  m_patchkeeper_sensor_config.strain_sensor.mode = mode;
  m_patchkeeper_sensor_config.strain_sensor.sample_interval_ms = sample_interval_ms;

  // Trigger the Changes.
  patchkeeper_strain_trigger();
};

void patchkeeper_strain_init (void) {

  if (m_strain_timer_state == UNINITIALISED) {
    app_timer_create(&m_strain_timer, APP_TIMER_MODE_REPEATED,&patchkeeper_strain_isr);
    m_strain_timer_state = IDLE;
  }
}

void patchkeeper_strain_trigger (void) {
  if ((m_strain_timer_state == IDLE) && (m_patchkeeper_sensor_config.strain_sensor.mode == STREAMING)) {
    app_timer_start(m_strain_timer, APP_TIMER_TICKS(m_patchkeeper_sensor_config.strain_sensor.sample_interval_ms), NULL);
    m_strain_timer_state = RUNNING;
    NRF_LOG_DEBUG("Starting Strain Timer");
  }

  if ((m_strain_timer_state == RUNNING) && (m_patchkeeper_sensor_config.strain_sensor.mode == OFF)) {
    app_timer_stop(m_strain_timer);
    m_strain_timer_state = IDLE;
  }

  if ((m_strain_timer_state == IDLE) && (m_patchkeeper_sensor_config.strain_sensor.mode == OFF)) {
    NRF_LOG_DEBUG("Strain Timer Disabled");
  }

};

void patchkeeper_strain_isr(void* p_dummy) {

  // Data is in m_adc_data.strain;
  static uint8_t item_count = 0;
  static uint8_t packet_count = 0;
  uint16_t   size = sizeof(patchkeeper_strain_data_t);

  ts64_t timestamp = get_timestamp();

  m_patchkeeper_strain_data.data[item_count].ts_lo = timestamp.lo;
  m_patchkeeper_strain_data.data[item_count].value =
  patchkeeper_get_voltage_from_adc_value(m_adc_data.strain,PATCHKEEPER_STRAIN_ADC_GAIN);

   NRF_LOG_DEBUG("Strain Sensor Voltage value:"
    NRF_LOG_FLOAT_MARKER,NRF_LOG_FLOAT(m_patchkeeper_strain_data.data[item_count].value));

  item_count++;
  NRF_LOG_DEBUG("strain item count: %d", item_count);

  if (item_count == PATCHKEEPER_STRAIN_BUFFER_SIZE) {

//#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
//  //if (m_ble_connected || (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES)) {
//    if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
//        Swap_and_Save_Buffer();
//      }
//#endif // PATCHKEEPER_BUFFER_SENSOR_DATA

    m_patchkeeper_strain_data.packet_info.sop            = SOP_BELLLABS;
    m_patchkeeper_strain_data.packet_info.timestamp_lo   = timestamp.lo;
    m_patchkeeper_strain_data.packet_info.timestamp_hi   = timestamp.hi;
    m_patchkeeper_strain_data.packet_info.logger_id      = PATCHKEEPER_STRAIN_ID;
    m_patchkeeper_strain_data.packet_info.length         = PATCHKEEPER_STRAIN_DATA_LENGTH_BYTES;
    m_patchkeeper_strain_data.packet_info.custom         = packet_count++;

    uint8_t* p_start = (uint8_t*) &m_patchkeeper_strain_data;
    uint8_t* p_end   = p_start + sizeof(patchkeeper_strain_data_t);


#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
  //if (m_ble_connected || (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES)) {
    if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
        Swap_and_Save_Buffer();
      }

  //ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES);
  memcpy((m_p_datalog_mem+m_write_offset), p_start, size);
  m_write_offset += size;
#endif  //PATCHKEEPER_BUFFER_SENSOR_DATA

#if BLE_STREAM_IMMEDIATE==1
  if (m_ble_connected == 1) {
    m_main_context.ble_send_data(p_start, p_end);
}
#endif

    item_count = 0;
  }
}


/*************************************
 ** @PULSE Measurement            **
 *************************************/

APP_TIMER_DEF(m_pulse_timer);
m_sensor_timer_state_t m_pulse_timer_state = UNINITIALISED;
patchkeeper_pulse_data_t m_patchkeeper_pulse_data;

void patchkeeper_config_pulse(ble_mode_t mode, uint8_t sample_interval_ms){

  // Update values.
  m_patchkeeper_sensor_config.pulse_sensor.mode = mode;
  m_patchkeeper_sensor_config.pulse_sensor.sample_interval_ms = sample_interval_ms;

  // Trigger the Changes.
  patchkeeper_pulse_trigger();
};

void patchkeeper_pulse_init (void) {

  if (m_pulse_timer_state == UNINITIALISED) {
    app_timer_create(&m_pulse_timer, APP_TIMER_MODE_REPEATED,&patchkeeper_pulse_isr);
    m_pulse_timer_state = IDLE;
  }
}

void patchkeeper_pulse_trigger (void) {
  if ((m_pulse_timer_state == IDLE) && (m_patchkeeper_sensor_config.pulse_sensor.mode == STREAMING)) {
    app_timer_start(m_pulse_timer, APP_TIMER_TICKS(m_patchkeeper_sensor_config.pulse_sensor.sample_interval_ms), NULL);
    m_pulse_timer_state = RUNNING;
    NRF_LOG_DEBUG("Starting Pulse Timer");
  }

  if ((m_pulse_timer_state == RUNNING) && (m_patchkeeper_sensor_config.pulse_sensor.mode == OFF)) {
    app_timer_stop(m_pulse_timer);
    m_pulse_timer_state = IDLE;
  }

  if ((m_pulse_timer_state == IDLE) && (m_patchkeeper_sensor_config.pulse_sensor.mode == OFF)) {
    NRF_LOG_DEBUG("Pulse Timer Disabled");
  }

};

void patchkeeper_pulse_isr(void* p_dummy) {

  // Data is in m_adc_data.pulse;
  static uint8_t item_count = 0;
  static uint8_t packet_count = 0;
  uint16_t   size = sizeof(patchkeeper_pulse_data_t);

  ts64_t timestamp = get_timestamp();
  m_patchkeeper_pulse_data.data[item_count].ts_lo = timestamp.lo;
  m_patchkeeper_pulse_data.data[item_count].value =
    patchkeeper_get_voltage_from_adc_value(m_adc_data.pulse,PATCHKEEPER_PULSE_ADC_GAIN);

   NRF_LOG_DEBUG("Pulse Sensor Voltage value:"
    NRF_LOG_FLOAT_MARKER,NRF_LOG_FLOAT(m_patchkeeper_pulse_data.data[item_count].value));

  item_count++;
  NRF_LOG_DEBUG("pulse item count: %d", item_count);

  if (item_count == PATCHKEEPER_PULSE_BUFFER_SIZE) {

#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
  //if (m_ble_connected || (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES)) {
    if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {  
      Swap_and_Save_Buffer();
      }
#endif // PATCHKEEPER_BUFFER_SENSOR_DATA

    m_patchkeeper_pulse_data.packet_info.sop            = SOP_BELLLABS;
    m_patchkeeper_pulse_data.packet_info.timestamp_lo   = timestamp.lo;
    m_patchkeeper_pulse_data.packet_info.timestamp_hi   = timestamp.hi;
    m_patchkeeper_pulse_data.packet_info.logger_id      = PATCHKEEPER_PULSE_ID;
    m_patchkeeper_pulse_data.packet_info.length         = PATCHKEEPER_PULSE_DATA_LENGTH_BYTES;
    m_patchkeeper_pulse_data.packet_info.custom         = packet_count++;

    uint8_t* p_start = (uint8_t*) &m_patchkeeper_pulse_data;
    uint8_t* p_end   = p_start + sizeof(patchkeeper_pulse_data_t);

#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
  ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES);
  memcpy((m_p_datalog_mem+m_write_offset), p_start, size);
  m_write_offset += size;
#endif  //PATCHKEEPER_BUFFER_SENSOR_DATA

#if BLE_STREAM_IMMEDIATE==1
  if (m_ble_connected == 1) {
    m_main_context.ble_send_data(p_start, p_end);
    NRF_LOG_DEBUG("pulse packet count: %d", packet_count);
}
#endif

    item_count = 0;
  }
}

/*************************************
 ** @TEMPERATURE Measurement        **
 *************************************/
APP_TIMER_DEF(m_temp_timer);

m_sensor_timer_state_t m_temp_timer_state = UNINITIALISED;
patchkeeper_temp_data_t m_patchkeeper_temp_data;

void patchkeeper_config_temp(ble_mode_t mode, uint8_t sample_interval_ms){

  // Update values.
  m_patchkeeper_sensor_config.temp_sensor.mode = mode;
  m_patchkeeper_sensor_config.temp_sensor.sample_interval_ms = sample_interval_ms;

  // Trigger the Changes.
  patchkeeper_temp_trigger();
};

void patchkeeper_temp_init (void) {

  if (m_temp_timer_state == UNINITIALISED) {
    app_timer_create(&m_temp_timer, APP_TIMER_MODE_REPEATED,&patchkeeper_temp_isr);
    m_temp_timer_state = IDLE;
  }
}

void patchkeeper_temp_trigger (void) {
  if ((m_temp_timer_state == IDLE) && (m_patchkeeper_sensor_config.temp_sensor.mode == STREAMING)) {
    app_timer_start(m_temp_timer, APP_TIMER_TICKS(m_patchkeeper_sensor_config.temp_sensor.sample_interval_ms), NULL);
    m_temp_timer_state = RUNNING;
    NRF_LOG_DEBUG("Starting Temperature Timer");
  }

  if ((m_temp_timer_state == RUNNING) && (m_patchkeeper_sensor_config.temp_sensor.mode == OFF)) {
    app_timer_stop(m_temp_timer);
    m_temp_timer_state = IDLE;
  }

  if ((m_temp_timer_state == IDLE) && (m_patchkeeper_sensor_config.temp_sensor.mode == OFF)) {
    NRF_LOG_DEBUG("Temp Timer Disabled");
  }

};


void patchkeeper_temp_isr(void* p_dummy) {

  // Data is in m_adc_data.temp;
  static uint8_t item_count = 0;
  static uint8_t packet_count = 0;
  uint16_t   size = sizeof(patchkeeper_temp_data_t);

  ts64_t timestamp = get_timestamp();

  float_t voltage = patchkeeper_get_voltage_from_adc_value(m_adc_data.temp,PATCHKEEPER_TEMP_ADC_GAIN);

  m_patchkeeper_temp_data.data[item_count].ts_lo = timestamp.lo;
  m_patchkeeper_temp_data.data[item_count].value = voltage;

  NRF_LOG_DEBUG("Temp Sensor Voltage value:"
    NRF_LOG_FLOAT_MARKER,NRF_LOG_FLOAT(m_patchkeeper_temp_data.data[item_count].value));

  item_count++;
  NRF_LOG_DEBUG("temperature item count: %d", item_count);

  if (item_count == PATCHKEEPER_TEMP_BUFFER_SIZE) {

//#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
//  //if (m_ble_connected || (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES)) {
//  if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
//      Swap_and_Save_Buffer();
//      }
//#endif // PATCHKEEPER_BUFFER_SENSOR_DATA

    m_patchkeeper_temp_data.packet_info.sop            = SOP_BELLLABS;
    m_patchkeeper_temp_data.packet_info.timestamp_lo   = timestamp.lo;
    m_patchkeeper_temp_data.packet_info.timestamp_hi   = timestamp.hi;
    m_patchkeeper_temp_data.packet_info.logger_id      = PATCHKEEPER_TEMP_ID;
    m_patchkeeper_temp_data.packet_info.length         = PATCHKEEPER_TEMP_DATA_LENGTH_BYTES;
    m_patchkeeper_temp_data.packet_info.custom         = packet_count++;

    uint8_t* p_start = (uint8_t*) &m_patchkeeper_temp_data;
    uint8_t* p_end   = p_start + sizeof(patchkeeper_temp_data_t);

#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
  //if (m_ble_connected || (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES)) {
  if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
      Swap_and_Save_Buffer();
      }

  //ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES);
  memcpy((m_p_datalog_mem+m_write_offset), p_start, size);
  m_write_offset += size;
#endif  //PATCHKEEPER_BUFFER_SENSOR_DATA

#if BLE_STREAM_IMMEDIATE==1
    if (m_ble_connected == 1) {
    m_main_context.ble_send_data(p_start, p_end);
}
#endif

    item_count = 0;
  }
}

/*************************************
 ** @Battery Measurement        **
 *************************************/
 #if BOARD_HAS_BATMON == 1

APP_TIMER_DEF(m_batt_timer);

m_sensor_timer_state_t m_batt_timer_state = UNINITIALISED;

patchkeeper_batt_data_t m_patchkeeper_batt_data;

//configure battery monitor enable pin and setup timer
void patchkeeper_batt_init (void) {
    nrf_gpio_cfg_output(BAT_MON_EN);
    nrf_gpio_pin_set(BAT_MON_EN);

  if (m_batt_timer_state == UNINITIALISED) {
    app_timer_create(&m_batt_timer, APP_TIMER_MODE_REPEATED,&patchkeeper_batt_isr);
    m_batt_timer_state = IDLE;
  }
}

// Enable the Battery Measurement Circuitry and start timer
void patchkeeper_batt_trigger (void) {

  // start or stop timer for battery depending on battery.mode
  if ((m_batt_timer_state == IDLE) && (m_patchkeeper_sensor_config.battery.mode == STREAMING)) {
    // enable BAT_MON_EN pin
    nrf_gpio_pin_set(BAT_MON_EN);
    app_timer_start(m_batt_timer, APP_TIMER_TICKS(m_patchkeeper_sensor_config.battery.sample_interval_ms), NULL);
    m_batt_timer_state = RUNNING;
    NRF_LOG_DEBUG("Starting Battery Timer");
  }

  if ((m_batt_timer_state == RUNNING) && (m_patchkeeper_sensor_config.battery.mode == OFF)) {
      // Disable BAT_MON_EN pin
    nrf_gpio_pin_clear(BAT_MON_EN);
    app_timer_stop(m_batt_timer);
    m_batt_timer_state = IDLE;
    NRF_LOG_DEBUG("Battery Timer Disabled");

  }

  if ((m_batt_timer_state == IDLE) && (m_patchkeeper_sensor_config.battery.mode == OFF)) {
       // Disable BAT_MON_EN pin
    nrf_gpio_pin_clear(BAT_MON_EN);
    NRF_LOG_DEBUG("Battery Timer Disabled");
  }

};

float patchkeeper_convert_adc_voltage_to_vbatt (float voltage) {

  float r5 = BATT_VOLTAGE_DIVIDER_R1;
  float r6 = BATT_VOLTAGE_DIVIDER_R2;
  float resistor_divider = r6 / (r5 + r6);

  float vbatt   =  voltage / resistor_divider;

  return vbatt;
}

void patchkeeper_batt_isr(void* p_dummy) {
  uint8_t                   battery_level_Percent = 0;
  static float              filtered_vbatt;
  uint16_t   size = sizeof(patchkeeper_batt_data_t); // this size auto padded to 20  bytes because of struct


//#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
//  //if (m_ble_connected || (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES)) {
//  if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
//      Swap_and_Save_Buffer();
//      }
//#endif // PATCHKEEPER_BUFFER_SENSOR_DATA

  // Data is in m_adc_data.batt;
  static uint8_t packet_count = 0;
  ts64_t timestamp = get_timestamp();

 // Figures taken from discharge curve measured overnight 27th November 2018
  // Entries correspond to           0%,  10%,  20%,  30%,  40%,  50%,  60%,  70%,  80%,  90%, 100%
//  const float battery_lookup [] = {3.20, 3.80, 3.95, 3.99, 4.04, 4.10, 4.15, 4.20, 4.23, 4.24, 4.25};

  // Entries correspond to          0%,  10%,  20%,  30%,  40%,  50%,  60%,  70%,  80%,  90%, 100%
 const float battery_lookup [] = {3.30, 3.67, 3.73, 3.77, 3.81, 3.86, 3.89, 3.96, 4.04, 4.11, 4.20};

//  m_patchkeeper_batt_data.data = m_adc_data.batt;

float voltage = patchkeeper_get_voltage_from_adc_value(m_adc_data.batt,PATCHKEEPER_BATT_ADC_GAIN);
float vbatt = patchkeeper_convert_adc_voltage_to_vbatt(voltage);

  // Restrict voltage to within a reasonable range. The readings are
  // not very accurate and so vary out-of-range a lot.
  filtered_vbatt = vbatt;

#if NBL_LOG_BATT == 1
  NRF_LOG_DEBUG("vbatt:"NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(vbatt));
#endif

  if (filtered_vbatt > 4.20) {
    filtered_vbatt = 4.20;
  } else if (filtered_vbatt < 3.3){
    filtered_vbatt = 3.3;
  }

  // Convert to percent from voltage - 4.25V is full
  // Use the lookup table
  uint8_t decile = 1;
  while ((filtered_vbatt > battery_lookup[decile]) && (decile < 10))
    decile++;

  float vlo   = battery_lookup[decile-1];
  float vhi   = battery_lookup[decile];

  ASSERT((vhi-vlo) != 0);

  battery_level_Percent = (uint8_t) (10.0 * ((decile-1) + ((filtered_vbatt-vlo)/(vhi-vlo))));
   // For debugging and discharge analysis record the actual voltage
  // To save this in uint8_t format: -
  //      Battery Voltage should be in range c. 3V2 to 4V2.
  //      Constrain 8-bit variable to voltage range 3v0 to 4v25
  //        - Store in variable with range 0->255
  //        - resolution is 1/200V (0.005V) Offset is 3.0v
  float coded_vbatt = filtered_vbatt - 3.0;

  coded_vbatt *= 200;

  uint8_t coded_vbatt_8 = (uint8_t) coded_vbatt;

  m_patchkeeper_batt_data.data[0] = battery_level_Percent;
  m_patchkeeper_batt_data.data[1] = coded_vbatt_8;

#if NBL_LOG_BATT == 1
 NRF_LOG_DEBUG("m_patchkeeper_batt_data.data[0]: %d", m_patchkeeper_batt_data.data[0]);
 NRF_LOG_DEBUG("m_patchkeeper_batt_data.data[1]: %d", m_patchkeeper_batt_data.data[1]);
#endif

// format all information and data, and send with ble
  m_patchkeeper_batt_data.packet_info.sop            = SOP_BELLLABS;
  m_patchkeeper_batt_data.packet_info.timestamp_lo   = timestamp.lo;
  m_patchkeeper_batt_data.packet_info.timestamp_hi   = timestamp.hi;
  m_patchkeeper_batt_data.packet_info.logger_id      = PATCHKEEPER_BATT_ID;
  m_patchkeeper_batt_data.packet_info.length         = PATCHKEEPER_BATT_DATA_LENGTH_BYTES;
  m_patchkeeper_batt_data.packet_info.custom         = packet_count++;

  uint8_t* p_start = (uint8_t*) &m_patchkeeper_batt_data;
  uint8_t* p_end   = p_start + size;

#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
  //if (m_ble_connected || (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES)) {
  if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
      Swap_and_Save_Buffer();
      }

  //ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES);
  memcpy((m_p_datalog_mem+m_write_offset), p_start, size);
  m_write_offset += size;
#endif  //PATCHKEEPER_BUFFER_SENSOR_DATA

#if BLE_STREAM_IMMEDIATE==1
  if (m_ble_connected == 1) {
    m_main_context.ble_send_data(p_start, p_end);
}
#endif

}

#endif


/*************************************
 ** @Audio         **
 *************************************/
/** @brief Audio Sampling Timer Handle.
 Sets the start and stop times for Audio Sampling
*/
#if BOARD_HAS_PDM_MIC == 1

APP_TIMER_DEF(m_timer_audio);             /**< Handler for repeated timer used to sample audio. */
static uint8_t    m_send_audio_data = 1; // flag for audio mean/peak to be send over BLE
uint16_t m_audio_sample_period_ms = PATCHKEEPER_DEFAULT_AUDIO_SAMPLE_INTERVAL_MS;

void patchkeeper_audio_timer_config (void) {
      ret_code_t err_code;
    // Create the Audio timer
    err_code = app_timer_create(&m_timer_audio,
                                APP_TIMER_MODE_REPEATED,
                                patchkeeper_audio_timer_handle);
    APP_ERROR_CHECK(err_code);

    // Start the Audio timer.
    err_code = app_timer_start(m_timer_audio, APP_TIMER_TICKS(m_audio_sample_period_ms), NULL);
    APP_ERROR_CHECK(err_code);
}

//patchkeeper_audio_timer_handle only set the m_send_audio_data flag to indicate if audio data need to
//be sent over ble. Audio data is continously recorded to buffers controlled by pdm.
void patchkeeper_audio_timer_handle (void * p_context) {

 m_send_audio_data = 1; //set the flag

//    uint32_t        next_interval_ms = 0;
//
//    switch (audio_state) {
//    case (0)  : next_interval_ms = (m_audio_sample_size_ms);
//                audio_state = 1;
//                break;
//    case (1)  : next_interval_ms = (m_audio_sample_period_ms - m_audio_sample_size_ms);
//                audio_state = 0;
//                break;
//    }
//
//    if (0 == m_audio_sample_period_ms) {
//      app_timer_stop(m_timer_audio);
//    } else {
//    app_timer_start(m_timer_audio, APP_TIMER_TICKS(next_interval_ms), NULL);
//    NRF_LOG_DEBUG("m_timer_audio start for %d ms.",next_interval_ms);
//    }
//
//    patchkeeper_audio_pdm_ctrl(audio_state);

//    if (audio_state == 0) {
//    ble_send_audio_data();
//    }

}


/**
 * Audio PDM Control Handler
 */

// There are two Audio handlers. The first is triggered by the timer and responsible
// for starting and stopping the PDM module.
// Starting the PDM will result in PDM Interrupts being generated to do any combination of
// the following: -
//   - Request a new buffer for EasyDMA
//   - Release a buffer from EasyDMA
//   - Communicate an ERROR condition.
//
// According to NRF5-SDK (v15.2) the correct sequence is: -
//    - Initialise the PDM (nrfx_pdm_init(...)) - this is performed in pdm_init called from main.
//    - Start the PDM (nrfx_pdm_start()). This generates an interrupt to provide the location of
//      the buffer for EasyDMA
//    - Service the Interrupt providing the first buffer
//    - Service the next Interrupt providing a new buffer
//    - Continue servicing interrupts providing new bufferes, releasing old buffers, checking errors.
//    - Stop the PDM (nrfx_pdm_stop())

// Here the RTC triggered interrupt handler handles the PDM Start and Stop.This function was used in
// previous version from Gecko. 

//void patchkeeper_audio_pdm_ctrl(uint8_t turn_on) {
//  ret_code_t ret_code;
//
//  if (turn_on) {
//
//    // Start the PDM if command to turn on
//#if NBL_LOG_PDM == 1
//    NRF_LOG_DEBUG("Starting PDM at RTC2 time %d\n", (uint32_t) NRF_RTC2->COUNTER);
//    NRF_LOG_FLUSH();
//#endif
//    ret_code = nrfx_pdm_start();
//
//    if (ret_code != 0) {
//      __BKPT();
//#if NBL_LOG_PDM == 1
//      NRF_LOG_DEBUG("PDM Already Started");
//      NRF_LOG_FLUSH();
//#endif
//    }
//
//  } else {
//#if NBL_LOG_PDM == 1
//    NRF_LOG_DEBUG("Stopping PDM at RTC2 time %d\n", (uint32_t) NRF_RTC2->COUNTER);
//    NRF_LOG_FLUSH();
//#endif
//    ret_code = nrfx_pdm_stop();
//
//    if (ret_code != 0) {
//      __BKPT();
//#if NBL_LOG_PDM == 1
//      NRF_LOG_DEBUG("PDM Already Stopping");
//      NRF_LOG_FLUSH();
//#endif
//    }
//  }
//}


/**
 * Audio PDM Event Handler
 */
// This Interrupt handles the Audio PDM Events, i.e.
//    - Providing new buffers for EasyDMA
//    - Processing buffered Audio Data and storing results.
//    - Check for Errors.

void patchkeeper_audio_handler (nrfx_pdm_evt_t const * const p_evt) {
  ret_code_t        ret_code;
  audio_data_t      audio_data;
  static uint8_t   packet_count = 0;
  static uint64_t   packet_count_1 = 0;
  uint16_t          size = sizeof(audio_data_t);
  static uint8_t    next_audio_buffer = 0;



#if NBL_LOG_PDM == 1
   static  uint16_t  event_count = 0;
   event_count++;
   NRF_LOG_DEBUG("audio_datalogger_handler event_count: %d",event_count);
  NRF_LOG_DEBUG("PDM Event Handler at ms = %d", (uint32_t) (NRF_RTC2->COUNTER)*1000/32768); 
  NRF_LOG_DEBUG("p_evt->buffer_requested = %d", p_evt->buffer_requested); 
  NRF_LOG_DEBUG("p_evt->buffer_released = 0x%08x", p_evt->buffer_released); 
  NRF_LOG_DEBUG("p_evt->error = %d", p_evt->error);   
  NRF_LOG_DEBUG("nrfx_pdm_enable_check returns: %d\n", (int) nrfx_pdm_enable_check()); 
   NRF_LOG_FLUSH();
#endif

  if (p_evt->error) {
    __BKPT();
  }


  if (p_evt->buffer_requested) {

    // Note that the PDM driver will call this function twice with NULL p_evt->buffer_released. 
      // The first time is to get the PDM started with the first buffer and the second time is to obtain 
      // the location of the NEXT buffer for data storage. So if continous capture is required, then at 
      // least two buffers are needed with the PDM ping-ponging between them. For this to work, the data 
      // needs to be able to be processed / transmitted / stored in the time that it takes to fill another 
      // buffer.  
 
      // Point the PDM EasyDMA to the next buffer 
      ret_code = nrfx_pdm_buffer_set((int16_t*) audiodata_memory[next_audio_buffer],AUDIO_MEMORY_BUFFER_SIZE/sizeof(audio_memory_t));  
      APP_ERROR_CHECK(ret_code); 
#if NBL_LOG_PDM == 1
      NRF_LOG_DEBUG("PDM Event Handler Providing Next Buffer from 0x%08x to 0x%08x (%d elements)\n",  
                          audiodata_memory[next_audio_buffer], 
                          ((int) audiodata_memory[next_audio_buffer]) + AUDIO_MEMORY_BUFFER_SIZE,
                          AUDIO_MEMORY_BUFFER_SIZE/sizeof(audio_memory_t)
                   );
      NRF_LOG_FLUSH();
#endif  
      if (++next_audio_buffer == AUDIO_MEMORY_NUM_BUFFERS) {
        next_audio_buffer = 0;
      }
  }  
  
  if (p_evt->buffer_released != NULL) {
//    comment out after debug 
//    ASSERT((audio_memory_t *) p_evt->buffer_released == (audio_memory_t *) audiodata_memory[0]) 


    audio_memory_t * p_buffer_released = (audio_memory_t *) p_evt->buffer_released;

  packet_count_1++;
  NRF_LOG_DEBUG("audio_datalogger_handler packet_count_1: %d",packet_count_1);
 if (packet_count_1 > 5) { // discard first several seconds
 
// save full audio trace
#if (USE_SDCARD + SAVE_FULL_AUDIO) == 2
 Save_Audio_Buffer(p_buffer_released);
 #endif // USE_SDCARD + SAVE_FULL_AUDIO

#if STREAM_FULL_AUDIO == 1
#if BLE_STREAM_IMMEDIATE==1
  if (m_ble_connected == 1) {
    uint8_t* p_start = (uint8_t*) p_buffer_released;
    uint8_t* p_end   = p_start + AUDIO_MEMORY_BUFFER_SIZE;
    m_main_context.ble_send_data(p_start, p_end);
  }
#endif  //end of BLE_STREAM_IMMEDIATE
#endif  //end of STREAM_FULL_AUDIO

#if CALC_AUDIO_MEAN == 1

// process audio data for peak and mean, stereo audio from both chanels are averaged together like a single channel
  if (m_send_audio_data == 1) {

#if NBL_LOG_PDM == 1 
//    NRF_LOG_DEBUG("audio timestamp_lo in ms: %d", timestamp.lo*1000/32768);
    NRF_LOG_DEBUG("audio buffer released : = 0x%08x",(audio_memory_t *) p_evt->buffer_released);
    NRF_LOG_DEBUG("start processing time at %d ms.", (uint64_t) (NRF_RTC2->COUNTER)*1000/32768);
//    nrf_delay_ms(20);
    NRF_LOG_FLUSH();
#endif

//    audio_memory_t * p_buffer_released = (audio_memory_t *) p_evt->buffer_released;
    ts64_t timestamp                        = get_timestamp();
    audio_data.packet_info.sop              = SOP_BELLLABS; 
    audio_data.packet_info.logger_id        = PATCHKEEPER_AUDIO_ID; 
    audio_data.packet_info.length           = PATCHKEEPER_AUDIO_RESULT_DATA_LENGTH_BYTES*sizeof(audio_memory_t); 
    audio_data.packet_info.custom           = packet_count++; 
    audio_data.packet_info.timestamp_lo     = timestamp.lo; 
    audio_data.packet_info.timestamp_hi     = timestamp.hi;

// from here onward can be replaced with code copied from Grecko if there are problems with audio trace
    int64_t mean = 0;
    int16_t peak = 0;

    for (int i=0; i< AUDIO_MEMORY_BUFFER_SIZE/sizeof(audio_memory_t); i++) {
      // Take the mean and peak of the absolute value (only interested in magnitude)
      // TODO: This assumes audio data always centred on 0?
      int16_t sample = *(p_buffer_released+i);
      
       if (sample == -32768) {sample++;}

      int16_t abs_sample = abs(sample);
      
//      if (i < 10 | i >AUDIO_MEMORY_BUFFER_SIZE/sizeof(audio_memory_t) -10) {
//      NRF_LOG_DEBUG(" i : sample: add = %d : %d : 0x%08x",i,sample, p_buffer_released+i);
//            nrf_delay_ms(5); 
//        }
      mean += abs_sample;
      if (abs_sample > peak) {
        peak = abs_sample;
      }
    }

    mean = mean / (AUDIO_MEMORY_BUFFER_SIZE/sizeof(audio_memory_t));
//replace code up to here with following averaging with 3 small blocks

//// the following are replacement code, use 3 small blocks to average data
//    int64_t mean = 0; 
//    int16_t peak = 0; 
//
//    int16_t block_size = 32; 
//    int32_t block_count = 0;      // used for calucation of mean, 
//
//    int16_t pre_block[3] = {0};
//    int16_t cur_block[3] = {0};
//    int16_t pos_block[3] = {0};
//         
//    int64_t pre_block_mean = 0;
//    int64_t pre_block_abs_mean = 0;
//    int16_t pre_block_peak = 0; 
//
//    int64_t cur_block_mean = 0;
//    int64_t cur_block_abs_mean = 0;
//    int16_t cur_block_peak = 0; 
//          
//    int64_t pos_block_mean = 0;
//    int64_t pos_block_abs_mean = 0;
//    int16_t pos_block_peak = 0;
//    int16_t pos_block_mean_threshold = 5000;
////process previous blocks
//   for (int j=0; j < block_size; j++) { 
//      int16_t sample = *(p_buffer_released - block_size + j);
//      if (sample == -32768) { sample ++;} //overflow at -32768 for abs
//      int16_t abs_sample = abs(sample); 
//      pre_block_mean += sample;
//      pre_block_abs_mean += abs_sample;
//
//      if (abs_sample > pre_block_peak) { 
//          pre_block_peak = abs_sample;
//          } 
//      }
//    pre_block_mean = pre_block_mean / block_size ; 
//    pre_block_abs_mean = pre_block_abs_mean / block_size ; 
//
//    if (abs(pre_block_mean) > pos_block_mean_threshold ) {
//        pre_block[0] = 1;
//        }
//
//    pre_block[1] = pre_block_abs_mean;
//    pre_block[2] = pre_block_peak;
////   NRF_LOG_DEBUG("pre_block info:mean:peak = %d:%d:%d",pre_block[0],pre_block[1],pre_block[2]);
////   nrf_delay_ms(20);
//
////process current blocks
////    NRF_LOG_DEBUG("start cur_block processing.");
////    NRF_LOG_DEBUG("cur_block address : = 0x%08x",p_buffer_released);
////    nrf_delay_ms(20);
//
//    for (int j=0; j < block_size; j++) { 
//      int16_t sample = *(p_buffer_released  + j);
//      if (sample == -32768) { sample ++;} //overflow at -32768 for abs
//      int16_t abs_sample = abs(sample); 
//      cur_block_mean += sample;
//      cur_block_abs_mean += abs_sample;
//
//      if (abs_sample > cur_block_peak) { 
//          cur_block_peak = abs_sample;
//          } 
//      }
//    cur_block_mean = cur_block_mean / block_size ; 
//    cur_block_abs_mean = cur_block_abs_mean / block_size ; 
//
//    if (abs(cur_block_mean) > pos_block_mean_threshold ) {
//        cur_block[0] = 1;
//        }
//
//    cur_block[1] = cur_block_abs_mean;
//    cur_block[2] = cur_block_peak;
// 
////     NRF_LOG_DEBUG("cur_block: %d, %d, %d",cur_block[0],cur_block[1],cur_block[2]);
////      nrf_delay_ms(10);
//
//      for (int i=0; i<AUDIO_MEMORY_BUFFER_SIZE/(sizeof(audio_memory_t)*block_size); i++) { 
//        
//// process post blocks
//           pos_block_mean = 0;
//           pos_block_abs_mean = 0;
//           pos_block_peak = 0;
//          
//            for (int j=0; j < block_size; j++) { 
//                int16_t sample = *(p_buffer_released+(i+1)*block_size + j);
//                if (sample == -32768) { sample ++;} //overflow at -32768 for abs
//                int16_t abs_sample = abs(sample); 
//                pos_block_mean += sample;
//                pos_block_abs_mean += abs_sample;
//            
//                if (abs_sample > pos_block_peak) { 
//                    pos_block_peak = abs_sample;
//                    } 
//                }
//            pos_block_mean = pos_block_mean / block_size ; 
//            pos_block_abs_mean = pos_block_abs_mean / block_size ; 
//                pos_block[0] = 0;
//            if (abs(pos_block_mean) > pos_block_mean_threshold ) {
//                pos_block[0] = 1;
//                }
//            
//              pos_block[1] = pos_block_abs_mean;
//              pos_block[2] = pos_block_peak;          
////           NRF_LOG_DEBUG("pos_block: %d, %d, %d at %d ",pos_block[0],pos_block[1],pos_block[2],i);
////            nrf_delay_ms(10);
//
//// check if current block data can be used or not?
//        if ((pre_block[0] + cur_block[0] + pos_block[0]) == 0) {
//            mean += cur_block[1];
//            if (cur_block[2] > peak) { 
//                peak = cur_block[2];
//                }
//            block_count++;
//            }
// // reset pre_block and cur_block for next iteration     
//          memcpy(pre_block, cur_block, 6);
//          memcpy(cur_block, pos_block, 6);
//
//        }
//        NRF_LOG_DEBUG("total block_count used: %d",block_count);
//        mean = mean / block_count;
//    
////end of replacement code

    audio_data.data[0] = mean;
    audio_data.data[1] = peak;

#if NBL_LOG_PDM == 1
   NRF_LOG_DEBUG("PDM Event Handler audio mean:peak = %d:%d", mean,peak);
   NRF_LOG_DEBUG("audio buffer processing finished at %d ms from 0x%08x.\n",
       (uint64_t) (NRF_RTC2->COUNTER)*1000/32768,p_buffer_released); 
   nrf_delay_ms(1);
#endif

#if NBL_AUDIO_TEST_MODE == 1
 if(peak > 30000 && packet_count_1 >1) {
//  if(packet_count == 5 ) {

        NRF_LOG_DEBUG("pdm stop ret_code: %d", ret_code = nrfx_pdm_stop()); 
        NRF_LOG_DEBUG("packet_count: %d",packet_count);
        nrf_delay_ms(1);
       __BKPT(); // Stop here to download an audio sample.
                 // Sample is at p_evt->buffer_released.
                 // Length is AUDIO_MEMORY_BUFFER_SIZE which is 
                 // UDIO_MEMORY_SIZE / AUDIO_MEMORY_NUM_BUFFERS
       nrfx_pdm_start();
 }
#endif

#if PATCHKEEPER_BUFFER_SENSOR_DATA==1  //save audio mean and peak
  //if (m_ble_connected || (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES)) {
  if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
      Swap_and_Save_Buffer();
    }

    //ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES);
    memcpy((m_p_datalog_mem+m_write_offset), &audio_data,  size);
    m_write_offset += size;
#endif  //PATCHKEEPER_BUFFER_SENSOR_DATA

#if BLE_STREAM_IMMEDIATE==1
  if (m_ble_connected == 1) {
    uint8_t* p_start = (uint8_t*) &audio_data;
    uint8_t* p_end   = p_start + sizeof(audio_data_t);
    m_main_context.ble_send_data(p_start, p_end);
    NRF_LOG_DEBUG("ble audio data sent at ms = %d for buffer: 0x%08x\n",
       (uint64_t) (NRF_RTC2->COUNTER)*1000/32768,p_buffer_released); 
  }
#endif  //BLE_STREAM_IMMEDIATE

    // Clear the Audio Memory Buffer. Is this necessary?
    memset(audiodata_memory, 0, AUDIO_MEMORY_BUFFER_SIZE);
  
    m_send_audio_data = 0;

     } // end of m_send_audio_data == 1
#endif  //end of CALC_AUDIO_MEAN
   }   // end of packet_count_1 >5 

#if BL_LOG_PDM == 1
  if (p_evt->error != NRF_SUCCESS) {
    NRF_LOG_DEBUG("PDM Event Handler\n");
    NRF_LOG_DEBUG("p_evt->buffer_requested = %d\n", p_evt->buffer_requested);
    NRF_LOG_DEBUG("p_evt->buffer_released = %d\n", p_evt->buffer_released);
    NRF_LOG_DEBUG("p_evt->error = %d\n", p_evt->error);
  }
#endif //BL_LOG_PDM == 1
}   // end of p_evt->buffer_released != NULL
}     // end of audio_datalogger_handler
 

/**
 * @brief Initialise the PDM for Audio Sampling.
 */
void pdm_init() {
  ret_code_t ret_code;
  nrfx_pdm_config_t pdm_config = NRFX_PDM_DEFAULT_CONFIG(MIC_CLK, MIC_DOUT);
// Up the gain
  pdm_config.gain_l = NRF_PDM_GAIN_MAXIMUM; // 0x50
  pdm_config.gain_r = NRF_PDM_GAIN_MAXIMUM; // 0x50
  
  //pdm_config.gain_l = NRF_PDM_GAIN_DEFAULT; // 0x28
  //pdm_config.gain_r = NRF_PDM_GAIN_DEFAULT; // 0x28
 
  // Clear the Audio Memory Buffer
  memset(audiodata_memory, 0, AUDIO_MEMORY_SIZE_BYTES);

  // Set the MIC Power Control pin to output 
//  nrf_gpio_cfg_output(MIC_PWR_CTRL); 
    nrf_gpio_cfg(
      MIC_PWR_CTRL,
      NRF_GPIO_PIN_DIR_OUTPUT,
      NRF_GPIO_PIN_INPUT_DISCONNECT,
      NRF_GPIO_PIN_NOPULL,
      NRF_GPIO_PIN_D0H1,
      NRF_GPIO_PIN_NOSENSE);

  // Turn the mic on
  nrf_gpio_pin_set(MIC_PWR_CTRL);
  nrf_delay_ms(5);
  // Initialise the PDM
  ret_code = nrfx_pdm_init(&pdm_config, patchkeeper_audio_handler);
  if (ret_code != NRF_SUCCESS) {
    __BKPT();
  }
//  nrf_pdm_int_enable(NRF_PDM_INT_STARTED | NRF_PDM_INT_STOPPED | NRF_PDM_INT_END);
  APP_ERROR_CHECK(ret_code);

  //NRF_LOG_DEBUG("nrf_pdm_int_enable_check(start) returns: %d ",(int) nrf_pdm_int_enable_check(NRF_PDM_INT_STARTED ));
  //NRF_LOG_DEBUG("nrf_pdm_int_enable_check(stop) returns: %d ",(int) nrf_pdm_int_enable_check(NRF_PDM_INT_STOPPED ));
  //NRF_LOG_DEBUG("nrf_pdm_int_enable_check(end) returns: %d ",(int) nrf_pdm_int_enable_check(NRF_PDM_INT_END ));

}


// Start PDM Audio Sampling. 
void pdm_start() {
    ret_code_t ret_code;
#if NBL_LOG_PDM == 1
    NRF_LOG_DEBUG("Starting PDM at RTC2 time ms = %d\n", (uint32_t) (NRF_RTC2->COUNTER)*1000/32768);  
    NRF_LOG_FLUSH();
#endif

    ret_code = nrfx_pdm_start(); 
    APP_ERROR_CHECK(ret_code);
}

#endif // BOARD_HAS_PDM_MIC



/*************************************
 ** @Common Routines                **
 *************************************/
/*
 ** @Disable all amplifiers **
 */
void  patchkeeper_disable_INA338_all(void) {
    nrf_gpio_pin_clear(GSR_AMP_EN);
    nrf_gpio_pin_clear(PULSE_AMP_EN);
    nrf_gpio_pin_clear(STRAIN_AMP_EN);
    nrf_gpio_pin_clear(TEMP_AMP_EN);
}


/*
 ** @Enable all amplifiers         **
 */
void  patchkeeper_enable_INA338_all(void) {

#if BOARD_HAS_GSR_SENSOR == 1
  nrf_gpio_pin_set(GSR_AMP_EN);
 #else
    nrf_gpio_pin_clear(GSR_AMP_EN);
#endif

#if BOARD_HAS_PULSE_SENSOR == 1
   nrf_gpio_pin_set(PULSE_AMP_EN);
#else
   nrf_gpio_pin_clear(PULSE_AMP_EN);
#endif

#if BOARD_HAS_STRAIN_SENSOR == 1
    nrf_gpio_pin_set(STRAIN_AMP_EN);
#else
    nrf_gpio_pin_clear(STRAIN_AMP_EN);
#endif

#if BOARD_HAS_TEMP_SENSOR == 1
    nrf_gpio_pin_set(TEMP_AMP_EN);
#else
    nrf_gpio_pin_clear(TEMP_AMP_EN);
#endif
}


/*
 ** @Enable only one amplifier        **
 */
 /*
void enable_INA338_channel(ina338_channel_t channel) {
  disable_ina338_all();
  switch(channel) {
    case(TERMPERATURE)    : nrf_gpio_pin_set(TEMP_AMP_EN); break;
    case(GSR) : nrf_gpio_pin_set(GSR_AMP_EN); break;
    case(STRAIN)   : nrf_gpio_pin_set(STRAIN_AMP_EN); break;
    case(PULSE)   : nrf_gpio_pin_set(PULSE_AMP_EN); break;
    default         : NRF_LOG_DEBUG("Unexpected Channnel for amplifier selection\n");
                      NRF_LOG_FLUSH();
  }
}


uint32_t read_temp_vds(void) {

  patchkeeper_read_adc(m_vout);

  return m_vout[TEMP];

}
*/


/**
 * @ SD card related.
 *
 * */

#define REFORMAT_ON_POR false
//#define REFORMAT_ON_POR true

// Variable to determine name of files written to SDCard
uint16_t m_fileindex = 0;


#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
//typedef uint8_t datalog_memory_t; 
extern datalog_memory_t datalog_memory_0 [DATALOG_MEMORY_SIZE_BYTES/sizeof(datalog_memory_t)];
extern datalog_memory_t datalog_memory_1 [DATALOG_MEMORY_SIZE_BYTES/sizeof(datalog_memory_t)];
extern datalog_memory_t *m_p_datalog_mem;
extern uint16_t m_write_offset;
#endif 

static FATFS m_fs; 


static void datalogger_wait_func(void) 
{ 
//    nrf_delay_ms(1); 
    __WFE(); 
} 


                                                         
void Swap_and_Save_Buffer(void) {

#if PATCHKEEPER_BUFFER_SENSOR_DATA == 1

  static datalog_memory_t *read_ptr;
  static datalog_memory_t *read_ptr_done;
  static uint8_t mem_in_use = 0;

  // Critical Region - prevent all non-SD interrupts.
  sd_nvic_critical_region_enter(0);


  // Move pointers
  if (mem_in_use) {
    m_p_datalog_mem     = (datalog_memory_t*) datalog_memory_0;
    read_ptr            = (datalog_memory_t*) datalog_memory_1;
//    mem_in_use        = 0;
  } else {
    m_p_datalog_mem     = (datalog_memory_t*) datalog_memory_1;
    read_ptr            = (datalog_memory_t*) datalog_memory_0;
//    mem_in_use        = 1;
  }

  // Save the end of the buffer prior to resetting the write offset pointer
  read_ptr_done     = (datalog_memory_t*) (read_ptr + m_write_offset);

  // Reset the Write Offset
  m_write_offset      = 0;

  mem_in_use ^= 1;

  // End of Critical Region
  sd_nvic_critical_region_exit(0);


#if USE_SDCARD == 1
  // Now write the buffer
  sdcard_write(read_ptr, read_ptr_done);
#endif

#if BLE_STREAM_IMMEDIATE == 0
  if (m_ble_connected == 1) {
#if USE_BLE_COMMS == 1
    m_main_context.ble_send_data(read_ptr, read_ptr_done);
#else
    ASSERT(0);
#endif
  } else {
#if USE_SDCARD == 0
    NRF_LOG_INFO("Discarding Buffer as BLE not connected");
    NRF_LOG_FLUSH();
#endif
  }
#endif
#else
  return;
#endif   //PATCHKEEPER_BUFFER_SENSOR_DATA

}

#if USE_SDCARD == 1 
static int Open_SDCard() 
{ 
//    static FIL file 
static uint8_t reformat = 1;
     
    // Only reformat if it was a Power On Reset and REFORMAT_ON_POR defined.
   if (REFORMAT_ON_POR && (m_last_reset_reason == PowerOnReset)) {
      reformat = 1;
#if NBL_LOG_OPEN_SDCARD == 1
      NRF_LOG_DEBUG("PowerOnReset -> SDCard will be reformatted");
#endif
    } else {
      reformat = 0;
#if NBL_LOG_OPEN_SDCARD == 1
      NRF_LOG_DEBUG("NOT PowerOnReset OR NOT REFORMAT_ON_POR -> SDCard will not be reformatted");
#endif
    }
 
    uint32_t bytes_written; 
    FRESULT ff_result; 
    DSTATUS disk_state = STA_NOINIT; 
 
    // Initialize FATFS disk I/O interface by providing the block device. 
    static diskio_blkdev_t drives[] = 
    { 
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), &datalogger_wait_func) 
    }; 
 
    diskio_blockdev_register(drives, ARRAY_SIZE(drives)); 
 

#if NBL_LOG_OPEN_SDCARD == 1
    NRF_LOG_INFO("Initializing disk 0 (SDC)..."); 
    NRF_LOG_FLUSH();
#endif
 
    for (uint32_t retries = 3; retries && disk_state; --retries) 
    { 
        disk_state = disk_initialize(0); 
        NRF_LOG_DEBUG("disk_state: %d",disk_state);
    } 
    if (disk_state) 
    { 
#if NBL_LOG_OPEN_SDCARD == 1
        NRF_LOG_INFO("Disk initialization failed."); 
        NRF_LOG_FLUSH(); 
#endif
        return NRF_ERROR_INTERNAL; 
    } 
 
    // Only reformat if it was a Power On Reset.
    if (reformat) { 
      // Re-use datalog_memory for the working buffer for formatting. 
      FRESULT retval; 
#ifdef REFORMAT_USES_DATALOG_MEM 
      uint8_t *p_work = datalog_memory_0; 
      uint16_t work_size = DATALOG_MEMORY_SIZE_BYTES;    
#else 
#if NBL_LOG_OPEN_SDCARD == 1
      NRF_LOG_INFO("=============================================="); 
      NRF_LOG_INFO("!!! Reformat uses separate memory.   !!!");       
      NRF_LOG_INFO("    Disable when not debugging"); 
      NRF_LOG_INFO("    (%s:%d)", __FILE__, __LINE__); 
#endif // NBL_LOG_OPEN_SDCARD
      uint8_t reformat_work [2048]; 
      uint8_t *p_work = reformat_work; 
      uint16_t work_size = sizeof(reformat_work); 
#endif // REFORMAT_USES_DATALOG_MEM

#if NBL_LOG_OPEN_SDCARD == 1
      NRF_LOG_INFO("Re-formatting volume..."); 
      NRF_LOG_INFO("   Using work memory size of %d bytes...", work_size); 
#endif // NBL_LOG_OPEN_SDCARD
      retval=f_mkfs("",FM_ANY, 0, p_work, work_size); 
 
#if NBL_LOG_OPEN_SDCARD == 1
      switch (retval) { 
        case FR_MKFS_ABORTED :  
          NRF_LOG_INFO("Re-formatting Aborted!!!"); 
          NRF_LOG_INFO("                      - possibly work buffer too small (%d)...", work_size); 
          NRF_LOG_FLUSH(); 
          break; 
        case (FR_DISK_ERR)        :   NRF_LOG_INFO("Disk Error!!!"); break; 
        case (FR_INT_ERR)         :   NRF_LOG_INFO("Internal Error!!!"); break; 
        case (FR_DENIED)          :   NRF_LOG_INFO("Disk Access Denied!!!"); break; 
        case (FR_INVALID_OBJECT)  :   NRF_LOG_INFO("File Object Invalid!!!"); break; 
        case (FR_TIMEOUT)         :   NRF_LOG_INFO("Thread Control Timed-out!!!"); break; 
        case (FR_OK)              :   NRF_LOG_INFO("Reformat Completed Successfully"); break; 
        default                   :   NRF_LOG_INFO("Unknown Response (%d)", retval); 
     } 
#endif // NBL_LOG_OPEN_SDCARD
 
#if NBL_LOG_OPEN_SDCARD == 1
      NRF_LOG_FLUSH(); 
#endif // NBL_LOG_OPEN_SDCARD
      if (retval != FR_OK) { 
        error_led_flash_loop(); 
        __BKPT(); 
      } 
 
      reformat = false; 
    } 
 

#if NBL_LOG_OPEN_SDCARD == 1
    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size; 
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb; 

    NRF_LOG_INFO("Capacity: %d MB", capacity); 
 
    NRF_LOG_INFO("Mounting volume..."); 
#endif
    ff_result = f_mount(&m_fs, "", 1); 
    if (ff_result) 
    { 
#if NBL_LOG_OPEN_SDCARD == 1
        NRF_LOG_INFO("Mount failed."); 
#endif
        return NRF_ERROR_INTERNAL; 
    } 
 
    return NRF_SUCCESS; 
} 
 
/** 
 * @brief List the SDCard (debugging only?). 
 */ 
static int List_SDCard() { 
    static DIR dir; 
    static FILINFO fno; 
    FRESULT ff_result; 
 
#if NBL_LOG_LIST_SDCARD == 1
    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size; 
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb; 

    NRF_LOG_INFO("Capacity: %d MB", capacity); 
 
    NRF_LOG_INFO("Mounting volume..."); 
#endif

    ff_result = f_mount(&m_fs, "", 1); 
    if (ff_result) 
    { 
#if NBL_LOG_LIST_SDCARD == 1
        NRF_LOG_INFO("Mount failed."); 
#endif
        return NRF_ERROR_INTERNAL; 
    } 
 
#if NBL_LOG_LIST_SDCARD == 1
    NRF_LOG_INFO("\r\n Listing directory: /"); 
#endif
    ff_result = f_opendir(&dir, "/"); 
    if (ff_result) 
    { 
#if NBL_LOG_LIST_SDCARD == 1
        NRF_LOG_INFO("Directory listing failed!"); 
#endif
        return NRF_ERROR_INTERNAL; 
    } 
 
    uint16_t file_index = 0;
    do 
    { 
        ff_result = f_readdir(&dir, &fno); 
        if (ff_result != FR_OK) 
        { 
#if NBL_LOG_LIST_SDCARD == 1
            NRF_LOG_INFO("Directory read failed."); 
#endif
            return NRF_ERROR_INTERNAL; 
        } 

        if (fno.fname[0]) 
        { 
            if (fno.fattrib & AM_DIR) 
            { 
#if NBL_LOG_LIST_SDCARD == 1
                NRF_LOG_RAW_INFO("   <DIR>   %s\n",(uint32_t)fno.fname); 
#endif
            } 
            else 
            { 
#if NBL_LOG_LIST_SDCARD == 1
                NRF_LOG_RAW_INFO("%9lu  %s\n", fno.fsize, (uint32_t)fno.fname); 
#endif
            
            //if (strncmp(fno.fname, FILENAME_BASENAME, strlen(FILENAME_BASENAME))) {
            //  uint8_t fileindex_position = strlen(FILENAME_BASENAME);
            //  char this_fileindex_char = fno.fname[fileindex_position];
            //  char this_fileindex_char2 = fno.fname[fileindex_position+1];
            //  if ((this_fileindex_char2 != '.') && (this_fileindex_char2 != '_') ){
            //      uint16_t this_fileindex2 = this_fileindex_char2 - '0';
      
            //  uint16_t this_fileindex = (this_fileindex_char - '0')*10 + this_fileindex2;
            //  if (this_fileindex >= file_index) {
            //    file_index = this_fileindex+1;
            //            }  
            //         }
            //   uint16_t this_fileindex = this_fileindex_char - '0';
            //  if (this_fileindex >= file_index) {
            //    file_index = this_fileindex+1;
            //    }
            //  }


    if(strncmp(fno.fname, FILENAME_BASENAME, strlen(FILENAME_BASENAME))==0)
    {
    
        uint8_t fileindex_position = strlen(FILENAME_BASENAME);
        char this_fileindex_char =  fno.fname[fileindex_position];
        char this_fileindex_char2 = fno.fname[fileindex_position+1];
        char this_fileindex_char3 = fno.fname[fileindex_position+2];

     if ('9' < this_fileindex_char3 || this_fileindex_char3 < '0'){

        if ((this_fileindex_char2 != '.') && (this_fileindex_char2 != '_') )
        {
            uint16_t this_fileindex2 = this_fileindex_char2 - '0';
            uint16_t this_fileindex = (this_fileindex_char - '0')*10 + this_fileindex2;
              if (this_fileindex >= file_index) 
              {
                file_index = this_fileindex+1;
              }  
        }
               uint16_t this_fileindex = this_fileindex_char - '0';
              if (this_fileindex >= file_index) 
              {
                file_index = this_fileindex+1;  
              }
    
     } else{
        uint16_t this_fileindex3 = this_fileindex_char3 - '0';
        uint16_t this_fileindex = (this_fileindex_char - '0')*100 + 
            (this_fileindex_char2 - '0')*10 + this_fileindex3;
              if (this_fileindex >= file_index) 
              {
                file_index = this_fileindex+1;
              }  
     }
    
    }
            }
        } 
    } 

    while (fno.fname[0]); 
#if NBL_LOG_LIST_SDCARD == 1
    NRF_LOG_RAW_INFO(""); 
#endif
    NRF_LOG_INFO("m_fileindex: %d",m_fileindex);
    if (file_index > m_fileindex) {
      m_fileindex = file_index;
    }
       NRF_LOG_INFO("last file_index: %d",m_fileindex);

#if NBL_LOG_LIST_SDCARD == 1 
    static char     filename [20];
    sprintf(filename, "%s%d.%s", FILENAME_BASENAME,m_fileindex,FILENAME_SUFFIX);   
    NRF_LOG_DEBUG("Setting initial file name to %s", filename); 
    NRF_LOG_FLUSH(); 
#endif
    return 0; 
} 


static FRESULT sdcard_write(datalog_memory_t *start_ptr, datalog_memory_t *end_ptr) { 
    static FIL      file;
    FRESULT         ff_result; 
    uint32_t        bytes_written = 0;  
    static char     filename [32]; 
    static uint16_t    file_seq = 0;
    uint32_t         filesize =0;

if (file_seq == 0) {
    sprintf(filename, "%s%d.%s", FILENAME_BASENAME,m_fileindex,FILENAME_SUFFIX);
    ff_result = f_open(&file, filename, FA_READ | FA_WRITE | FA_OPEN_APPEND); 
      if (ff_result != FR_OK) 
        { 
          NRF_LOG_DEBUG("ff_result: %d", ff_result);
          NRF_LOG_DEBUG("Unable to open or create file: %s", filename); 
          NRF_LOG_FLUSH(); 
          return NRF_ERROR_INTERNAL; 
        } 

    filesize = f_size(&file); 

    if (filesize > MAX_FILESIZE) { 
      f_close(&file); 
      file_seq++; 
      sprintf(filename, "%s%d_%d.%s", FILENAME_BASENAME,m_fileindex,file_seq,FILENAME_SUFFIX); 
      ff_result = f_open(&file, filename, FA_READ | FA_WRITE | FA_OPEN_APPEND);
        if (ff_result != FR_OK) 
        { 
          NRF_LOG_DEBUG("ff_result: %d", ff_result);
          NRF_LOG_DEBUG("Unable to open or create file: %s", filename); 
          NRF_LOG_FLUSH(); 
          return NRF_ERROR_INTERNAL; 
        }        
      } 
 
  } else {
     sprintf(filename, "%s%d_%d.%s", FILENAME_BASENAME,m_fileindex,file_seq,FILENAME_SUFFIX); 
     ff_result = f_open(&file, filename, FA_READ | FA_WRITE | FA_OPEN_APPEND);
      if (ff_result != FR_OK) 
        { 
          NRF_LOG_DEBUG("ff_result: %d", ff_result);
          NRF_LOG_DEBUG("Unable to open or create file: %s", filename); 
          NRF_LOG_FLUSH(); 
          return NRF_ERROR_INTERNAL; 
        }      
      
      filesize = f_size(&file);
      if (filesize > MAX_FILESIZE) 
        { 
        f_close(&file); 
        file_seq++; 
        sprintf(filename, "%s%d_%d.%s", FILENAME_BASENAME,m_fileindex,file_seq,FILENAME_SUFFIX); 
        ff_result = f_open(&file, filename, FA_READ | FA_WRITE | FA_OPEN_APPEND);
          if (ff_result != FR_OK) 
            { 
              NRF_LOG_DEBUG("ff_result: %d", ff_result);
              NRF_LOG_DEBUG("Unable to open or create file: %s", filename); 
              NRF_LOG_FLUSH(); 
              return NRF_ERROR_INTERNAL; 
            }  
      filesize = f_size(&file);
      }
    }
    

 #if NBL_LOG_SDCARD_WRITE == 1 
    uint64_t timestamp_us = get_timestamp_us();
    char buffer[32];
    sprintf(buffer, "%llu", timestamp_us);
    NRF_LOG_DEBUG("currentime is: %s\n", buffer);
    NRF_LOG_DEBUG("Writing to file %s...", filename); 
    NRF_LOG_DEBUG("File size is %d bytes...", filesize); 
    //NRF_LOG_DEBUG("start_ptr = 0x%08x", (int) start_ptr); 
    //NRF_LOG_DEBUG("end_ptr = 0x%08x", (int) end_ptr); 
    //NRF_LOG_DEBUG("Total bytes to write = %d", (int) (end_ptr - start_ptr)); 
    NRF_LOG_FLUSH(); 
#endif
 
    uint8_t done = false; 
    uint16_t total_bytes_written = 0; 
 

// Write as chunks of SD_WRITE_BLOCK_SIZE 
#define SD_WRITE_BLOCK_SIZE 0 // 0: write everything in a single block
  while (!done) { 
 
    uint16_t bytes_remaining = (int) (end_ptr - start_ptr); 
 
    uint16_t bytes_per_write_cycle = SD_WRITE_BLOCK_SIZE; 
 
    if (0 == bytes_per_write_cycle) { 
      bytes_per_write_cycle = bytes_remaining; 
    }   
 
    if (bytes_remaining < bytes_per_write_cycle) { 
      bytes_per_write_cycle = bytes_remaining; 
    } 
  //   NRF_LOG_DEBUG("bytes_per_write_cycle used: %d",bytes_per_write_cycle);

    ff_result = f_write(&file, (const void*) start_ptr, bytes_per_write_cycle, (UINT *) &bytes_written); 
    start_ptr += bytes_per_write_cycle; 
    total_bytes_written += bytes_written; 
 
    if (start_ptr >= end_ptr) { 
      done = true; 
    } 
  } 

#if NBL_LOG_SDCARD_WRITE == 1   
    if (ff_result != FR_OK) { 
        NRF_LOG_INFO("Write failed\r\n."); 
        NRF_LOG_FLUSH(); 
    } else { 
        NRF_LOG_INFO("%d bytes written.", total_bytes_written); 
        NRF_LOG_FLUSH(); 
    } 
#endif
 
    (void) f_close(&file); 
 
//    board_led_off(LED_GREEN); 
 
    return ff_result; 
   
} 


/****************************************************************
Save buffered audio raw data in memory into files on SD card 
*****************************************************************/
void Save_Audio_Buffer(audio_memory_t *start_ptr) {

  datalog_memory_t *save_start_ptr = (datalog_memory_t *)start_ptr;
  datalog_memory_t *save_end_ptr = save_start_ptr+AUDIO_MEMORY_BUFFER_SIZE;

#if NBL_LOG_SDCARD_WRITE == 1
   NRF_LOG_INFO("save_start_ptr: 0x%08x", save_start_ptr );
   NRF_LOG_INFO("AUDIO_MEMORY_BUFFER_SIZE: 0x%08x", AUDIO_MEMORY_BUFFER_SIZE);
   NRF_LOG_INFO("save_end_ptr: 0x%08x", save_end_ptr );
#endif

  // Now write the buffer to file on SD card
    sdcard_write_audio_trace(save_start_ptr, save_end_ptr);
    //sdcard_write_audio_trace_small_files(save_start_ptr, save_end_ptr);

}



/****************************************************************
Write blcok data in memory into files on SD card 
*****************************************************************/
static FRESULT sdcard_write_audio_trace(datalog_memory_t *start_ptr, datalog_memory_t *end_ptr) { 
    static FIL      file_audio; 
    FRESULT         ff_result_audio; 
    uint32_t        bytes_written = 0;  
    static char     filename_audio[32]; 
    static uint16_t   file_seq_audio = 0;
    uint32_t       filesize = 0;

if (file_seq_audio == 0) {
    sprintf(filename_audio, "%s%d.%s", FILENAME_BASENAME_AUDIO,m_fileindex,FILENAME_SUFFIX);
    ff_result_audio = f_open(&file_audio, filename_audio, FA_READ | FA_WRITE | FA_OPEN_APPEND); 
      if (ff_result_audio != FR_OK) 
      { 
          NRF_LOG_DEBUG("ff_result_audio: %d", ff_result_audio);
          NRF_LOG_DEBUG("Unable to open or create file: %s", filename_audio); 
          NRF_LOG_FLUSH(); 
          return NRF_ERROR_INTERNAL; 
      } 
    
    filesize = f_size(&file_audio); 


    if (filesize > MAX_FILESIZE) { 
      f_close(&file_audio); 
      file_seq_audio++; 
      sprintf(filename_audio, "%s%d_%d.%s", FILENAME_BASENAME_AUDIO,m_fileindex,file_seq_audio,FILENAME_SUFFIX); 
      ff_result_audio = f_open(&file_audio, filename_audio, FA_READ | FA_WRITE | FA_OPEN_APPEND); 
      } 
 
  } else {
     sprintf(filename_audio, "%s%d_%d.%s", FILENAME_BASENAME_AUDIO,m_fileindex,file_seq_audio,FILENAME_SUFFIX); 
     ff_result_audio = f_open(&file_audio, filename_audio, FA_READ | FA_WRITE | FA_OPEN_APPEND); 
     if (ff_result_audio != FR_OK) 
      { 
          NRF_LOG_DEBUG("ff_result_audio: %d", ff_result_audio);
          NRF_LOG_DEBUG("Unable to open or create file: %s", filename_audio); 
          NRF_LOG_FLUSH(); 
          return NRF_ERROR_INTERNAL; 
      } 
     filesize = f_size(&file_audio);
      NRF_LOG_DEBUG("current file size: %d", filesize);
        
      if (filesize > MAX_FILESIZE) { 
      f_close(&file_audio); 
      file_seq_audio++; 
      sprintf(filename_audio, "%s%d_%d.%s", FILENAME_BASENAME_AUDIO,m_fileindex,file_seq_audio,FILENAME_SUFFIX); 
      ff_result_audio = f_open(&file_audio, filename_audio, FA_READ | FA_WRITE | FA_OPEN_APPEND); 
        if (ff_result_audio != FR_OK) { 
          NRF_LOG_DEBUG("ff_result_audio: %d", ff_result_audio);           
          NRF_LOG_DEBUG("Unable to open or create file: %s", filename_audio); 
          NRF_LOG_FLUSH(); 
          return NRF_ERROR_INTERNAL; 
          }
        filesize = f_size(&file_audio);

      }
    }
    

#if NBL_LOG_SDCARD_WRITE == 1 
    NRF_LOG_DEBUG("current audio filename is: %s", filename_audio);
    NRF_LOG_DEBUG("File size is %d bytes...", filesize);
    uint64_t timestamp_us = get_timestamp_us();
    char buffer[32];
    sprintf(buffer, "%llu", timestamp_us);
    NRF_LOG_DEBUG("currentime is: %s", buffer);
    NRF_LOG_DEBUG("start_ptr = 0x%08x", (int) start_ptr); 
    NRF_LOG_DEBUG("end_ptr = 0x%08x", (int) end_ptr); 
    NRF_LOG_DEBUG("Total bytes to write = %d", (int) (end_ptr - start_ptr)); 
    NRF_LOG_FLUSH(); 
#endif
 
    uint8_t done = false; 
    uint16_t total_bytes_written = 0; 
 

// Write as chunks of SD_WRITE_BLOCK_SIZE 
#define SD_WRITE_BLOCK_SIZE 0 
  while (!done) { 
 
    uint16_t bytes_remaining = (int) (end_ptr - start_ptr); 
 
    uint16_t bytes_per_write_cycle = SD_WRITE_BLOCK_SIZE; 
 
    if (0 == bytes_per_write_cycle) { 
      bytes_per_write_cycle = bytes_remaining; 
    }   
 
    if (bytes_remaining < bytes_per_write_cycle) { 
      bytes_per_write_cycle = bytes_remaining; 
    } 
 
    ff_result_audio = f_write(&file_audio, (const void*) start_ptr, bytes_per_write_cycle, (UINT *) &bytes_written); 
    start_ptr += bytes_per_write_cycle; 
    total_bytes_written += bytes_written; 
 
    if (start_ptr >= end_ptr) { 
      done = true; 
    } 
  } 

#if NBL_LOG_SDCARD_WRITE == 1   
    if (ff_result_audio != FR_OK) { 
        NRF_LOG_INFO("Write failed."); 
        NRF_LOG_FLUSH(); 
    } else { 
        NRF_LOG_INFO("%d bytes written.", total_bytes_written); 
        NRF_LOG_FLUSH(); 
    } 
#endif
 
    (void) f_close(&file_audio); 
 
//    board_led_off(LED_GREEN); 
 
    return ff_result_audio; 

    }

#endif
