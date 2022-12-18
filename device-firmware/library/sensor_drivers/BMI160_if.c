/*
 * © 2022 Nokia
 * Licensed under the BSD 3-Clause License
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "BMI160_if.h"
#include "nrf_drv_gpiote.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "sdk_config.h"

static struct bmi160_dev m_bmi160_sensor;
static struct bmi160_sensor_data m_accel;
static struct bmi160_sensor_data m_gyro;


#define BMI160_SENSOR_INT1_PIN 11
#define BMI160_SENSOR_INT2_PIN 12

// Define the IMU Callback Function Pointer Type
typedef void (*imu_int_callback_fptr_t)();

static imu_int_callback_fptr_t imu_int_callback_fn = NULL;

void BMI160_Check() {
    
    int8_t rslt = BMI160_OK;

    struct regview {
        uint8_t chip_id;
        uint8_t reserved_0;
        uint8_t errors;
        uint8_t pmu_status;
        uint8_t data [20];
        uint8_t sensortime [3];
        uint8_t status;
        uint8_t int_status [4];
        uint8_t temperature [2];
        uint8_t fifo_length [2];
        uint8_t fifo_data ;
        uint8_t reserved_1;
        uint8_t reserved_2;
        uint8_t acc_conf;
        uint8_t acc_range;
    } myregview;
    
    for (uint8_t i=0; i<sizeof(myregview); i++) {
        *((uint8_t*) &myregview + i) = 0xff;
    }
    
    rslt =  bmi160_get_regs(BMI160_CHIP_ID_ADDR,  (uint8_t*) &myregview.chip_id, 4, &m_bmi160_sensor);
    if (rslt) {
      __BKPT();
    }
    APP_ERROR_CHECK(rslt);

    rslt =  bmi160_get_regs(BMI160_CHIP_ID_ADDR,  (uint8_t*) &myregview.sensortime, 8, &m_bmi160_sensor);
    APP_ERROR_CHECK(rslt);
#if NBL_LOG_BOARD == 1    
    SEGGER_RTT_printf(0, "CHIP_ID :     %x\n", myregview.chip_id);   
    SEGGER_RTT_printf(0, "ERROR_REG :   %x\n", myregview.errors);   
    SEGGER_RTT_printf(0, "PMU_STATUS :  %x\n", myregview.pmu_status);   
    
    if (myregview.chip_id != BMI160_CHIP_ID) {
        SEGGER_RTT_printf(0, "BMI160 Connection Lost (Chip ID returned %x)\n", myregview.chip_id);
    }
    if (myregview.errors) {
        SEGGER_RTT_printf(0, "BMI160 Errors reported (%x)\n", myregview.errors);
    }
#endif
    
}

int8_t BMI160_FastOffsetCompensation(struct bmi160_dev *dev);

void bmi160_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
void bmi160_int1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  //NRF_LOG_INFO("In INT1 Handler")
  bmi160_int_handler(pin, action);
}

void bmi160_int2_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  //NRF_LOG_INFO("In INT2 Handler")
  bmi160_int_handler(pin, action);
}


/**@brief Interrupt Handler for BMI160.
 */
void bmi160_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    //static int count = 0;

    //NRF_LOG_DEBUG("Received BMI160 Interrupt - in bmi160_int_handler (%d)", count++);

    int8_t rslt = BMI160_OK;
    uint8_t data[4];

    // Read the Interrupt Status Register before anything else, otherwise the status register is cleared!!!

//    rslt = bmi160_get_regs(BMI160_CHIP_ID_ADDR, data, 1, &sensor);
//
//    if (BMI160_OK != rslt) {
//      NRF_LOG_DEBUG("Error from bmi160_get_regs for BMI160_CHIP_ID_ADDR (return value is 0x%x)");
//    } else if (data[0] == 0xd1) {
//      NRF_LOG_DEBUG("BMI Device found after interrupt on GPIO");
//    } else {
//      NRF_LOG_ERROR("BMI Device not found");
//    }

    rslt = bmi160_get_regs(BMI160_INT_STATUS_ADDR, data, 4, &m_bmi160_sensor);
    if (BMI160_OK != rslt) {
      NRF_LOG_DEBUG("Error from bmi160_get_regs for BMI160_INT_STATUS_ADDR (return value is 0x%x)");
    } else  {

      NRF_LOG_INFO("Interrupt Status Register reads 0x%02x", data[0]);
      if (data[0] & (1 << 7)) {NRF_LOG_INFO("Flat Interrupt Fired");}
      if (data[0] & (1 << 6)) {NRF_LOG_INFO("Orientation Interrupt Fired");}
      if (data[0] & (1 << 5)) {NRF_LOG_INFO("Single Tap Interrupt Fired");}
      if (data[0] & (1 << 4)) {NRF_LOG_INFO("Double Tap Interrupt Fired");}
      if (data[0] & (1 << 3)) {NRF_LOG_INFO("PMU Trigger Interrupt Fired");}
      if (data[0] & (1 << 2)) {NRF_LOG_INFO("AnyMotion Interrupt Fired");}
      if (data[0] & (1 << 1)) {NRF_LOG_INFO("Significant Motion Interrupt Fired");}
      if (data[0] & (1 << 0)) {NRF_LOG_INFO("Step Detector Interrupt Fired");}
      if (data[0] == 0)       {NRF_LOG_INFO("No Interrupt present in status register");}
    
    }
    NRF_LOG_FLUSH();

    if (imu_int_callback_fn != NULL) {
      imu_int_callback_fn();
    }

}

/**@brief Callback from IMU Interrupt. 
  This is NOT the ISR but, if defined, will be called from the ISR
 */

//void imu_int_callback () {
//
//  // TODO - Do something useful here e.g. send a notification over BLE?
//#if NBL_LOG_BOARD == 1 
//  NRF_LOG_INFO("In IMU Interrupt Callback (imu_int_callback) within main.c");
//#endif
//} 


int8_t BMI160_Configure(){

#if NBL_LOG_BOARD == 1 
    NRF_LOG_INFO("Configuring BME160 Accel and Gyroscope");
#endif
    
    int8_t rslt = BMI160_OK;

    /* Select the Output data rate, range of accelerometer sensor */
    m_bmi160_sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    m_bmi160_sensor.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
    m_bmi160_sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    m_bmi160_sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    m_bmi160_sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    m_bmi160_sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    m_bmi160_sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    m_bmi160_sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&m_bmi160_sensor);
    APP_ERROR_CHECK(rslt);

//    // Self Test - it is not clear how to get the device back from a self test. 
//    if (BMI160_OK == BMI160_SelfTest()) {
//        // Reconfigure after self test
//        rslt = bmi160_soft_reset(&m_bmi160_sensor);
//        APP_ERROR_CHECK(rslt);
//        rslt = bmi160_set_sens_conf(&m_bmi160_sensor);
//        APP_ERROR_CHECK(rslt);
//     } else {
//        rslt = NRF_ERROR_INTERNAL;
//        NRF_LOG_INFO("ERROR Self Test Failed.");
//        while (1);
//     }
    


    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&m_bmi160_sensor);
    APP_ERROR_CHECK(rslt);

//    rslt = BMI160_FastOffsetCompensation(&m_bmi160_sensor);
//    APP_ERROR_CHECK(rslt);

#if USE_BLE_COMMS == 1 || USE_SDCARD == 1
    /* Set the sensor to send an interrupt (INT1) on double tap. */
    struct bmi160_int_settg int_config;
    int_config.int_channel = BMI160_INT_CHANNEL_1;
    int_config.int_type = BMI160_ACC_DOUBLE_TAP_INT;
    int_config.int_pin_settg.output_en = 1;
    int_config.int_pin_settg.output_mode = 0;
    int_config.int_pin_settg.output_type = 1;
    int_config.int_pin_settg.edge_ctrl = 0;
    int_config.int_pin_settg.input_en = 0;
    int_config.int_pin_settg.latch_dur = 4;

    rslt = bmi160_set_int_config(&int_config, &m_bmi160_sensor);
    APP_ERROR_CHECK(rslt);
#endif

    return rslt;
    
};


void BMI160_Turn_On( void ){
    
    int8_t rslt = BMI160_OK;

#ifdef PCA20020
    // Power up the VDD rail
    NRF_LOG_INFO("Turning on power to VDD for BME160");
    nrf_gpio_cfg_output(VDD_PWR_CTRL);
    nrf_gpio_pin_set(VDD_PWR_CTRL);
    nrf_delay_ms(5);
#endif
    
#if NBL_LOG_BOARD == 1 
    NRF_LOG_INFO("Initialising BMI160");
#endif
        
    m_bmi160_sensor.id = BMI160_I2C_ADDR;
    m_bmi160_sensor.interface = BMI160_I2C_INTF;
    m_bmi160_sensor.read = (bmi160_com_fptr_t) I2C_Read;
    m_bmi160_sensor.write = (bmi160_com_fptr_t) I2C_Write;
    m_bmi160_sensor.delay_ms = (bmi160_delay_fptr_t) nrf_delay_ms;

    // Reset the board in case there is already an error
    // TODO - do we still need this now we know we were not powering the board up?
    // rslt = bmi160_soft_reset(&m_bmi160_sensor);
    
    nrf_delay_ms(100);
    
    APP_ERROR_CHECK(rslt);

    BMI160_Check();

    rslt = bmi160_init(&m_bmi160_sensor);
    /* After the above function call, accel and gyro parameters in the device structure 
    are set with default values, found in the datasheet of the sensor */

    APP_ERROR_CHECK(rslt);

    if (m_bmi160_sensor.chip_id == BMI160_CHIP_ID) {
        BMI160_Configure();
    } else {
#if NBL_LOG_BOARD == 1 
        SEGGER_RTT_printf(0, "BMI160 ID:%x Should be = %x\n", m_bmi160_sensor.chip_id, BMI160_CHIP_ID);   
#endif
    } 

//    // Turn OFF the accelerometer.
//    m_bmi160_sensor.accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;
//    
//    NRF_LOG_INFO("Setting Power Mode (Suspend)");
//    rslt =  bmi160_set_power_mode(&m_bmi160_sensor);
//    APP_ERROR_CHECK(rslt);
//
//    // Turn ON the accelerometer.
//    m_bmi160_sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
//    
//    NRF_LOG_INFO("Setting Power Mode (Powering Up)");
//    rslt =  bmi160_set_power_mode(&m_bmi160_sensor);
//    APP_ERROR_CHECK(rslt);

    struct    bmi160_pmu_status     pmu_status;
    
    rslt =  bmi160_get_power_mode(&pmu_status, &m_bmi160_sensor);
    APP_ERROR_CHECK(rslt);
    
#if NBL_LOG_BOARD == 1 
    SEGGER_RTT_printf(0, "BMI160 Accel power Status : %d\n", pmu_status.accel_pmu_status);   
    SEGGER_RTT_printf(0, "BMI160 Gyro power Status : %d\n", pmu_status.aux_pmu_status);   
    SEGGER_RTT_printf(0, "BMI160 Aux power Status : %d\n", pmu_status.aux_pmu_status);   
#endif
    
};

uint8_t BMI160_SelfTest(void){

  uint8_t rslt = BMI160_OK;
  uint8_t errors = 0;
  /* Call the "bmi160_init" API as a prerequisite before performing self test
   * since invoking self-test will reset the sensor */

   // We assume that this function has been called after calling bmi160_init;

    rslt = bmi160_perform_self_test(BMI160_ACCEL_ONLY, &m_bmi160_sensor);

    if (rslt == BMI160_OK) {
#if NBL_LOG_BOARD == 1 
        SEGGER_RTT_printf(0, "\n ACCEL SELF TEST RESULT SUCCESS\n");
#endif
    } else {
#if NBL_LOG_BOARD == 1 
        SEGGER_RTT_printf(0, "\n ACCEL SELF TEST RESULT FAIL\n");
#endif
        errors++;
    }

        rslt = bmi160_perform_self_test(BMI160_GYRO_ONLY, &m_bmi160_sensor);

    if (rslt == BMI160_OK) {
#if NBL_LOG_BOARD == 1 
        SEGGER_RTT_printf(0, "\n GYRO SELF TEST RESULT SUCCESS\n");
#endif 
    } else {
#if NBL_LOG_BOARD == 1 
        SEGGER_RTT_printf(0, "\n GYRO SELF TEST RESULT FAIL\n");
#endif
        errors++;
    }

    return errors;
}

typedef struct  {
        /*! X-axis sensor data */
        int16_t x;
        /*! Y-axis sensor data */
        int16_t y;
        /*! Z-axis sensor data */
        int16_t z;
} bmi160_sensor_data_simple_t;

typedef struct  {
      bmi160_sensor_data_simple_t gyro_data;
      bmi160_sensor_data_simple_t accel_data;
      uint32_t                    sensortime;

} imu_data_packed_t;

typedef union {
      int8_t  dest8   [(sizeof(imu_data_packed_t)/sizeof(int8_t))];
      int16_t dest16  [(sizeof(imu_data_packed_t)/sizeof(int16_t))];
      imu_data_packed_t dest_struct;
} dest_mem_union_t;

void BMI160_Get_Data(int16_t * dest){
    
    uint8_t rslt = BMI160_OK;
    uint8_t data;
    //imu_data_packed_t *imu_data_packed;

    // This is for debug purposes. It is not used. 
    dest_mem_union_t *dest_mem __attribute__ ((unused)) = (dest_mem_union_t*) dest;

    // Check that the I2C still appears to be working
    rslt = I2C_Read(BMI160_I2C_ADDR, BMI160_CHIP_ID_ADDR, &data, 1);

#if NBL_LOG_BOARD == 1 
    if (BMI160_CHIP_ID != data) {
      SEGGER_RTT_printf(0,"BMI160 no longer appears to be responding to I2C - CHIP_ID returned 0x%02x\n", data);
    }
#endif

    // Bypass the driver as it is not efficient.  Memory usage is wasteful and unnecessary processing 
    // of data into structure which is better done with a union as here. It's probably not intended 
    // for serious use - perhaps the FIFO is better used?
    //rslt = bmi160_get_sensor_data(BMI160_BOTH_ACCEL_AND_GYRO, &dest_mem->dest_struct.accel_data,&dest_mem->dest_struct.gyro_data, &m_bmi160_sensor);
    // TODO: Try using FIFO and interrupts.
    // length is 6 for gyro and accel (6 dimensions * 2 bytes / dimension) + 3 for the 24-bit sensor time.

    rslt = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, (uint8_t*) dest, 12 + 3, &m_bmi160_sensor);
    APP_ERROR_CHECK(rslt);
};

void BMI160_Get_Unpacked_Data(int16_t * dest){
    
    uint8_t rslt = BMI160_OK;
    uint8_t data;

    // Check that the I2C still appears to be working
    rslt = I2C_Read(BMI160_I2C_ADDR, BMI160_CHIP_ID_ADDR, &data, 1);

#if NBL_LOG_BOARD == 1 
    if (BMI160_CHIP_ID != data) {
      SEGGER_RTT_printf(0,"BMI160 no longer appears to be responding to I2C - CHIP_ID returned 0x%02x\n", data);
    }
#endif     

    // TODO: 
    // This is silly. The bmi160 drivers read the data as bytes and combines them into half 
    // words and words, only for them to be unpacked again here. Better to bypass the driver 
    // API and call the lower function directly.
    rslt = bmi160_get_sensor_data(BMI160_BOTH_ACCEL_AND_GYRO, &m_accel, &m_gyro, &m_bmi160_sensor);

    APP_ERROR_CHECK(rslt);
        
    dest[0]  = (m_accel.x >> 0) & 0xff;
    dest[1]  = (m_accel.x >> 8) & 0xff;
    dest[2]  = (m_accel.y >> 0) & 0xff;
    dest[3]  = (m_accel.y >> 8) & 0xff;
    dest[4]  = (m_accel.z >> 0) & 0xff;
    dest[5]  = (m_accel.z >> 8) & 0xff;
    dest[6]  = (((uint32_t) m_accel.sensortime >>  0) & 0x0000ff);
    dest[7]  = (((uint32_t) m_accel.sensortime >>  8) & 0x0000ff);
    dest[8]  = (((uint32_t) m_accel.sensortime >> 16) & 0x0000ff);
    dest[9]  = (((uint32_t) m_accel.sensortime >> 24) & 0x0000ff);

    dest[10] = (m_gyro.x >> 0) & 0xff;
    dest[11] = (m_gyro.x >> 8) & 0xff;
    dest[12] = (m_gyro.y >> 0) & 0xff;
    dest[13] = (m_gyro.y >> 8) & 0xff;
    dest[14] = (m_gyro.z >> 0) & 0xff;
    dest[15] = (m_gyro.z >> 8) & 0xff;
    dest[16] = (((uint32_t) m_gyro.sensortime >>  0) & 0x0000ff);
    dest[17] = (((uint32_t) m_gyro.sensortime >>  8) & 0x0000ff);
    dest[18] = (((uint32_t) m_gyro.sensortime >> 16) & 0x0000ff);
    dest[19] = (((uint32_t) m_gyro.sensortime >> 24) & 0x0000ff);
        
    //SEGGER_RTT_printf(0, "BMI160 Data x=%d; y=%d; z=%d\n", m_accel.x, m_accel.y, m_accel.z);   


};

/* An example for configuring FOC for accel and gyro data */
int8_t BMI160_FastOffsetCompensation(struct bmi160_dev *dev)
{
    int8_t rslt = 0;
    /* FOC configuration structure */
    struct bmi160_foc_conf foc_conf;
    /* Structure to store the offsets */
    struct bmi160_offsets offsets;

    /* Enable FOC for accel with target values of z = 1g ; x,y as 0g */
    foc_conf.acc_off_en = BMI160_ENABLE;
    foc_conf.foc_acc_x  = BMI160_FOC_ACCEL_0G;
    foc_conf.foc_acc_y  = BMI160_FOC_ACCEL_0G;
    foc_conf.foc_acc_z  = BMI160_FOC_ACCEL_POSITIVE_G;
    
    /* Enable FOC for gyro */
    foc_conf.foc_gyr_en = BMI160_ENABLE;
    foc_conf.gyro_off_en = BMI160_ENABLE;

    // Add a delay befor calling this.
    dev->delay_ms(125);

#if NBL_LOG_BOARD == 1 
    SEGGER_RTT_printf(0,"\n Starting FOC... ");
#endif
    rslt = bmi160_start_foc(&foc_conf, &offsets, dev);
#if NBL_LOG_BOARD == 1 
    SEGGER_RTT_printf(0,"Done\n ");
    
    if (rslt == BMI160_OK) {
        SEGGER_RTT_printf(0,"\n FOC DONE SUCCESSFULLY ");
        SEGGER_RTT_printf(0,"\n OFFSET VALUES AFTER FOC : ");
        SEGGER_RTT_printf(0,"\n OFFSET VALUES ACCEL X : %d ",offsets.off_acc_x);
        SEGGER_RTT_printf(0,"\n OFFSET VALUES ACCEL Y : %d ",offsets.off_acc_y);
        SEGGER_RTT_printf(0,"\n OFFSET VALUES ACCEL Z : %d ",offsets.off_acc_z);
        SEGGER_RTT_printf(0,"\n OFFSET VALUES GYRO  X : %d ",offsets.off_gyro_x);
        SEGGER_RTT_printf(0,"\n OFFSET VALUES GYRO  Y : %d ",offsets.off_gyro_y);
        SEGGER_RTT_printf(0,"\n OFFSET VALUES GYRO  Z : %d \n",offsets.off_gyro_z);    
    } else {
        SEGGER_RTT_printf(0,"\n Fast Offset Compensation FAILED - returned : %d \n",rslt);  
    }
#endif
    
    /* After start of FOC offsets will be updated automatically and 
     * the data will be very much close to the target values of measurement */

    return rslt;
}

