/*
 * © 2022 Nokia
 * Licensed under the BSD 3-Clause License
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "I2C.h"
#include "nrf_drv_twi.h"
#include <stdlib.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"


//#define DEBUG_I2C

#define NRF_LOG_MODULE_NAME i2c
#include "nrf_log.h"

NRF_LOG_MODULE_REGISTER();

// I2C instance.
#define TWI_INSTANCE_ID 1
#define APP_IRQ_PRIORITY_LOW 3  //overrides definition elsewhere


static const nrf_drv_twi_t i2c = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

// Indicates if operation on TWI has ended.
static volatile bool m_xfer_done = false;

// @brief UART initialization.
void I2C_init(void)
{
    //NRF_LOG_DEBUG("twi_init(void)\r\n");
        
    ret_code_t err_code;

    
    // Clear the i2c bus by clocking SCL 100 times.
    // Have to do this before enabling TWI0
    // as we need to drive the GPIO SCL pin directly.

    nrf_gpio_cfg_output(TWI_SCL);

    for (int cycle=0; cycle < 100; cycle++) {
      nrf_gpio_pin_write(TWI_SCL, 1);
      nrf_delay_us(10);
      nrf_gpio_pin_write(TWI_SCL, 0);
      nrf_delay_us(10);
    }
     
    nrf_gpio_cfg_default(TWI_SCL);

    const nrf_drv_twi_config_t i2c_config = 
    {
       .scl                = TWI_SCL,
       .sda                = TWI_SDA,
       .frequency          = NRFX_TWI_DEFAULT_CONFIG_FREQUENCY,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW,
       .clear_bus_init     = false
    };

    //last one is some kind of context - no idea what that is....
    //documentation is vague/nonexistant
    err_code = nrf_drv_twi_init(&i2c, &i2c_config, I2C_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&i2c); 

    
    //NRF_LOG_DEBUG("I2C_init(void) done\r\n");
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    uint8_t temp[2];
    
    temp[0] = subAddress;
    temp[1] = data;
    
    ret_code_t err_code;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&i2c, address, &temp[0], 2, true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false); //wait until end of transfer
}

 
uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    ret_code_t err_code = 0;
    
    uint8_t value;
    
    m_xfer_done = false;
    //NRF_LOG_DEBUG("readByte - Writing\r\n");
    //last position is the transfer pending flag
    err_code = nrf_drv_twi_tx(&i2c, address, &subAddress, 1, true);
    
    APP_ERROR_CHECK(err_code);
    
    while (m_xfer_done == false); //wait until end of transfer
    
    if (err_code == NRF_SUCCESS)
    {
        m_xfer_done = false;
        //NRF_LOG_DEBUG("readByte - Reading\r\n");
        err_code = nrf_drv_twi_rx(&i2c, address, &value, 1);
        APP_ERROR_CHECK(err_code);
        
        while (m_xfer_done == false);
    };
    
    //NRF_LOG_DEBUG("readByte done, returned 0x\r\n");
    //NRF_LOG_HEXDUMP_DEBUG(&value, 1);
    //NRF_LOG_FLUSH();
    return value;
}

// @brief TWI events handler.
void I2C_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            
            //todo -difference between read and write???
            m_xfer_done = true;
            
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //NRF_LOG_INFO("Data just came back!");  
            }
            
            //NRF_LOG_DEBUG("I2C_handler responding to NRF_DRV_TWI_EVT_DONE\r\n");
            break;
        default:
            break;
    }
}



uint32_t readBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t n_bytes )
{
    ret_code_t err_code = 0;
   
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&i2c, address, &subAddress, 1, true);
    
    while (m_xfer_done == false) {};
    
    //comes back with error code
    //SEGGER_RTT_printf(0, "ReadBytes code: %d\n", err_code);
    //here is the problem????
    //APP_ERROR_CHECK(err_code);
    
    if (err_code == NRF_SUCCESS)
    {
        m_xfer_done = false;
        //NRF_LOG_DEBUG("readBytes - Reading\r\n");
        err_code = nrf_drv_twi_rx(&i2c, address, dest, n_bytes);
        //SEGGER_RTT_printf(0, "ReadBytes RX code: %d\n", err_code);
        //APP_ERROR_CHECK(err_code);
        while (m_xfer_done == false) {};
    };

    //NRF_LOG_DEBUG("readBytes done\r\n");
    //NRF_LOG_FLUSH();

    
    return err_code;

}

/**@brief I2C Read API. This is the same as above readbytes
 */
uint8_t I2C_Read(uint8_t slv_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {

    ret_code_t err_code;
     m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&i2c, slv_addr, &reg_addr, 1, true);
    while (m_xfer_done == false) {};
   if (err_code == NRF_SUCCESS)
    {
      m_xfer_done = false;
      err_code = nrf_drv_twi_rx(&i2c, slv_addr, reg_data, len);
      while (m_xfer_done == false) {};
    };


#ifdef DEBUG_I2C
    nrf_delay_ms(10);
    for (int i=0; i<len; i++){
        if (0==i) {
            SEGGER_RTT_printf(0, "\nI2C Read Slave  - length = %d\n", len);
        }
        SEGGER_RTT_printf(0, "0x%02x, Reg 0x%02x, Data 0x%02x\n", slv_addr, reg_addr+i, *(reg_data+i));
    }
#endif
    uint8_t ret =  (err_code >> 24) & 0xff;
    return ret;

}

//ret_code_t I2C_Read_Debug(uint8_t slv_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
//    ret_code_t err_code;

//    err_code = I2C_Read( slv_addr, reg_addr, reg_data, len);
  
//    for (int i=0; i<len; i++){
//        if (0==i) {
//            SEGGER_RTT_printf(0, "\nI2C Read Slave  - length = %d\n", len);
//        }
//        SEGGER_RTT_printf(0, "0x%02x, Reg 0x%02x, Data 0x%02x\n", slv_addr, reg_addr+i, *(reg_data+i));
//    }

//    return err_code;
//}


/**@brief I2C Write API.
 */
uint8_t I2C_Write(uint8_t slv_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {

    ret_code_t err_code;
    uint8_t *packet = (uint8_t*) malloc(sizeof(uint8_t) * (len+1));
    *packet = reg_addr;
    for (int i=0; i < len; i++) {
      *(packet+i+1) = *(reg_data+i);
    }

    err_code = nrf_drv_twi_tx(&i2c, slv_addr, packet, len+1, false);
    //nrf_delay_ms(1);
    while (m_xfer_done == false) {};
    free(packet);
#ifdef DEBUG_I2C
        
    for (int i=0; i<len; i++){
        if (0==i) {
            SEGGER_RTT_printf(0, "\nI2C Write Slave - length = %d\n", len);
        }
        SEGGER_RTT_printf(0, "0x%02x, Reg 0x%02x, Data 0x%02x\n", slv_addr, reg_addr+i, *(reg_data+i));
    }
#endif
    
    //nrf_delay_ms(10);

    uint8_t ret =  (err_code >> 24) & 0xff;
    return ret;


}

//ret_code_t I2C_Write_Debug(uint8_t slv_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {

//    ret_code_t err_code;
//    err_code = I2C_Write(slv_addr, reg_addr, reg_data, len);

//    for (int i=0; i<len; i++){
//        if (0==i) {
//            SEGGER_RTT_printf(0, "\nI2C Write Slave - length = %d\n", len);
//        }
//        SEGGER_RTT_printf(0, "0x%02x, Reg 0x%02x, Data 0x%02x\n", slv_addr, reg_addr+i, *(reg_data+i));
//    }

//    return err_code;
//}
