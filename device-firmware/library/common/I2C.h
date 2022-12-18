/*
 * © 2022 Nokia
 * Licensed under the BSD 3-Clause License
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */


#ifndef MA_I2C_H__
#define MA_I2C_H__

#include "boards.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(BOARD_PCA10040)
#  ifndef TWI_SCL
#    define TWI_SCL ARDUINO_SCL_PIN
#  endif 
#  ifndef TWI_SDA
#    define TWI_SDA ARDUINO_SDA_PIN
#  endif 
#elif defined(BOARD_PCA20020)
#  ifndef TWI_SCL
#    define TWI_SCL      15       // SCL signal pin
#  endif 
#  ifndef TWI_SDA
#    define TWI_SDA      14       // SDA signal pin
#  endif 
#endif

//#define DEBUG_I2C

    #include "nrf_drv_twi.h"
    #include "app_error.h" //for APP_ERROR_CHECK(err_code);
    #include "SEGGER_RTT.h"

    void I2C_init( void );
    
    void writeByte( uint8_t address, uint8_t subAddress, uint8_t data);
    
    uint8_t readByte(uint8_t address, uint8_t subAddress);

    uint32_t readBytes( uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t n_bytes);
     
    void I2C_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
    
    uint8_t I2C_Read(uint8_t slv_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

    uint8_t I2C_Write(uint8_t slv_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* MA_I2C_H__ */
