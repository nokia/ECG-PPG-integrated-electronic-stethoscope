/***************************************************************************//**
* @file maxm86161_i2c.c
* @brief I2C device setup for maxm86161 driver.
* @version 1.0
*******************************************************************************
* # License
* <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
*******************************************************************************
*
* SPDX-License-Identifier: Zlib
*
* The licensor of this software is Silicon Laboratories Inc.
*
* This software is provided \'as-is\', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
*    claim that you wrote the original software. If you use this software
*    in a product, an acknowledgment in the product documentation would be
*    appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
*    misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*
*******************************************************************************
*
* EVALUATION QUALITY
* This code has been minimally tested to ensure that it builds with the specified dependency versions and is suitable as a demonstration for evaluation purposes only.
* This code will be maintained at the sole discretion of Silicon Labs.
*
******************************************************************************/
#include "maxm86161.h"
#include "maxm86161_if.h"
#include "string.h"
#include "nrf_log.h"


static uint32_t maxm86161_i2c_write_byte_data(uint8_t address, uint8_t data);
static uint32_t maxm86161_i2c_read_byte_data(uint8_t address, uint8_t *data);
static uint32_t maxm86161_i2c_write_i2c_block_data(uint8_t address, uint8_t length, uint8_t *values);
static uint32_t maxm86161_i2c_read_i2c_block_data(uint8_t address, uint16_t length, uint8_t* values);


/**************************************************************************//**
 * @brief Write to Maxim register
 *****************************************************************************/
uint32_t maxm86161_i2c_write_to_register(uint8_t address, uint8_t data)
{
  return maxm86161_i2c_write_byte_data(address, data);
}

/**************************************************************************//**
 * @brief Read from Maxim register.
 *****************************************************************************/
uint8_t maxm86161_i2c_read_from_register(uint8_t address)
{
  uint8_t data = 0;
  maxm86161_i2c_read_byte_data(address, &data);

  return data;
}

/**************************************************************************//**
 * @brief block write to maxim
 * Block writes should never be used.
 *****************************************************************************/
uint32_t maxm86161_i2c_block_write( uint8_t address, uint8_t length, uint8_t *values)
{
  return maxm86161_i2c_write_i2c_block_data(address, length, values);
}

/**************************************************************************//**
 * @brief Block read from Maxim.
 *****************************************************************************/
uint32_t maxm86161_i2c_block_read(uint8_t address, uint16_t length, uint8_t *values)
{
  return maxm86161_i2c_read_i2c_block_data(address, length, values);
}

/**************************************************************************//**
 * @brief Write to Maxim i2c.
 *****************************************************************************/
static uint32_t maxm86161_i2c_write_byte_data(uint8_t address, uint8_t data)
{
     ret_code_t error_code = 0;
     writeByte(MAXM86161_SLAVE_ADDRESS, address, data);  //hope this one works
      // writeByte(uint8_t address, uint8_t subAddress, uint8_t data)        
	
       	return error_code;
}

/**************************************************************************//**
 * @brief read byte from maxim i2c.
 *****************************************************************************/
static uint32_t maxm86161_i2c_read_byte_data(uint8_t address, uint8_t *data)
{
    ret_code_t error_code;
    error_code = I2C_Read(MAXM86161_SLAVE_ADDRESS,address, data, 1);
    //NRF_LOG_DEBUG("I2C Read error: 0x%x ",error_code);
    //NRF_LOG_DEBUG("data: 0x%x",*data);

    return error_code;
}

/**************************************************************************//**
 * @brief Write block data to Maxim i2c.
 *****************************************************************************/
static uint32_t maxm86161_i2c_write_i2c_block_data(uint8_t address, uint8_t length, uint8_t *data)
{
        ret_code_t err_code;
	uint16_t len = length;

        err_code = I2C_Write(MAXM86161_SLAVE_ADDRESS,address, data, len);
//        uint8_t I2C_Write(uint8_t slv_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)

  	return err_code;
}

/**************************************************************************//**
 * @brief read block data from to maxim i2c.
 *****************************************************************************/
static uint32_t maxm86161_i2c_read_i2c_block_data(uint8_t address, uint16_t length, uint8_t* data)
{
	
        ret_code_t err_code = 0;
        //err_code = I2C_Read(MAXM86161_SLAVE_ADDRESS,address, data, length);
	err_code = readBytes(MAXM86161_SLAVE_ADDRESS,address, data, length);
        return err_code;
}
