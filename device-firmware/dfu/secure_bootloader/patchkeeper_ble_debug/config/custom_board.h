/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif


#include "nrf_gpio.h"
#define NRF_NUM_GPIO_PINS        48

// config which board is in use:
// For Patchkeeper Hardware versions 

//#define CONFIG_BOARD_VERSION_0        // This is for hand modified patchkeeper V1.0
//#define CONFIG_BOARD_VERSION_1        // This is for  patchkeeper V1.1
#define CONFIG_BOARD_VERSION_2        // This is for Patchkeeper V2.0

// Board Components Selection
#define BOARD_HAS_ADS1292    1  // ADS1292R for ECG
#define BOARD_HAS_BATMON     1  // Battery monitor circuits
#define BOARD_HAS_BMI160     1  // IMU sensor
  // IMU device
#define BOARD_HAS_PDM_MIC    1  // Microphone

#define BOARD_HAS_PPG_SENSOR 1
#define PPG_FOR_HR_ONLY      1
#define PPG_FOR_SPO2_ONLY    0


#ifndef BOARD_HAS_ANA_SENSOR
#define BOARD_HAS_ANA_SENSOR  0 
#endif
#if BOARD_HAS_ANA_SENSOR == 1 
#define BOARD_HAS_GSR_SENSOR    0
#define BOARD_HAS_PULSE_SENSOR  0
#define BOARD_HAS_STRAIN_SENSOR 0
#define BOARD_HAS_TEMP_SENSOR   1
#else 
#define BOARD_HAS_GSR_SENSOR    0
#define BOARD_HAS_PULSE_SENSOR  0
#define BOARD_HAS_STRAIN_SENSOR 0
#define BOARD_HAS_TEMP_SENSOR   0
#endif 

#define BOARD_NAME "Belt"

//baoard version selection
#ifdef CONFIG_BOARD_VERSION_2 // _2 is for Patchkeeper_V2.0 hardware


#define OSC_XL1                  NRF_GPIO_PIN_MAP(0,0)
#define OSC_XL2                  NRF_GPIO_PIN_MAP(0,1)

//#define ANA_DIG2                 NRF_GPIO_PIN_MAP(0,4)
//#define ANA_DIG3                 NRF_GPIO_PIN_MAP(0,5)

#define TWI_SDA                  NRF_GPIO_PIN_MAP(0,26)
#define TWI_SCL                  NRF_GPIO_PIN_MAP(0,27)
#define NFC1                     NRF_GPIO_PIN_MAP(0,9)
#define NFC2                     NRF_GPIO_PIN_MAP(0,10)
#define SPI_SCK                  NRF_GPIO_PIN_MAP(0,8)
#define IMU_INT1                 NRF_GPIO_PIN_MAP(0,24)
#define IMU_INT2                 NRF_GPIO_PIN_MAP(0,22)
#define SPI_MOSI                 NRF_GPIO_PIN_MAP(1,1)
#define SPI_MISO                 NRF_GPIO_PIN_MAP(1,2)
#define SPI2_MOSI                NRF_GPIO_PIN_MAP(0,11) // DIG1
#define SPI2_SCK                 NRF_GPIO_PIN_MAP(1,14) //DIG2
#define SPI2_MISO                NRF_GPIO_PIN_MAP(1,5)  //DIG3
//#define DIG4                     NRF_GPIO_PIN_MAP(1,4)
//#define DIG5                     NRF_GPIO_PIN_MAP(1,8)  // V1.0: (1,13)
#define PPG_LDO_EN               NRF_GPIO_PIN_MAP(1,8)   //DIG5
#define PPG_INT                  NRF_GPIO_PIN_MAP(1,4)  //DIG4 
#define DIG6                     NRF_GPIO_PIN_MAP(1,10) //V1.0: (1,12)
#define DIG7                     NRF_GPIO_PIN_MAP(0,12)
#define DIG8                     NRF_GPIO_PIN_MAP(1,7)  //V1.0: (1,11)
#define DIG_ANA2                 NRF_GPIO_PIN_MAP(0,31)
#define DIG_ANA3                 NRF_GPIO_PIN_MAP(0,05)

#define RESET                    NRF_GPIO_PIN_MAP(0,18)
#define CS_SD                    NRF_GPIO_PIN_MAP(1,15)
#define SD_DETECTION             NRF_GPIO_PIN_MAP(1,3)
#define MIC_PWR_CTRL             NRF_GPIO_PIN_MAP(1,11)
#define MIC_DOUT                 NRF_GPIO_PIN_MAP(1,12)   //V1.0: (0,30)
#define MIC_CLK                  NRF_GPIO_PIN_MAP(0,25)
#define SWO                      NRF_GPIO_PIN_MAP(1,0)

//Analog control channels for CUSTOM_BOARD
#define GSR_AMP_EN                      NRF_GPIO_PIN_MAP(0,7)
#define TEMP_AMP_EN                     NRF_GPIO_PIN_MAP(0,3)
#define STRAIN_AMP_EN                   NRF_GPIO_PIN_MAP(1,13)  //V1.0 (0,29)
#define PULSE_AMP_EN                    NRF_GPIO_PIN_MAP(1,06)
#define BAT_MON_EN                      NRF_GPIO_PIN_MAP(0,20)

//Aalogue input for all sensors (for version V1.1)
#define NRF_SAADC_GSR_OUT               NRF_SAADC_INPUT_AIN2
#define NRF_SAADC_PULSE_OUT             NRF_SAADC_INPUT_AIN5  //v1.0 AIN3
#define NRF_SAADC_STRAIN_OUT            NRF_SAADC_INPUT_AIN4
#define NRF_SAADC_TEMP_OUT              NRF_SAADC_INPUT_AIN0
#define NRF_SAADC_BATT_OUT              NRF_SAADC_INPUT_AIN6 //V1.0: AIN7

//ECG_ADS_1292 CONNECTION
#define ADS_DRDY                        NRF_GPIO_PIN_MAP(0,14)
#define ADS_START                       NRF_GPIO_PIN_MAP(0,16)
#define ADS_PWDN                        NRF_GPIO_PIN_MAP(0,23)
#define CS_ADS                          NRF_GPIO_PIN_MAP(0,15)


// LEDs definitions for CUSTOM_BOARD
#define LEDS_NUMBER    3

#define LED_R          NRF_GPIO_PIN_MAP(0,13) //Red
#define LED_G          NRF_GPIO_PIN_MAP(1,9) //Green
#define LED_B          NRF_GPIO_PIN_MAP(0,21) //Blue
#define LED_START      29
#define LED_STOP       31


#define LEDS_ACTIVE_STATE 0
#define LEDS_INV_MASK  LEDS_MASK

// SDC SPI Aliases
#define SDC_SCK_PIN                     SPI2_SCK   ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN                    SPI2_MOSI  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN                    SPI2_MISO  ///< SDC serial data out (DO) pin.
//#define SDC_CS_PIN                      DIG6 //  //used for testing on PKV1.1 with extyernal SD card
#define SDC_CS_PIN                      CS_SD     ///< SDC chip select (CS) pin.

#endif

#ifdef CONFIG_BOARD_VERSION_1

#define OSC_XL1                  NRF_GPIO_PIN_MAP(0,0)
#define OSC_XL2                  NRF_GPIO_PIN_MAP(0,1)

//#define ANA_DIG2                 NRF_GPIO_PIN_MAP(0,4)
//#define ANA_DIG3                 NRF_GPIO_PIN_MAP(0,5)

#define TWI_SDA                  NRF_GPIO_PIN_MAP(0,26)
#define TWI_SCL                  NRF_GPIO_PIN_MAP(0,27)
#define NFC1                     NRF_GPIO_PIN_MAP(0,9)
#define NFC2                     NRF_GPIO_PIN_MAP(0,10)
#define SPI_SCK                  NRF_GPIO_PIN_MAP(0,8)
#define IMU_INT1                 NRF_GPIO_PIN_MAP(0,24)
#define IMU_INT2                 NRF_GPIO_PIN_MAP(0,22)
#define SPI_MOSI                 NRF_GPIO_PIN_MAP(1,1)
#define SPI_MISO                 NRF_GPIO_PIN_MAP(1,2)
#define DIG1                     NRF_GPIO_PIN_MAP(0,11) // DIG1
#define DIG2                     NRF_GPIO_PIN_MAP(1,14) //DIG2
#define DIG3                      NRF_GPIO_PIN_MAP(1,5)  //DIG3
#define DIG4                     NRF_GPIO_PIN_MAP(1,4)
#define DIG5                     NRF_GPIO_PIN_MAP(1,8)  // V1.0: (1,13)
#define DIG6                     NRF_GPIO_PIN_MAP(1,10) //V1.0: (1,12)
#define DIG7                     NRF_GPIO_PIN_MAP(0,12)
#define DIG8                     NRF_GPIO_PIN_MAP(1,7)  //V1.0: (1,11)
#define DIG_ANA2                 NRF_GPIO_PIN_MAP(0,31)
#define DIG_ANA3                 NRF_GPIO_PIN_MAP(0,05)

#define RESET                    NRF_GPIO_PIN_MAP(0,18)
#define CS_SD                    NRF_GPIO_PIN_MAP(1,15)
#define SD_DETECTION             NRF_GPIO_PIN_MAP(1,3)
#define MIC_PWR_CTRL             NRF_GPIO_PIN_MAP(1,11)
#define MIC_DOUT                 NRF_GPIO_PIN_MAP(1,12)   //V1.0: (0,30)
#define MIC_CLK                  NRF_GPIO_PIN_MAP(0,25)
#define SWO                      NRF_GPIO_PIN_MAP(1,0)

//Analog control channels for CUSTOM_BOARD
#define GSR_AMP_EN                      NRF_GPIO_PIN_MAP(0,7)
#define TEMP_AMP_EN                     NRF_GPIO_PIN_MAP(0,3)
#define STRAIN_AMP_EN                   NRF_GPIO_PIN_MAP(1,13)  //V1.0 (0,29)
#define PULSE_AMP_EN                    NRF_GPIO_PIN_MAP(1,06)
#define BAT_MON_EN                      NRF_GPIO_PIN_MAP(0,20)

//Aalogue input for all sensors (for version V1.1)
#define NRF_SAADC_GSR_OUT               NRF_SAADC_INPUT_AIN2
#define NRF_SAADC_PULSE_OUT             NRF_SAADC_INPUT_AIN5  //v1.0 AIN3
#define NRF_SAADC_STRAIN_OUT            NRF_SAADC_INPUT_AIN4
#define NRF_SAADC_TEMP_OUT              NRF_SAADC_INPUT_AIN0
#define NRF_SAADC_BATT_OUT              NRF_SAADC_INPUT_AIN6 //V1.0: AIN7

//ECG_ADS_1292 CONNECTION
#define ADS_DRDY                        NRF_GPIO_PIN_MAP(0,14)
#define ADS_START                       NRF_GPIO_PIN_MAP(0,16)
#define ADS_PWDN                        NRF_GPIO_PIN_MAP(0,23)
#define CS_ADS                          NRF_GPIO_PIN_MAP(0,15)


// LEDs definitions for CUSTOM_BOARD
#define LEDS_NUMBER    3

#define LED_R          NRF_GPIO_PIN_MAP(0,13) //Red
#define LED_G          NRF_GPIO_PIN_MAP(1,9) //Green
#define LED_B          NRF_GPIO_PIN_MAP(0,21) //Blue
#define LED_START      29
#define LED_STOP       31


#define LEDS_ACTIVE_STATE 0
#define LEDS_INV_MASK  LEDS_MASK

// SDC SPI Aliases
#define SDC_SCK_PIN                     SPI_SCK   ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN                    SPI_MOSI  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN                    SPI_MISO  ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN                      CS_SD     ///< SDC chip select (CS) pin.

#endif 

#ifdef CONFIG_BOARD_VERSION_0 
 //FOR KATCHKEEPER VERIONS 1.0 MANAUL MODIFIED TWO BOARDS FOR TESTING

#define OSC_XL1                  NRF_GPIO_PIN_MAP(0,0)
#define OSC_XL2                  NRF_GPIO_PIN_MAP(0,1)
//#define CS_EXT                   NRF_GPIO_PIN_MAP(0,12) //shared PIN with D7
//#define ANA_DIG1                 3
//#define ANA_DIG2                 NRF_GPIO_PIN_MAP(0,4)
//#define ANA_DIG3                 NRF_GPIO_PIN_MAP(0,5)

#define TWI_SDA                  NRF_GPIO_PIN_MAP(0,26)
#define TWI_SCL                  NRF_GPIO_PIN_MAP(0,27)
#define NFC1                     NRF_GPIO_PIN_MAP(0,9)
#define NFC2                     NRF_GPIO_PIN_MAP(0,10)
#define SPI_SCK                  NRF_GPIO_PIN_MAP(0,8)
#define IMU_INT1                 NRF_GPIO_PIN_MAP(0,24)
#define IMU_INT2                 NRF_GPIO_PIN_MAP(0,22)
#define SPI_MOSI                 NRF_GPIO_PIN_MAP(1,1)
#define SPI_MISO                 NRF_GPIO_PIN_MAP(1,2)
#define DIG1                     NRF_GPIO_PIN_MAP(0,11)
#define DIG2                     NRF_GPIO_PIN_MAP(1,14)
#define DIG3                     NRF_GPIO_PIN_MAP(1,5)
#define DIG4                     NRF_GPIO_PIN_MAP(1,4)
#define DIG5                     NRF_GPIO_PIN_MAP(1,13)
#define DIG6                     NRF_GPIO_PIN_MAP(1,12)
#define DIG7                     NRF_GPIO_PIN_MAP(0,12)
#define DIG8                     NRF_GPIO_PIN_MAP(1,11)
//#define PRES_INT                 18
//#define CS_MEM                   20
#define RESET                    NRF_GPIO_PIN_MAP(0,18)
//#define COLOUR_INT               22
#define CS_SD                    NRF_GPIO_PIN_MAP(1,15)
#define SD_DETECTION             NRF_GPIO_PIN_MAP(1,3)
#define MIC_PWR_CTRL             NRF_GPIO_PIN_MAP(1,11)
//#define MIC_PWR_CTRL             NRF_GPIO_PIN_MAP(0,31)
#define MIC_DOUT                 NRF_GPIO_PIN_MAP(0,30)
#define MIC_CLK                  NRF_GPIO_PIN_MAP(0,25)
#define SWO                          NRF_GPIO_PIN_MAP(1,0)

//Analog channels for CUSTOM_BOARD
#define TEMP_AMP_EN                     NRF_GPIO_PIN_MAP(0,3)
//#define TEMP_ANALOG                     NRF_GPIO_PIN_MAP(0,2)

#define GSR_AMP_EN                      NRF_GPIO_PIN_MAP(0,7)
//#define GSR_ANALOG                      NRF_GPIO_PIN_MAP(0,4)

//#define GSR_ANALOG                    NRF_GPIO_PIN_MAP(1,8)

#define STRAIN_AMP_EN                   NRF_GPIO_PIN_MAP(0,29)
//#define STRAIN_ANALOG                   NRF_GPIO_PIN_MAP(0,28)

#define PULSE_AMP_EN                    NRF_GPIO_PIN_MAP(1,06)
//#define PULSE_ANALOG                    NRF_GPIO_PIN_MAP(0,5)
//#define PULSE_ANALOG                  NRF_GPIO_PIN_MAP(1,10)
#define BAT_MON_EN                      NRF_GPIO_PIN_MAP(0,20)

//Aalogue input for all sensors (for version V1.0_b, manual modified V1.0 for testing))
#define NRF_SAADC_GSR_OUT               NRF_SAADC_INPUT_AIN2
#define NRF_SAADC_PULSE_OUT             NRF_SAADC_INPUT_AIN3
#define NRF_SAADC_STRAIN_OUT            NRF_SAADC_INPUT_AIN4
#define NRF_SAADC_TEMP_OUT              NRF_SAADC_INPUT_AIN0
#define NRF_SAADC_BATT_OUT              NRF_SAADC_INPUT_AIN7

//ECG_ADS_1292 CONNECTION
#define ADS_DRDY                        NRF_GPIO_PIN_MAP(0,14)
#define ADS_START                       NRF_GPIO_PIN_MAP(0,16)
#define ADS_PWDN                        NRF_GPIO_PIN_MAP(0,23)
#define CS_ADS                          NRF_GPIO_PIN_MAP(0,15)

// LEDs definitions for CUSTOM_BOARD
#define LEDS_NUMBER    3

#define LED_R          NRF_GPIO_PIN_MAP(0,13) //Red
#define LED_G          NRF_GPIO_PIN_MAP(1,9) //Green
#define LED_B          NRF_GPIO_PIN_MAP(0,21) //Blue
#define LED_START      29
#define LED_STOP       31


#define LEDS_ACTIVE_STATE 0
#define LEDS_INV_MASK  LEDS_MASK

// SDC SPI Aliases
#define SDC_SCK_PIN                     SPI_SCK   ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN                    SPI_MOSI  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN                    SPI_MISO  ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN                      CS_SD     ///< SDC chip select (CS) pin.

#endif


// BLE BSP uses first LED in this list to
// indicate BLE status. Nice if this was Blue.

#define LEDS_LIST { LED_B, LED_G, LED_R }
//#define LEDS_LIST { LED_G, LED_B, LED_R } // swapt blue and green on ble indicators


#define LED_BLUE       0
#define LED_GREEN      1
#define LED_RED        2

#define BSP_LED_0      LED_BLUE
#define BSP_LED_1      LED_GREEN
#define BSP_LED_2      LED_RED

#define BUTTONS_NUMBER 0
#define BUTTONS_ACTIVE_STATE 1
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP


#define BATT_VOLTAGE_DIVIDER_R1         10000
#define BATT_VOLTAGE_DIVIDER_R2         21500

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}



#ifdef __cplusplus
}
#endif

#endif // CUSTOM_BOARD_H
