/*
 * © 2022 Nokia
 * Licensed under the BSD 3-Clause License
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */


#include <stdint.h>
#include "boards.h"
#include "nrf_mbr.h"
#include "nrf_bootloader.h"
#include "nrf_bootloader_app_start.h"
#include "nrf_bootloader_dfu_timers.h"
#include "nrf_dfu.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_error.h"
#include "app_error_weak.h"
#include "nrf_bootloader_info.h"
#include "nrf_delay.h"

//#include "dfu_conf.h"
    
void GPIO_init(void);

static void on_error(void)
{
    NRF_LOG_FINAL_FLUSH();

#if NRF_MODULE_ENABLED(NRF_LOG_BACKEND_RTT)
    // To allow the buffer to be flushed by the host.
    nrf_delay_ms(100);
#endif
#ifdef NRF_DFU_DEBUG_VERSION
    NRF_BREAKPOINT_COND;
#endif
    NVIC_SystemReset();
}


void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    NRF_LOG_ERROR("%s:%d", p_file_name, line_num);
    on_error();
}


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    NRF_LOG_ERROR("Received a fault! id: 0x%08x, pc: 0x%08x, info: 0x%08x", id, pc, info);
    on_error();
}


void app_error_handler_bare(uint32_t error_code)
{
    NRF_LOG_ERROR("Received an error: 0x%08x!", error_code);
    on_error();
}

/**
 * @brief Function notifies certain events in DFU process.
 */
static void dfu_observer(nrf_dfu_evt_type_t evt_type)
{
    switch (evt_type)
    {
        case NRF_DFU_EVT_DFU_FAILED:
        case NRF_DFU_EVT_DFU_ABORTED:
        case NRF_DFU_EVT_DFU_INITIALIZED:
            bsp_board_init(BSP_INIT_LEDS);
            bsp_board_led_on(BSP_BOARD_LED_0);
            bsp_board_led_on(BSP_BOARD_LED_1);
            bsp_board_led_off(BSP_BOARD_LED_2);
            break;
        case NRF_DFU_EVT_TRANSPORT_ACTIVATED:
            bsp_board_led_off(BSP_BOARD_LED_1);
            bsp_board_led_on(BSP_BOARD_LED_2);
            break;
        case NRF_DFU_EVT_DFU_STARTED:
            break;
        default:
            break;
    }
}

ret_code_t nrf_bootloader_write_bl_addr_to_mbr(void){

    // Check if addresses already have been written to the MBR
    if( (*(volatile uint32_t *)MBR_BOOTLOADER_ADDR != BOOTLOADER_START_ADDR) && 
     (*(volatile uint32_t *)MBR_PARAM_PAGE_ADDR = NRF_MBR_PARAMS_PAGE_ADDRESS))
     {
          // Enable Write with the NVMC
          NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
          while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

          // Write Bootloader start address to MBR 
          *(volatile uint32_t *)MBR_BOOTLOADER_ADDR = BOOTLOADER_START_ADDR;
          // Write MBR parameter page address to MBR
          *(volatile uint32_t *)MBR_PARAM_PAGE_ADDR = NRF_MBR_PARAMS_PAGE_ADDRESS;
          // Revert NVMC to read-only
          NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
          while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
      }

    return NRF_SUCCESS;
}



/**@brief Function for application main entry. */
int main(void)
{
    uint32_t ret_val;


    // Write bootloader start address to the MBR
    ret_val = nrf_bootloader_write_bl_addr_to_mbr();
//    APP_ERROR_CHECK(ret_val);

    // Protect MBR and bootloader code from being overwritten.
    ret_val = nrf_bootloader_flash_protect(0, MBR_SIZE, false);
    APP_ERROR_CHECK(ret_val);
    ret_val = nrf_bootloader_flash_protect(BOOTLOADER_START_ADDR, BOOTLOADER_SIZE, false);
    APP_ERROR_CHECK(ret_val);

    (void) NRF_LOG_INIT(nrf_bootloader_dfu_timer_counter_get);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
 //   conf_init();
 //   GPIO_init();

    NRF_LOG_INFO("Inside main");

    ret_val = nrf_bootloader_init(dfu_observer);
    APP_ERROR_CHECK(ret_val);

    // Either there was no DFU functionality enabled in this project or the DFU module detected
    // no ongoing DFU operation and found a valid main application.
    // Boot the main application.
    nrf_bootloader_app_start();

    // Should never be reached.
    NRF_LOG_INFO("After main");
}

/**
 * @}
 */
