/*
 * © 2022 Nokia
 * Licensed under the BSD 3-Clause License
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */


#include <stdint.h>
#include <string.h>
#include "sdk_config.h"

#include "nordic_common.h"
#include "nrf.h"
#include "nrfx.h"
//#include "app_error.h"
//#include "nrf_delay.h"
//#include "ble.h"
#include "ble_hci.h"
//#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
//#include "ble_conn_state.h"
//#include "fds.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_drv_wdt.h"
#include "diskio_blkdev.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrfx_pdm.h"
#include "nrf_drv_gpiote.h"
//#include "math.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// OTA DFU SUPPORT
#if BLE_DFU_ENABLED == 1
//#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "ble_dfu.h"
#include "nrf_power.h"
#include "nrf_bootloader_info.h"

#include "peer_manager.h"
#include "peer_manager_handler.h"

#endif

#include "I2C.h"            //everything on the I2C bus
#include "ADC.h"            // ADC for Battery Level Measurement
#include "patchkeeper.h"
#include "timestamp.h"
//#if USE_SDCARD == 1
//#include "nrf_block_dev_sdc.h"
//#include "app_sdcard.h"
//#endif

// FIRMWARE VERSION information define at the beginning of sdk_config.h file
# define FIRMWARE_VERSION_ID FIRMWARE_MAJOR_VERSION * 1000000 + \
                             FIRMWARE_MINOR_VERSION *   10000 + \
                             FIRMWARE_PATCH_NUM *         100 + \
                             FIRMWARE_BUILD_NUM



// Anytime the format of the data is changed in a way that requires a change to the data
// processing script that is NOT backward compatible with older data (i.e. not just adding
// a new sensor) then increment this version code


#if USE_BLE_COMMS == 1
#ifdef BOARD_NAME
#define DEVICE_NAME BOARD_NAME
#else
#define DEVICE_NAME                     "PATCHKEEPER"                                     /**< Name of device. Will be included in the advertising data. */
#endif
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */
#endif

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
//#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


//#define REFORMAT_ON_POR true
#define REFORMAT_USES_DATALOG_MEM
//#define MEMORY_SIZE_KB(n) (n) * 1024
//#define DATALOG_MEMORY_SIZE_BYTES MEMORY_SIZE_KB(5)

typedef uint8_t datalog_memory_t;

#if USE_BLE_COMMS == 1
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

uint8_t   m_ble_wait_for_tx = 0;

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}/*,
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_BLOOD_PRESSURE_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_ALERT_NOTIFICATION_SERVICE, BLE_UUID_TYPE_BLE}*/
};
#endif

uint8_t   m_ble_connected = 0;
uint16_t  m_rtc2_cc2_count = 0;
uint8_t   m_adc_oversample; //FIXME is this used?

// BLE CONTROL COMMANDS
#define SYS_CONFIG_INFO       0x00
#define SYS_LED_ENABLE        0x01
#define IMU_SAMPLE_INTERVAL   0x02
#define SET_TIME_COMMAND      0xff


// Function Declarations
void watchdog_refresh(void);
void led_heartbeat_handler (void * p_context);
static void nus_data_handler(ble_nus_evt_t * p_evt);

//
//#define SDC_SECTOR_SIZE             512     ///< Size of a single SD card block in bytes.
//
#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
typedef uint8_t datalog_memory_t; 
datalog_memory_t datalog_memory_0 [DATALOG_MEMORY_SIZE_BYTES/sizeof(datalog_memory_t)];
datalog_memory_t datalog_memory_1 [DATALOG_MEMORY_SIZE_BYTES/sizeof(datalog_memory_t)];
datalog_memory_t *m_p_datalog_mem;
uint16_t m_write_offset = 0;
#endif //PATCHKEEPER_BUFFER_SENSOR_DATA

#if USE_BLE_COMMS == 1
#if BLE_DFU_ENABLED == 1
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
NRF_LOG_DEBUG("app_shutdown_handler event: %d",event);
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
//             if (!m_ready_for_reset)
//             {
//                  return false;
//             }
//             else
//            {
//            
//                // Device ready to enter
//                uint32_t err_code;
//                err_code = sd_softdevice_disable();
//                APP_ERROR_CHECK(err_code);
//                err_code = app_timer_stop_all();
//                APP_ERROR_CHECK(err_code);
//            }
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context){
  NRF_LOG_DEBUG("In buttonless_dfu_sdh_state_observer with state %d", state);

    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

//NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) = {};
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) = {
  .handler = buttonless_dfu_sdh_state_observer,
  .p_context = NULL,
  };

static void advertising_config_get(ble_adv_modes_config_t * p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled  = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout  = APP_ADV_DURATION;
}


static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}



static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event){
  NRF_LOG_DEBUG("In ble_dfu_evt_handler with event %d", event);
  NRF_LOG_FLUSH();
  nrf_delay_ms(10);
      switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

            // Prevent device from advertising on disconnect.
            ble_adv_modes_config_t config;
            advertising_config_get(&config);
            config.ble_adv_on_disconnect_disabled = true;
            ble_advertising_modes_config_set(&m_advertising, &config);

            // Disconnect all other bonded devices that currently are connected.
            // This is required to receive a service changed indication
            // on bootup after a successful (or aborted) Device Firmware Update.
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }

  }
#endif
#endif


#if USE_BLE_COMMS == 1
static uint32_t ble_send_data(datalog_memory_t *, datalog_memory_t *);
#endif


// Replace NRFX_RTC_US_TO_TICKS.
// If us > 0x1fff then the intermediate expression overflows and truncates giving smaller
// delays than intended.
//#define NBL_NRFX_RTC_US_TO_TICKS(us,freq) (us > 0x1fff) ? (us / 1000000) * freq : (((us) * (freq)) / 1000000U)

#define NBL_NRFX_RTC_US_TO_TICKS(us, freq) (int32_t) ((us / 1000000.0) * freq)

//#define FILENAME_BASENAME "dlog_"
//#define FILENAME_SUFFIX "bin"
//#define MAX_FILESIZE_4GB 0xffffffff /* Corresponds to 4GB-1 */
//#define MAX_FILESIZE_2GB 0x80000000 /* Corresponds to 2GB   */
//#define MAX_FILESIZE MAX_FILESIZE_2GB

#ifndef BOARD_CUSTOM
#  define SDC_SCK_PIN     SPIM0_SCK_PIN   ///< SDC serial clock (SCK) pin.
#  define SDC_MOSI_PIN    SPIM0_MOSI_PIN  ///< SDC serial data in (DI) pin.
#  define SDC_MISO_PIN    SPIM0_MISO_PIN  ///< SDC serial data out (DO) pin.
#  define SDC_CS_PIN      SPIM0_SS_PIN    ///< SDC chip select (CS) pin.
#endif

//Time (in miliseconds) between LED Flashes
#define PULSE_LED_PERIOD_MS 10000
//Duration (in miliseconds) of LED Flashes
#define PULSE_LED_ON_TIME_MS 50
#define PULSE_LED_OFF_TIME_MS (PULSE_LED_PERIOD_MS - PULSE_LED_ON_TIME_MS)

#define DEFAULT_WATCHDOG_REFRESH_PERIOD_MS      2000

ts64_t m_timestamp_offset;
ts64_t m_old_timestamp_offset;
uint16_t m_imu_sensor_sample_period_ms = PATCHKEEPER_DEFAULT_IMU_SAMPLE_INTERVAL_MS;

// Variable to indicate if watchdog is running
uint8_t m_watchdog_running = 0;

//// Variable to determine name of files written to SDCard
//uint16_t m_fileindex = 0;


//patchkeeper_context_t m_main_patchkeeper_context;

// Variable to store reason for last reset
uint32_t m_last_reset_reason;

//enum reset_type { PowerOnReset=0,
//                  ResetPin=1,
//                  WatchdogReset=2,
//                  SoftwareReset=4,
//                  CPULockupReset=8,
//                  GPIOWakeUp=0x10000,
//                  LPCompWakeUp=0x20000,
//                  DebugWakeUp=0x40000,
//                  NFCWakeUp=0x80000
//                  };;

/**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. */
#define COMPARE_COUNTERTIME  (30UL)
#define RTC_CONFIG_PRESCALER 0

#ifndef BOARD_CUSTOM
// Define any leds as we don't know what colour they are.
// These are already defined for BOARD_CUSTOM.
#define LED_GREEN      0
#define LED_BLUE       1
#define LED_RED        2
#endif

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    NRF_LOG_ERROR("ASSERTION ERROR %s:%d", p_file_name, line_num);
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for SystemView Timestamp Retrieval.
 *
*/

#if DEBUG_WITH_SYSTEMVIEW == 1

#include "SEGGER_SYSVIEW_Conf.h"
#include "SEGGER_SYSVIEW.h"

#endif


/**@brief Function for initializing the timer module.
 */
static void app_timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


#if USE_BLE_COMMS == 1
/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    ble_gap_addr_t ble_addr;

    err_code = sd_ble_gap_addr_get(&ble_addr);

    char app_string[5];
    char tmp_string [50];

    sprintf(tmp_string, "MAC ID:- ");
    for (int i=5; i>=0; i--) {
      sprintf(app_string, "%02x", ble_addr.addr[i]);
      strcat(tmp_string, app_string);
      if (i) {
        sprintf(app_string, ":");
        strcat(tmp_string, app_string);

      }
    }

    NRF_LOG_INFO("%s\0", tmp_string);

    sprintf(tmp_string, "%s_%02x%02x", DEVICE_NAME, ble_addr.addr[1], ble_addr.addr[0]);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) tmp_string,
                                          strlen(tmp_string));


    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}



/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

#if BLE_DFU_ENABLED == 1

    ble_dfu_buttonless_init_t dfus_init = {0};


    // Initialize the async SVCI interface to bootloader.
//    err_code = ble_dfu_buttonless_async_svci_init();
//    APP_ERROR_CHECK(err_code);

    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
#endif

}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        m_ble_connected = 0;
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
//    else if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED)
//    {
//        m_ble_connected = 1;
//    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}
#endif

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEBUG("Sleeping / Waiting for Event...\n");

    __WFE();
    // Prepare wakeup buttons.
//    err_code = bsp_btn_ble_sleep_mode_prepare();
//    APP_ERROR_CHECK(err_code);
//
//    // Go to system-off mode (this function will not return; wakeup will cause a reset).
//    err_code = sd_power_system_off();
//    APP_ERROR_CHECK(err_code);
}

#if USE_BLE_COMMS == 1
/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
           err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
           APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
//            sleep_mode_enter();
            break;
        default:
            break;
    }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_INFO("Connecting...");
#endif
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_INFO("Disconnected");
#endif
            m_ble_connected = 0;
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_DEBUG("PHY update request.");
#endif
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            // Wake up CPU
//            NRF_LOG_DEBUG("TX Complete Event\n");
            m_ble_wait_for_tx = 0;
            __SEV();
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_DEBUG("BLE GATTC Write Response Event\n");
#endif
            break;

        case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_DEBUG("BLE GATTC MTU Exchange Response Event\n");
#endif
            break;

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_DEBUG("BLE GAP Data Length Update Event\n");
#endif
            break;

        case BLE_GATTS_EVT_WRITE:
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_DEBUG("BLE Write Event\n");
#endif
            break;

        case BLE_GAP_EVT_PHY_UPDATE:
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_DEBUG("BLE GAP PHY Update Event\n");
#endif
            break;

        case BLE_NUS_EVT_TX_RDY:
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_DEBUG("BLE GAP Connected Event\n");
#endif
            break;

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_DEBUG("BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST\n");
#endif
            break;

//        case BLE_GAP_EVT_PHY_UPDATE:
//            NRF_LOG_DEBUG("\n");
//            break;
//        case BLE_GAP_EVT_PHY_UPDATE:
//            NRF_LOG_DEBUG("\n");
//            break;
//        case BLE_GAP_EVT_PHY_UPDATE:
//            NRF_LOG_DEBUG("\n");
//            break;
//        case BLE_GAP_EVT_PHY_UPDATE:
//            NRF_LOG_DEBUG("\n");
//            break;
        default:
            // No implementation needed.
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_DEBUG("Event id 0x%x Received\n", p_ble_evt->header.evt_id);
#endif
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEBUG("ram_start hex: 0x%x",ram_start);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    NRF_LOG_DEBUG("Returning from ble_stack_init", err_code);
    NRF_LOG_FLUSH();
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init()
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    NRF_LOG_DEBUG("Starting Peer Manager");
    NRF_LOG_FLUSH();

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEBUG("Peer Manager Initialized");
    NRF_LOG_FLUSH();
}

/** @brief Clear bonding information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


///**@brief   Function for handling app_uart events.
// *
// * @details This function will receive a single character from the app_uart module and append it to
// *          a string. The string will be be sent over BLE when the last character received was a
// *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
// */
//void uart_event_handle(app_uart_evt_t * p_event) {}; //FIXME is this used?

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;
    
    advertising_config_get(&init.config);
    
    init.evt_handler = on_adv_evt;
    
    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{

    ret_code_t err_code;
 if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
    if (m_ble_connected == 0) {
      if (m_advertising.adv_evt == BLE_ADV_EVT_IDLE) {
        NRF_LOG_DEBUG("Starting Advertising");
        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
      } else {
        NRF_LOG_DEBUG("Advertising already started...");
      }
    }
    }

}
#endif

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
//  bsp_event_t startup_event;

  uint32_t err_code = bsp_init(BSP_INIT_LEDS /*| BSP_INIT_BUTTONS*/, bsp_event_handler);
//  uint32_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
  ;APP_ERROR_CHECK(err_code);

//  err_code = bsp_btn_ble_init(NULL, &startup_event);
//  APP_ERROR_CHECK(err_code);

//  *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
  *p_erase_bonds = 0;
}



/**@brief Function for initializing the Watchdog module.
 */
nrf_drv_wdt_channel_id m_wdt_channel_id;

void watchdog_event_handler(void)
{
    NRF_LOG_ERROR("!!! WDOG FIRED !!!");

    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

void watchdog_refresh(void)
{

    if (m_watchdog_running) {
      nrf_drv_wdt_channel_feed(m_wdt_channel_id);
          NRF_LOG_DEBUG("watchdog refreshed");
    }
}

static void watchdog_init(void)
{
    uint32_t err_code;
    //Configure WDT.
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, watchdog_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_wdt_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();

    m_watchdog_running = 1;
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
#if USE_POWER_MANAGEMENT==1
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run(); // Calls sd_app_evt_wait();
#endif
}


/*
 * @brief LED Control
 */

#define led_state_off 0
#define led_state_on  1

static uint8_t  m_led_state = 0;
#define LED_OVERRIDE_BIT 7U
#define LED_OVERRIDE_MASK 1U << LED_OVERRIDE_BIT

void board_led_on(uint8_t led) {
  m_led_state |= led;
  if (0 == (m_led_state & LED_OVERRIDE_MASK))
    bsp_board_led_on(led);
}

void board_led_off(uint8_t led) {
  m_led_state &= ~led;
  bsp_board_led_off(led);
}

void board_led_override(void) {
  m_led_state |= LED_OVERRIDE_MASK;
  bsp_board_led_off(LED_RED);
  bsp_board_led_off(LED_GREEN);
  bsp_board_led_off(LED_BLUE);
}

void board_led_restore(void) {
  m_led_state &= ~LED_OVERRIDE_MASK;
  if (m_led_state & LED_RED) bsp_board_led_on(LED_RED);
  if (m_led_state & LED_GREEN) bsp_board_led_on(LED_GREEN);
  if (m_led_state & LED_BLUE) bsp_board_led_on(LED_BLUE);
}


void error_led_flash_loop(void) {

  bsp_board_leds_off();

  while (true) {
    bsp_board_led_invert(LED_RED);
    nrf_delay_ms(200);
  }
}

/// ===============================================
///
/// RTC, Timers and Handlers
///
/// ===============================================

#if USE_BLE_COMMS == 0
// Do not need if BLE enabled as this is already set up (since it uses RTC1)
/// LFCLK
///
/**
 * @brief Dummy handler for LF clock init.
 *        SDK15.2+ requires handler for init (can no longer supply NULL)
 */
static void lfclk_handler(nrfx_clock_evt_type_t event_type)
{
    switch (event_type) {
    case NRFX_CLOCK_EVT_HFCLK_STARTED : NRF_LOG_DEBUG("HFCLK has been started.\n"); break;
    case NRFX_CLOCK_EVT_LFCLK_STARTED : NRF_LOG_DEBUG("LFCLK has been started.\n"); break;
    case NRFX_CLOCK_EVT_CTTO          : NRF_LOG_DEBUG("Calibration timeout.\n"); break;
    case NRFX_CLOCK_EVT_CAL_DONE      : NRF_LOG_DEBUG("Calibration has been done.\n"); break;
    default                           : NRF_LOG_DEBUG("???? (!!! Unknown Event !!!)\n");
    }

    return;
}

/**
 * @brief Function starting the internal LFCLK XTAL oscillator.
 */

static void lfclk_config(void)
{
    ret_code_t err_code = nrfx_clock_init(lfclk_handler);
    APP_ERROR_CHECK(err_code);
    if (false == nrfx_clock_lfclk_is_running())
      nrfx_clock_lfclk_start();
}

#endif
 /**
 * @brief RTC Timer Handle
 */
const nrfx_rtc_t m_rtc_2 = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrfx_rtc for RTC2. */

#if BOARD_HAS_PDM_MIC==1
void audio_datalogger_handler (nrfx_pdm_evt_t const * const p_evt);
void audio_datalogger_pdm_ctrl(uint8_t);
#endif // BOARD_HAS_PDM_MIC


/** @brief: Function for handling the RTC2 interrupts.
 * Triggered on COMPAREn and OVERFLOW .
 */
static void rtc2_handler(nrfx_rtc_int_type_t int_type)
{

    switch (int_type) {
    case NRFX_RTC_INT_OVERFLOW :
                                  add_ts64(&m_timestamp_offset, 1<<24);
                                  break;
    default                    :  NRF_LOG_DEBUG("!!! In rtc2_handler due to Unknown or Unexpected Interrupt (%d) !!!)\n",
                                                    int_type
                                                    );
                                  ASSERT(0);
    }

    return;
}



/** @brief Function initialization and configuration of RTC driver instance.
 */

static void rtc_config(void)
{
    uint32_t err_code;

    // RTC 0 (SoftDevice managed)
    // ==========================

    // RTC 1 (Used for app_timer)
    // ==========================

    // RTC 2 (Available)
    // =================

    // Use RTC2 for Timestamp.
    // All other timers use app_timer library.

    //Initialize RTC instance
    NRF_LOG_DEBUG("Starting RTC2 for timestamp");

    nrfx_rtc_config_t rtc2_config = NRF_DRV_RTC_DEFAULT_CONFIG;
    rtc2_config.prescaler         = RTC_CONFIG_PRESCALER;
    err_code = nrfx_rtc_init(&m_rtc_2, &rtc2_config, rtc2_handler);
    APP_ERROR_CHECK(err_code);

    //Disable tick event & interrupt
    nrfx_rtc_tick_disable(&m_rtc_2);

    // Enable Overflow Event (to keep track of timestamp).
    nrfx_rtc_overflow_enable(&m_rtc_2, true);

    // Enable the timer
    nrfx_rtc_enable(&m_rtc_2);
}

/**
// App Timer Handlers for watchdog
 */

APP_TIMER_DEF(m_timer_watchdog_feed);          /**< Handler to feed the watchdog. */

void watchdog_feed (void * p_context) {
//    ret_code_t err_code;

    watchdog_refresh();
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void app_timer_config(void) {
    ret_code_t err_code;

    // Watchdog feed timer
    // ==========================

    // Create the Watchdog feed timer
    err_code = app_timer_create(&m_timer_watchdog_feed,
                                APP_TIMER_MODE_REPEATED,
                                watchdog_feed);
    APP_ERROR_CHECK(err_code);

    // Start the Watchdog feed timer.
//    uint16_t adc_oversample = (int) pow(2.0, NRF_SAADC->OVERSAMPLE);
    app_timer_start(m_timer_watchdog_feed, APP_TIMER_TICKS(DEFAULT_WATCHDOG_REFRESH_PERIOD_MS), NULL);

    // Audio Timer
    // ===========

}


/**@brief Functions for getting and setting the timestamp.
 **/
/**@brief Functions for setting the timestamp.
 **/
void set_timestamp(uint32_t UnixEpochTime) {

  uint64_t timestamp_offset = 0;

  timestamp_offset = ((uint64_t) UnixEpochTime) << 15;

  // Reset the Sensor RTC Counter
  NRF_RTC2->TASKS_CLEAR = 1;

  m_old_timestamp_offset = m_timestamp_offset;

  set_ts64(&m_timestamp_offset, timestamp_offset);

}


#if USE_BLE_COMMS == 1
/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt) {

  switch (p_evt->type) {
    case BLE_NUS_EVT_RX_DATA:

          NRF_LOG_DEBUG("Received data from BLE NUS: -");
          NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

          for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
          {

              NRF_LOG_INFO("%c", p_evt->params.rx_data.p_data[i]);

          }

          uint8_t command = p_evt->params.rx_data.p_data[0];
//          uint8_t arg1    = p_evt->params.rx_data.p_data[1];

          //    let ControlCommands:[String] = ["SYS_Conf", "LED_En", "IMU_I", "ENV_I", "COL_I", "Audio"]
          //    let ControlArguments_SYS_Conf:[String] = ["Resend"]
          //    let ControlArguments_LED_En:[String] = ["Disabled", "Enabled"]
          //    let ControlArguments_IMU_I:[String] = ["Disabled", "1 ms", "2 ms", "5 ms", "10 ms", "20 ms", "50 ms", "100 ms", "200 ms", "500 ms", "1s", "2s", "5s"]

          //    let ControlArguments_Audio:[String] = ["Peak/Ave", "Stream"]
//          uint32_t imu_vals[] = {0, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000};
//          uint16_t last_value;

          switch (command) {
          case (SYS_CONFIG_INFO):
            config_datalogger_handler();
            break;

          case (SYS_LED_ENABLE): // TODO: Not really compatible with ble led indication ??
            if (p_evt->params.rx_data.p_data[1] == 0) {
              board_led_override();
            } else {
              board_led_restore();
            }
            break;

//          case (IMU_SAMPLE_INTERVAL): 
//            last_value = m_imu_sensor_sample_period_ms;
//            m_imu_sensor_sample_period_ms = imu_vals[arg1];
//            if (last_value == 0) {
//              // If this was disabled there will be no upcoming timing events for this.
//              // Create a new event. 
//              app_timer_imu_config();
//            }
//            NRF_LOG_DEBUG("Setting IMU Sensor Interval to %d ms", m_imu_sensor_sample_period_ms);
//            break;

          case(PATCHKEEPER_ECG_ID):
            {
              patchkeeper_config_ecg( p_evt->params.rx_data.p_data[1], p_evt->params.rx_data.p_data[2]);

            }
            break;


          case(PATCHKEEPER_TEMP_ID):
            {
              patchkeeper_config_temp( p_evt->params.rx_data.p_data[1], p_evt->params.rx_data.p_data[2]);

            }
            break;


          case(PATCHKEEPER_GSR_ID):
            {
              patchkeeper_config_gsr( p_evt->params.rx_data.p_data[1], p_evt->params.rx_data.p_data[2]);

            }
            break;


          case(PATCHKEEPER_STRAIN_ID):
            {
              patchkeeper_config_strain( p_evt->params.rx_data.p_data[1], p_evt->params.rx_data.p_data[2]);

            }
            break;

          case(PATCHKEEPER_PULSE_ID):
            {
              patchkeeper_config_pulse( p_evt->params.rx_data.p_data[1], p_evt->params.rx_data.p_data[2]);

            }
            break;

          case(PATCHKEEPER_IMU_ID):
            {
              patchkeeper_config_imu( p_evt->params.rx_data.p_data[1], p_evt->params.rx_data.p_data[2]);

            }
            break;


          case (PATCHKEEPER_BATT_ID):
//              m_patchkeeper_sensor_config.battery.mode 
//              m_patchkeeper_sensor_config.battery.sample_interval_ms

            break;

          case (PATCHKEEPER_PPG_ID):
              //patchkeeper_config_ppg( p_evt->params.rx_data.p_data[1], p_evt->params.rx_data.p_data[2]);

            break;

          case (SET_TIME_COMMAND):
             {
                uint8_t t0          = p_evt->params.rx_data.p_data[1];
                uint8_t t1          = p_evt->params.rx_data.p_data[2];
                uint8_t t2          = p_evt->params.rx_data.p_data[3];
                uint8_t t3          = p_evt->params.rx_data.p_data[4];
                uint32_t timestamp  = ((uint64_t) t3 << 24)
                                    + ((uint64_t) t2 << 16)
                                    + ((uint64_t) t1 <<  8)
                                    + ((uint64_t) t0 <<  0);
                set_timestamp(timestamp);

                // Send the config data packet again so post processing can record the
                // updated timestamp.
                config_datalogger_handler();
             }
//            m_audio_sample_size_ms
            break;

          default: break;
          }

          break;

    case BLE_NUS_EVT_TX_RDY:
//      NRF_LOG_DEBUG("TX Ready for data ");
      m_ble_wait_for_tx = 0;
      break;


    case BLE_NUS_EVT_COMM_STARTED:
      NRF_LOG_DEBUG("BLE_NUS_EVT_COMM_STARTED");
      m_ble_connected = 1;
      config_datalogger_handler();

      // Also send environment and battery data as it may be a while before this is scheduled.
      //batt_level_handler(NULL);


      break;
    case BLE_NUS_EVT_COMM_STOPPED:
      NRF_LOG_DEBUG("BLE_NUS_EVT_COMM_STOPPED");
      m_ble_connected = 0;
      break;
  }

  NRF_LOG_FLUSH();
}


/**@snippet [Handling the data received over BLE] */
#endif

/**@brief LED Heartbeat Timer Handle.
 *
 */
APP_TIMER_DEF(m_timer_led_heartbeat);     /**< Handler for single shot timer used to blink LED 1. */

void led_heartbeat_handler (void * p_context) {

  ret_code_t err_code;

  switch (m_led_state & LED_GREEN) {
    case (led_state_off)  : err_code = app_timer_start(m_timer_led_heartbeat, APP_TIMER_TICKS(PULSE_LED_ON_TIME_MS), NULL);
                            APP_ERROR_CHECK(err_code);
                            board_led_on(LED_GREEN);
                            break;
    case (led_state_on)   : err_code = app_timer_start(m_timer_led_heartbeat, APP_TIMER_TICKS(PULSE_LED_OFF_TIME_MS), NULL);
                            APP_ERROR_CHECK(err_code);
                            board_led_off(LED_GREEN);
                            break;
  }

}



//
// Write some Configuration Information for backend processing.
//
//

void config_datalogger_handler (void) {
  config_data_t             config_data;
  static uint8_t           packet_count = 0;
  ts64_t                    timestamp = get_timestamp();
  uint16_t                  size = sizeof(config_data_t);

//  watchdog_refresh();

#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
  if (m_ble_connected || (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES)) {
      Swap_and_Save_Buffer();
      NRF_LOG_DEBUG("Config Handler\n");
      NRF_LOG_FLUSH();
  }
#endif // PATCHKEEPER_BUFFER_SENSOR_DATA

  config_data.packet_info.sop           = SOP_BELLLABS;
  config_data.packet_info.timestamp_lo  = timestamp.lo;
  config_data.packet_info.timestamp_hi  = timestamp.hi;
  config_data.packet_info.logger_id     = LOGGER_CONFIG_ID;
  config_data.packet_info.length        = LOGGER_CONFIG_DATA_LENGTH_BYTES;
  config_data.packet_info.custom        = packet_count++;

  NRF_LOG_DEBUG(" config_data.packet_info.length : 0x%x" , config_data.packet_info.length);
  NRF_LOG_DEBUG("length of packet_info_t : %d" ,sizeof(packet_info_t));

  // Change the pointer type so it is transferred correctly.
  uint32_t *data32 = (uint32_t*) config_data.data;
  uint16_t *data16 = (uint16_t*) config_data.data;

  *data32 = NRF_RTC2->PRESCALER;
  *(data16+2) = RTC_INPUT_FREQ;
  *(data16+3) = CONFIG_DATA_VERSION;

  *(data32+2) = m_timestamp_offset.lo;
  *(data32+3) = m_timestamp_offset.hi;
  *(data32+4) = m_old_timestamp_offset.lo;
  *(data32+5) = m_old_timestamp_offset.hi;
  *(data32+6) = FIRMWARE_VERSION_ID;
  *(data32+7) = m_last_reset_reason;

#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
  ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES);
//  NRF_LOG_DEBUG("Memory Copy destination 0x%08x to 0x%08x (config)\n", m_p_datalog_mem+m_write_offset, m_p_datalog_mem+m_write_offset + size);
  memcpy((m_p_datalog_mem+m_write_offset), &config_data,  size);
  m_write_offset += size;
#endif  //PATCHKEEPER_BUFFER_SENSOR_DATA

#if BLE_STREAM_IMMEDIATE==1
  if (m_ble_connected == 1) {
    ble_send_data((unsigned char*) &config_data, (unsigned char*) &config_data+size);
  }
#endif   //BLE_STREAM_IMMEDIATE
}

 
#if USE_BLE_COMMS == 1
static uint32_t ble_send_data(datalog_memory_t *start_ptr, datalog_memory_t *end_ptr) {

  static uint16_t packets_sent = 0;
  ret_code_t err_code;

#if NBL_LOG_BLE_SEND == 1
  NRF_LOG_INFO("======================");
  NRF_LOG_INFO("   ble_send_data()");
  NRF_LOG_INFO("======================");
#endif


#if NBL_LOG_BLE_SEND == 1
  NRF_LOG_INFO("start_ptr = 0x%08x", (int) start_ptr);
  NRF_LOG_INFO("end_ptr = 0x%08x", (int) end_ptr);
  NRF_LOG_INFO("Total bytes to send = %d", (int) (end_ptr - start_ptr));
  NRF_LOG_INFO("Total packets to send (approx) = %d", (int) (end_ptr - start_ptr)/m_ble_nus_max_data_len);
  NRF_LOG_FLUSH();
#endif

  uint8_t done = false;
  uint16_t total_bytes_sent = 0;

  // Send as packets of m_ble_nus_max_data_len

  while (!done) {

    uint16_t bytes_remaining = (int) (end_ptr - start_ptr);

    uint16_t packet_length = m_ble_nus_max_data_len;

    if (bytes_remaining < packet_length) {
      packet_length = bytes_remaining;
    }

    err_code = NRF_ERROR_IO_PENDING; // Anything non-zero.
    uint16_t attempts_this_packet = 0;

    while (err_code != NRF_SUCCESS) {

      if (0 == m_ble_connected) {
         break;
      }

      err_code = ble_nus_data_send(&m_nus, start_ptr, &packet_length, m_conn_handle);
      if (err_code == NRF_ERROR_INVALID_STATE) {
//        NRF_LOG_ERROR("BLE : NRF_ERROR_INVALID_STATE.  (Attempt %d; Packet %d)", attempts_this_packet++, packets_sent);
//        NRF_LOG_FLUSH();
      } else if (err_code == NRF_ERROR_BUSY) {
        NRF_LOG_ERROR("BLE : NRF_ERROR_BUSY. (Attempt %d; Packet %d)", attempts_this_packet++, packets_sent);
        NRF_LOG_FLUSH();
      } else if (err_code == NRF_ERROR_NOT_FOUND) {
        NRF_LOG_ERROR("BLE : NRF_ERROR_NOT_FOUND. (Attempt %d; Packet %d)", attempts_this_packet++, packets_sent);
        NRF_LOG_FLUSH();
      } else if (err_code == NRF_ERROR_RESOURCES) {
#if NBL_LOG_BLE_SEND == 1
        NRF_LOG_DEBUG("BLE : Not Enough Resource. Wait for TX Event... (Attempt %d; Packet %d)", attempts_this_packet++, packets_sent);
        NRF_LOG_FLUSH();
#endif
        m_ble_wait_for_tx = 1;
        // Sample RTC to reset if we spend too long waiting for this to complete.
        // Perhaps the client has gone out of range.
        uint32_t wait_for_tx_start_time = NRF_RTC2->COUNTER;
        uint32_t wait_for_tx_timeout_time = wait_for_tx_start_time + NBL_NRFX_RTC_US_TO_TICKS(5000000, RTC_INPUT_FREQ);

        uint32_t delay_loops = 0;
        while (m_ble_wait_for_tx) {
            // Signal we are waiting for TX through the LED.
            // Blue should already be on...

            if (0 == m_ble_connected) {
              board_led_off(LED_GREEN);
              break;
            }

            if (!(delay_loops++%1000)) {
#if NBL_LOG_BLE_SEND == 1
              NRF_LOG_INFO("Wait for TX");
#endif
            }

            board_led_on(LED_GREEN);

//          __WFE();
            // TODO - Can we use a timer to do this?
            nrf_delay_ms(1);
            if (NRF_RTC2->COUNTER > (wait_for_tx_timeout_time & 0x00ffffff)) {
              // TODO - Force a reset. It would be better if we know what sort of
              // reset happened.
              // Option 1 - keep in this loop. This (as it is called from the sensor handlers) will
              // starve all the subsequent sensor handlers and thus starve the watchdog resulting in a
              // WDOG reset.
              // In future, do this more cleanly.
#if NBL_LOG_BLE_SEND == 1
              NRF_LOG_DEBUG("Enter loop to starve WDOG");
#endif
              board_led_on(LED_RED);
              nrf_delay_ms(1000);
              while(0) {};
            }
            board_led_off(LED_GREEN);
        }

#if NBL_LOG_BLE_SEND == 1
        if (delay_loops) {
          NRF_LOG_DEBUG("TX Done");
        }
        NRF_LOG_DEBUG("BLE : TX Event Received...");
        NRF_LOG_FLUSH();
#endif
      } else if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Unexpected Return Code %x. Re-trying. (Attempt %d; Packet %d)", err_code, attempts_this_packet++, packets_sent);
       NRF_LOG_FLUSH();
       __BKPT();
      }
    }

#if NBL_LOG_BLE_SEND == 1
    NRF_LOG_DEBUG("Packet %d complete after %d retries", packets_sent, attempts_this_packet);
#endif
    start_ptr += packet_length;
    total_bytes_sent += packet_length;

    if (start_ptr >= end_ptr) {
      done = true;
    }
  }  //  while (!done)

  packets_sent++;

//  board_led_off(LED_BLUE);

#if NBL_LOG_BLE_SEND == 1
    NRF_LOG_DEBUG("main: BLE_SEND Completed");
#endif
  return NRF_SUCCESS;
}
#endif  //USE_BLE_COMMS


void resetreas2string(uint32_t resetreas, char* reason_string) {
  char* strings [8] = {"PIN","WDOG","SREQ","LOCKUP","OFF","LPCOMP","DIF","NFC"};
  uint32_t compressed_reason;

  *reason_string = (char) '\0';


  if (resetreas == 0) {
    strcat(reason_string, "POR");
  } else {

    compressed_reason = resetreas & 0xf; // DCBA (bits 3:0);
    compressed_reason |= (resetreas & (0xf<<16)) >> 12; // HGFE (bits 19:16);

    for (int8_t i=0; i<8; i++) {
      if (compressed_reason & (1 << i)) {
        if (strlen(reason_string)) {
          strcat(reason_string, " | ");
        }
        strcat(reason_string, strings[i]);
      }
    }
  }

}



/**@brief Application main function.
 */
int main(void)
{
  bool erase_bonds;
  ret_code_t err_code;

#if DEBUG_WITH_SYSTEMVIEW == 1
  SEGGER_SYSVIEW_Conf();            /* Configure and initialize SystemView  */
  SEGGER_SYSVIEW_Start();           /* Starts SystemView recording*/
  SEGGER_SYSVIEW_OnIdle();          /* Tells SystemView that System is currently in "Idle"*/
#endif

  // Read and store the reset reason
  m_last_reset_reason = NRF_POWER->RESETREAS;

  // Reset the reset reason (could also write 0xf)
  NRF_POWER->RESETREAS = m_last_reset_reason;

  // Clear the timestamp.
  set_ts64(&m_timestamp_offset, 0);
  set_ts64(&m_old_timestamp_offset, 0);

  // Indicate that the watchdog is not yet running
  m_watchdog_running = 0;

#if USE_BLE_COMMS == 0
  // If BLE_COMMS not used then we need to start the LF Clock.
  lfclk_config();
#endif

  // Initialize.
  log_init();

#if NBL_LOG_BOOT == 1
  char reason_string [55];
  resetreas2string(m_last_reset_reason, reason_string);
  NRF_LOG_DEBUG("");
  NRF_LOG_DEBUG("");
  NRF_LOG_DEBUG("*****************************************");
  NRF_LOG_DEBUG("Booting.");
  NRF_LOG_DEBUG("--> Last reset was %s", reason_string);
  NRF_LOG_DEBUG("*****************************************");
  NRF_LOG_FLUSH();
#endif

#if USE_BLE_COMMS == 1 
#if BLE_DFU_ENABLED==1
  // Initialize the async SVCI interface to bootloader before any interrupts are enabled.
  err_code = ble_dfu_buttonless_async_svci_init();
  NRF_LOG_DEBUG("ble_dfu_buttonless_async_svci_init error code: %d", err_code);
  APP_ERROR_CHECK(err_code);
#endif
#endif


  app_timers_init(); 
  power_management_init();
  buttons_leds_init(&erase_bonds);


#if BLE_DFU_ENABLED==1
  NRF_LOG_DEBUG("Secure DFU Enabled!!!");
  NRF_LOG_FLUSH();
#else
  NRF_LOG_DEBUG("Secure NOT Enabled!!!");
  NRF_LOG_FLUSH();
#endif

#if USE_BLE_COMMS == 1
  NRF_LOG_DEBUG("ble_stack_init...");
  NRF_LOG_FLUSH();
  ble_stack_init();
  peer_manager_init();
  gap_params_init();
  gatt_init();
  services_init();
  advertising_init();
  conn_params_init();

#if BLE_DFU_ENABLED==1
  NRF_LOG_DEBUG("BLE Comms Setup Complete...");
  NRF_LOG_FLUSH();
#endif

  patchkeeper_init_context_t init_context;
  init_context.ble_send_data = &ble_send_data;
  init_context.ble_advertising_start = &advertising_start;
#endif

#if PATCHKEEPER_BUFFER_SENSOR_DATA==1
  // Point to first buffer for datalog memory and reset write offset
    m_p_datalog_mem = (datalog_memory_t*) datalog_memory_0;
    m_write_offset  = 0;
//  NRF_LOG_DEBUG("m_p_datalog_mem: 0x%x",m_p_datalog_mem);
#endif // PATCHKEEPER_BUFFER_SENSOR_DATA

  // RTC2 (timestamp) configuration
  rtc_config();
  NRF_LOG_DEBUG(" RTC2 started");

  m_ble_connected = 0;

  // Make sure the LEDs are off.
  board_led_off(LED_BLUE);
  board_led_off(LED_RED);
  board_led_off(LED_GREEN);

  // app_timer configuration
  app_timer_config();
  NRF_LOG_DEBUG("app timer configured.");
  // Start the Watchdog Timer
  watchdog_init();
  NRF_LOG_DEBUG("Watchdog inited.");

#if USE_BLE_COMMS == 1
  // Start Advertising.
  NRF_LOG_INFO("Starting BLE..."); 
  advertising_start(erase_bonds);
#endif

  patchkeeper_init(init_context);

//  // TODO : REMOVE!!!
//  uint8_t test_mask = PATCHKEEPER_ECG_TEST_MASK_BIT || PATCHKEEPER_GSR_TEST_MASK_BIT;
//  patchkeeper_trigger(test_mask);

  // Enter main loop.
  for (;;)
  {
      idle_state_handle(); // Calls sd_app_evt_wait();
  }
}


/**
 * @}
 */
