/*
 * © 2022 Nokia
 * Licensed under the BSD 3-Clause License
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */


#include "evt_trace.h"
#include "sdk_config.h"
//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"
#include "SEGGER_RTT.h"
#include "string.h"


#if NBL_LOG_BLE_EMCD_NUS==1
char* ble_evt_id_to_name (unsigned char evt_id) {
  char* retchar;
  switch (evt_id) {
  case (0x00)   :  retchar = "BLE_EVT_INVALID"; break;
  case (0x01)   :  retchar = "BLE_EVT_USER_MEM_REQUEST"; break;
  case (0x02)   :  retchar = "BLE_EVT_USER_MEM_RELEASE"; break;
 
  case (0x10)   :  retchar = "BLE_GAP_EVT_CONNECTED"; break;
  case (0x11)   :  retchar = "BLE_GAP_EVT_DISCONNECTED"; break;
  case (0x12)   :  retchar = "BLE_GAP_EVT_CONN_PARAM_UPDATE"; break;
  case (0x13)   :  retchar = "BLE_GAP_EVT_SEC_PARAMS_REQUEST"; break;        
  case (0x14)   :  retchar = "BLE_GAP_EVT_SEC_INFO_REQUEST"; break;          
  case (0x15)   :  retchar = "BLE_GAP_EVT_PASSKEY_DISPLAY"; break;           
  case (0x16)   :  retchar = "BLE_GAP_EVT_KEY_PRESSED"; break;               
  case (0x17)   :  retchar = "BLE_GAP_EVT_AUTH_KEY_REQUEST"; break;          
  case (0x18)   :  retchar = "BLE_GAP_EVT_LESC_DHKEY_REQUEST"; break;        
  case (0x19)   :  retchar = "BLE_GAP_EVT_AUTH_STATUS"; break;               
  case (0x1a)   :  retchar = "BLE_GAP_EVT_CONN_SEC_UPDATE"; break;           
  case (0x1b)   :  retchar = "BLE_GAP_EVT_TIMEOUT"; break;                   
  case (0x1c)   :  retchar = "BLE_GAP_EVT_RSSI_CHANGED"; break;              
  case (0x1d)   :  retchar = "BLE_GAP_EVT_ADV_REPORT"; break;                
  case (0x1e)   :  retchar = "BLE_GAP_EVT_SEC_REQUEST"; break;               
  case (0x1f)   :  retchar = "BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST"; break; 
  case (0x20)   :  retchar = "BLE_GAP_EVT_SCAN_REQ_REPORT"; break;           
  case (0x21)   :  retchar = "BLE_GAP_EVT_PHY_UPDATE_REQUEST"; break;        
  case (0x22)   :  retchar = "BLE_GAP_EVT_PHY_UPDATE"; break;                
  case (0x23)   :  retchar = "BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST"; break;
  case (0x24)   :  retchar = "BLE_GAP_EVT_DATA_LENGTH_UPDATE"; break;        
  case (0x25)   :  retchar = "BLE_GAP_EVT_QOS_CHANNEL_SURVEY_REPORT"; break; 
  case (0x26)   :  retchar = "BLE_GAP_EVT_ADV_SET_TERMINATED"; break;        
 
  case (0x30)   :  retchar = "BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP"; break;
  case (0x31)   :  retchar = "BLE_GATTC_EVT_REL_DISC_RSP"; break;
  case (0x32)   :  retchar = "BLE_GATTC_EVT_CHAR_DISC_RSP"; break;
  case (0x33)   :  retchar = "BLE_GATTC_EVT_DESC_DISC_RSP"; break;
  case (0x34)   :  retchar = "BLE_GATTC_EVT_ATTR_INFO_DISC_RSP"; break;
  case (0x35)   :  retchar = "BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP"; break;
  case (0x36)   :  retchar = "BLE_GATTC_EVT_READ_RSP"; break;
  case (0x37)   :  retchar = "BLE_GATTC_EVT_CHAR_VALS_READ_RSP"; break;
  case (0x38)   :  retchar = "BLE_GATTC_EVT_WRITE_RSP"; break;
  case (0x39)   :  retchar = "BLE_GATTC_EVT_HVX"; break;
  case (0x3a)   :  retchar = "BLE_GATTC_EVT_EXCHANGE_MTU_RSP"; break;
  case (0x3b)   :  retchar = "BLE_GATTC_EVT_TIMEOUT"; break;
  case (0x3c)   :  retchar = "BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE"; break;
 
  case (0x50)   :  retchar = "BLE_GATTS_EVT_WRITE"; break;
  case (0x51)   :  retchar = "BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST"; break;
  case (0x52)   :  retchar = "BLE_GATTS_EVT_SYS_ATTR_MISSING"; break;
  case (0x53)   :  retchar = "BLE_GATTS_EVT_HVC"; break;
  case (0x54)   :  retchar = "BLE_GATTS_EVT_SC_CONFIRM"; break;
  case (0x55)   :  retchar = "BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST"; break;
  case (0x56)   :  retchar = "BLE_GATTS_EVT_TIMEOUT"; break;
  case (0x57)   :  retchar = "BLE_GATTS_EVT_HVN_TX_COMPLETE"; break;
 
  case (0x70)   :  retchar = "BLE_L2CAP_EVT_CH_SETUP_REQUEST"; break;
  case (0x71)   :  retchar = "BLE_L2CAP_EVT_CH_SETUP_REFUSED"; break;   
  case (0x72)   :  retchar = "BLE_L2CAP_EVT_CH_SETUP"; break;           
  case (0x73)   :  retchar = "BLE_L2CAP_EVT_CH_RELEASED"; break;
  case (0x74)   :  retchar = "BLE_L2CAP_EVT_CH_SDU_BUF_RELEASED"; break;
  case (0x75)   :  retchar = "BLE_L2CAP_EVT_CH_CREDIT"; break;
  case (0x76)   :  retchar = "BLE_L2CAP_EVT_CH_RX"; break;              
  case (0x77)   :  retchar = "BLE_L2CAP_EVT_CH_TX"; break;
  default       :  retchar = "*** UNKNOWN BLE Evt Type ***"; break;
  }

  return retchar;

}
#endif

void ble_evt_trace (char* func_name, unsigned char evt_id) {
#if BLE_EVT_TRACE_ON==1
   char print_string [51];
   memset(print_string, ' ',50);
   strcpy(print_string, func_name);
   print_string[strlen(func_name)] = ' ';
   print_string[50] = 0;
   SEGGER_RTT_printf(0, "%-30s : Event ID 0x%x (%s)\n", print_string, evt_id, ble_evt_id_to_name(evt_id) );
#endif
  return;
}

