/*
 * © 2022 Nokia
 * Licensed under the BSD 3-Clause License
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */


#include "patchkeeper_saadc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

//static nrf_saadc_value_t m_vout_buffer[PATCHKEEPER_ADC_CHANNELS_USED];


#define PATCHKEEPER_ADC_REFERENCE_VOLTAGE       (0.6f)                              // The standard internal ADC reference voltage.
#define PATCHKEEPER_ADC_RESOLUTION_BITS         (8 + (NRF_SAADC_RESOLUTION_12BIT * 2)) //ADC resolution [bits].

adc_callback_fn_t m_main_patchkeeper_adc_callback = NULL;
uint8_t m_calibrating;
uint8_t m_adc_done_flag;
uint8_t m_adc_gains [PATCHKEEPER_ADC_CHANNELS_USED];



void patchkeeper_saadc_init(nrf_saadc_value_t * vout_buffer)
{
    ret_code_t err_code;

    nrfx_saadc_config_t   patchkeeper_saadc_config = NRFX_SAADC_PATCHKEEPER_DEFAULT_CONFIG;

    // Create the Channel Configs for each channel used.
    nrf_saadc_channel_config_t patchkeeper_gsr_channel_config     = NRFX_SAADC_PATCHKEEPER_CHANNEL_CONFIG_SE(NRF_SAADC_GSR_OUT);
    nrf_saadc_channel_config_t patchkeeper_pulse_channel_config   = NRFX_SAADC_PATCHKEEPER_CHANNEL_CONFIG_SE(NRF_SAADC_PULSE_OUT);
    nrf_saadc_channel_config_t patchkeeper_strain_channel_config  = NRFX_SAADC_PATCHKEEPER_CHANNEL_CONFIG_SE(NRF_SAADC_STRAIN_OUT);
    nrf_saadc_channel_config_t patchkeeper_temp_channel_config    = NRFX_SAADC_PATCHKEEPER_CHANNEL_CONFIG_SE(NRF_SAADC_TEMP_OUT);
    nrf_saadc_channel_config_t patchkeeper_batt_channel_config    = NRFX_SAADC_PATCHKEEPER_CHANNEL_CONFIG_SE(NRF_SAADC_BATT_OUT);

    // Modify the gains from the default.
    patchkeeper_gsr_channel_config.gain     = PATCHKEEPER_GSR_ADC_GAIN;
    patchkeeper_strain_channel_config.gain  = PATCHKEEPER_STRAIN_ADC_GAIN;
    patchkeeper_pulse_channel_config.gain   = PATCHKEEPER_PULSE_ADC_GAIN;
    patchkeeper_temp_channel_config.gain    = PATCHKEEPER_TEMP_ADC_GAIN;
    patchkeeper_batt_channel_config.gain    = PATCHKEEPER_BATT_ADC_GAIN;

    // Initialise the SAADC module
    err_code = nrfx_saadc_init(&patchkeeper_saadc_config, patchkeeper_saadc_callback);
    APP_ERROR_CHECK(err_code);

    // Calibrate the ADC
    m_calibrating = 1;
    err_code = nrfx_saadc_calibrate_offset();
    APP_ERROR_CHECK(err_code);
    
    while (m_calibrating) {};

    // Setup the Channels
    err_code = nrfx_saadc_channel_init(0, &patchkeeper_gsr_channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_channel_init(1, &patchkeeper_pulse_channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_channel_init(2, &patchkeeper_strain_channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_channel_init(3, &patchkeeper_temp_channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_channel_init(4, &patchkeeper_batt_channel_config);
    APP_ERROR_CHECK(err_code);

// Setup the buffer for ADC convert. Conversion will be triggered by sample command: nrfx_saadc_sample().
    err_code = nrfx_saadc_buffer_convert(vout_buffer, PATCHKEEPER_ADC_CHANNELS_USED);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEBUG("SAADC Init Done");
}


//callback here are to re-setup the same buffer (vout_buffer as used in patchkeeper_saadc_init)
//for next sampling event, with p_event->data.done.p_buffer
void patchkeeper_saadc_callback(nrfx_saadc_evt_t const * p_event)
{

    if (p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE) {
      m_calibrating = 0;
//      NRF_LOG_DEBUG("SAADC Calibration Done");
    }

    if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, PATCHKEEPER_ADC_CHANNELS_USED);

        APP_ERROR_CHECK(err_code);

        m_adc_done_flag = 1;
//        NRF_LOG_DEBUG("SAADC Event Done");

    }

//    if (p_event->type == NRFX_SAADC_EVT_LIMIT) {
//      NRF_LOG_DEBUG("SAADC Event Limit");
//    }

}

float patchkeeper_get_voltage_from_adc_value (nrf_saadc_value_t adc_value, uint8_t adc_saadc_gain) {

  // Adjust for gain and reference
  float adc_gain = 0.0f;

  switch (adc_saadc_gain) {
    case NRF_SAADC_GAIN1_6 :  adc_gain = 1 / (float) 6.0f; break;   ///< Gain factor 1/6.
    case NRF_SAADC_GAIN1_5 :  adc_gain = 1 / (float) 5.0f; break;   ///< Gain factor 1/5.
    case NRF_SAADC_GAIN1_4 :  adc_gain = 1 / (float) 4.0f; break;   ///< Gain factor 1/4.
    case NRF_SAADC_GAIN1_3 :  adc_gain = 1 / (float) 3.0f; break;   ///< Gain factor 1/3.
    case NRF_SAADC_GAIN1_2 :  adc_gain = 1 / (float) 2.0f; break;   ///< Gain factor 1/2.
    case NRF_SAADC_GAIN1   :  adc_gain = 1; break;         ///< Gain factor 1.
    case NRF_SAADC_GAIN2   :  adc_gain = 2; break;         ///< Gain factor 2.
    case NRF_SAADC_GAIN4   :  adc_gain = 4; break;         ///< Gain factor 4.
    default                :  APP_ERROR_CHECK(0);
  }

  float voltage = adc_value / ( (adc_gain / PATCHKEEPER_ADC_REFERENCE_VOLTAGE) * pow(2, PATCHKEEPER_ADC_RESOLUTION_BITS));

  return voltage;
}

void patchkeeper_saadc_read_adc(void* p_dummy) {

  // TODO - Use PPI to take this away from MCU
  //        But this will require a dedicated timer.

  ret_code_t err_code;

//  NRF_LOG_DEBUG("Trigger ADC Read");

  m_adc_done_flag = 0;

  err_code = nrfx_saadc_sample();
  APP_ERROR_CHECK(err_code);

  while(m_adc_done_flag==0) {};
//  NRF_LOG_DEBUG("ADC Value Available");
}
