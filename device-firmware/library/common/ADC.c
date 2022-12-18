/*
 * © 2022 Nokia
 * Licensed under the BSD 3-Clause License
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */


#include "ADC.h"
#include "math.h"

#define ADC_SAMPLES_IN_BUFFER   1
static nrf_saadc_value_t m_buffer[ADC_SAMPLES_IN_BUFFER];
float VBATT                     = 0.0;

//#define ADC_SAADC_GAIN NRF_SAADC_GAIN1                                  // ADC gain.
#define ADC_REFERENCE_VOLTAGE       (0.6f)                              // The standard internal ADC reference voltage.
#define ADC_RESOLUTION_BITS         (8 + (NRF_SAADC_RESOLUTION_12BIT * 2)) //ADC resolution [bits].

adc_callback_fn_t m_main_adc_callback;

//void ADC_init(void (*m_main_adc_callback(void) ))
void ADC_init(void)
{
    ret_code_t err_code;
    
    nrfx_saadc_config_t   batt_config = NRFX_SAADC_DEFAULT_CONFIG;

    nrf_saadc_channel_config_t channel_config =
    NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);

//    channel_config.gain       = ADC_SAADC_GAIN;
#if ORIGINAL_DESIGN_ADC_RES_VALUES==1
  channel_config.gain       = NRF_SAADC_GAIN1;
#else
  channel_config.gain       = NRF_SAADC_GAIN1_6;

  if ((NRF_UICR->CUSTOMER[0] & 0x1) == 0) {
    channel_config.gain       = NRF_SAADC_GAIN1;
  }
#endif

    // Channel Burst Mode
    // Burst Enabled with oversampling  - the ADCs sample the relevant number of times
    //                                    as quickly as possible and return the average.
    //                                    Data returned every call to sample.
    // 
    // Burst Disabled with oversampling - the ADCs sample once each sample call. Once 
    //                                    the correct numbers of samples have been collected, 
    //                                    returns the average.
    //                                    Data returned every OVERSAMPLES calls to sample.
    //
    // For the battery monitor it is actually fluctuating (dependant on current drain) so we 
    // need to average over a longer time i.e. disable BURST mode.
//    channel_config.burst      = NRF_SAADC_BURST_ENABLED;
    channel_config.burst      = NRF_SAADC_BURST_DISABLED;
    channel_config.acq_time   = NRF_SAADC_ACQTIME_40US;

    err_code = nrfx_saadc_init(&batt_config, ADC_callback);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrfx_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrfx_saadc_buffer_convert(m_buffer, ADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}


void ADC_init_with_callback(adc_callback_fn_t callback_fn){
  m_main_adc_callback = callback_fn;
  ADC_init();
}


float get_voltage_from_adc_value (uint32_t adc_value) {

  // Adjust for gain and reference
  float adc_gain = 0.0f;
#if ORIGINAL_DESIGN_ADC_RES_VALUES==1
  float r25 = 1500.0, r26=180.0;  //original design value, too large
  uint8_t adc_saadc_gain = NRF_SAADC_GAIN1;
#else
  float r25 = 10, r26= 21.5; //datalogger resistor pair updated
  uint8_t adc_saadc_gain = NRF_SAADC_GAIN1_6;

  if ((NRF_UICR->CUSTOMER[0] & 0x1) == 0) {
    r25 = 1500.0, r26=180.0;  //original design value, too large but board has not been updated
    adc_saadc_gain = NRF_SAADC_GAIN1;
  }
#endif

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


  float resistor_divider = r26 / (r25 + r26);

  float volt_tmp1 = adc_value / ( (adc_gain / ADC_REFERENCE_VOLTAGE) * pow(2, ADC_RESOLUTION_BITS));
  float voltage   = volt_tmp1 / resistor_divider;

  return voltage;
}



void ADC_callback(nrfx_saadc_evt_t const * p_event)
{

    if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
        
        err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, ADC_SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        VBATT = get_voltage_from_adc_value((float)p_event->data.done.p_buffer[0]);

        m_main_adc_callback(VBATT);

    }
}

