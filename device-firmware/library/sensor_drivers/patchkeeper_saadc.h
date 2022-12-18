/*
 * © 2022 Nokia
 * Licensed under the BSD 3-Clause License
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef PATCHKEEPER_SAADC_H_
#define PATCHKEEPER_SAADC_H_

#include "nordic_common.h" 
#include "nrf.h"
#include <string.h>
#include "ADC.h"
#include "math.h"
#include "nrfx.h" 
#include "nrfx_saadc.h"
#include "app_timer.h"

#define PATCHKEEPER_ADC_CHANNELS_USED        5
#define PATCHKEEPER_GSR_ADC_GAIN             NRF_SAADC_GAIN1_4
#define PATCHKEEPER_STRAIN_ADC_GAIN          NRF_SAADC_GAIN1_5
#define PATCHKEEPER_PULSE_ADC_GAIN           NRF_SAADC_GAIN1_5
#define PATCHKEEPER_TEMP_ADC_GAIN            NRF_SAADC_GAIN1_5
#define PATCHKEEPER_BATT_ADC_GAIN            NRF_SAADC_GAIN1_6

#define PATCHKEEPER_ADC_REFERENCE_VOLTAGE    (0.6f)                              // The standard internal ADC reference voltage.
#define PATCHKEEPER_ADC_RESOLUTION_BITS      (8 + (NRF_SAADC_RESOLUTION_12BIT * 2)) //ADC resolution [bits].

typedef struct {
    nrf_saadc_value_t gsr;
    nrf_saadc_value_t pulse;
    nrf_saadc_value_t strain;
    nrf_saadc_value_t temp;
    nrf_saadc_value_t batt;
} patchkeeper_adc_data_t;

void patchkeeper_saadc_init(nrf_saadc_value_t *);
void patchkeeper_saadc_callback(nrfx_saadc_evt_t const * p_event);
float patchkeeper_get_voltage_from_adc_value (nrf_saadc_value_t, uint8_t);
void patchkeeper_saadc_read_adc(void*);

/**
 * @brief Macro for setting @ref nrfx_saadc_config_t to default settings.
 */
#define NRFX_SAADC_PATCHKEEPER_DEFAULT_CONFIG                                               \
{                                                                               \
    .resolution         = (nrf_saadc_resolution_t)NRFX_SAADC_CONFIG_RESOLUTION, \
    .oversample         = (nrf_saadc_oversample_t)NRFX_SAADC_CONFIG_OVERSAMPLE, \
    .interrupt_priority = NRFX_SAADC_CONFIG_IRQ_PRIORITY,                       \
    .low_power_mode     = NRFX_SAADC_CONFIG_LP_MODE                             \
}
/**
 * @brief Macro for setting @ref nrf_saadc_channel_config_t to default settings
 *        in single ended mode.
 *
 * @param PIN_P Analog input.
 */
#define NRFX_SAADC_PATCHKEEPER_CHANNEL_CONFIG_SE(PIN_P) \
{                                                   \
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,      \
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,      \
    .gain       = NRF_SAADC_GAIN1_6,                \
    .reference  = NRF_SAADC_REFERENCE_INTERNAL,     \
    .acq_time   = NRF_SAADC_ACQTIME_10US,           \
    .mode       = NRF_SAADC_MODE_SINGLE_ENDED,      \
    .burst      = NRF_SAADC_BURST_DISABLED,         \
    .pin_p      = (nrf_saadc_input_t)(PIN_P),       \
    .pin_n      = NRF_SAADC_INPUT_DISABLED          \
}

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


#endif // PATCHKEEPER_SAADC_H_