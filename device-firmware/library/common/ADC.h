/*
 * © 2022 Nokia
 * Licensed under the BSD 3-Clause License
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */


#ifndef MA_ADC_H__
#define MA_ADC_H__

#define BATTERY_AVERAGE_PERIOD 5

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef void (*adc_callback_fn_t)(float vbatt);

    #include "nrfx_saadc.h"
    #include "boards.h"
//    void ADC_init(void);
    void ADC_init_with_callback(adc_callback_fn_t);
    void ADC_callback(nrfx_saadc_evt_t const * p_event);
    
#ifdef __cplusplus
}
#endif

#endif
