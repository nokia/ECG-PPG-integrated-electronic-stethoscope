/*
 * © 2022 Nokia
 * Licensed under the BSD 3-Clause License
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef TIMESTAMP_H_
#define TIMESTAMP_H_

#include "nrf.h"


// 24 bits from RTC + 32 bits here makes 56 bits which @32768Hz gives 3,268 years
// NRF_RTC2->COUNTER bits 14->0 are sub-second times (each count is 1/32768 seconds)
// NRF_RTC2->COUNTER bits 23->15 are Unix Epoch Time (if synchronised) 
// TODO: INVESTIGATE >>>   For some reason if I declare the following: - 
// static uint64_t m_timestamp_offset; 
// Then the IMU interrupt is never detected!!!
// Have to declare it as 2x32 bit instead
typedef struct {
  uint32_t lo;
  uint32_t hi;
} ts64_t;


extern ts64_t m_timestamp_offset;
extern ts64_t m_old_timestamp_offset;

void set_ts64 (ts64_t *p_ts64, uint64_t ts);
uint64_t ts64_to_uint64(ts64_t *p_ts64);
void add_ts64 (ts64_t *p_ts64,  uint64_t ts);
ts64_t get_timestamp(void);
uint64_t get_timestamp_us(void);

#endif // TIMESTAMP_H_