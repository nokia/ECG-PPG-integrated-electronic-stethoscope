/*
 * © 2022 Nokia
 * Licensed under the BSD 3-Clause License
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */



#include "timestamp.h"

void set_ts64 (ts64_t *p_ts64, uint64_t ts) {
  p_ts64->lo = ts & 0xffffffff;
  p_ts64->hi = (ts >> 32) & 0xffffffff;
}

uint64_t ts64_to_uint64(ts64_t *p_ts64){
  uint64_t uint64 = ((uint64_t) p_ts64->hi << 32) + p_ts64->lo;
  return uint64;
}

void add_ts64 (ts64_t *p_ts64,  uint64_t ts) {
  uint64_t uint64 = ts64_to_uint64(p_ts64);
  uint64 += ts;
  set_ts64(p_ts64, uint64);
}

ts64_t get_timestamp(void) { 
  ts64_t timestamp = m_timestamp_offset;
  add_ts64(&timestamp, (uint64_t) NRF_RTC2->COUNTER);
  return timestamp; 
} 

uint64_t get_timestamp_us(void) { 

  ts64_t timestamp_t64 = get_timestamp();

  uint64_t timestamp = ts64_to_uint64(&timestamp_t64);

  // Timestamp is in units of 1/32768 second.
  // We need to conver this to us.
  timestamp *= 1000000/32768;
  
  return timestamp; 
} 