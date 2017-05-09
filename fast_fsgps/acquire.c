/****************************************************************************
* acquire.c - Attempt to acquire Space Vehicles
*
* Author: Mike Field <hamster@snap.net.nz>
*
*****************************************************************************
MIT License

Copyright (c) 2017 Mike Field

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
****************************************************************************/
#include <stdio.h>
#include "types.h"
#include "gold_codes.h"
#include "acquire.h"

#define N_BANDS  31
#define N_PARALLEL 2

uint_32 ncos_phase[N_BANDS];
uint_32 ncos_step[N_BANDS];

struct Acq_state {
  uint_32 *sv_gold_codes;
  uint_32 code_phase;
  uint_32 ones_s[N_BANDS];
  uint_32 ones_c[N_BANDS];
  uint_32 tries;
  uint_32 max_offset;
  uint_32 max_power;
  uint_32 max_step;
  uint_32 current_sv;
  void (*finished_cb)(int sv, uint_32 power);
  void (*power_cb)(int sv, uint_32 freq, uint_32 offset, uint_32 power);
};

static struct Acq_state states[N_PARALLEL];

static unsigned char ones_lookup[256];
static void setup_count_ones(void) {
  int i;
  for(i = 0; i < 256; i++) {
    int j;
    for(j = 0; j < 8; j++) {
      if(i&(1<<j))
        ones_lookup[i]++;
    }
  }
}


static int count_ones(uint_32 a) {
  int rtn;

  rtn  = ones_lookup[ a      & 0xFF];
  rtn += ones_lookup[(a>> 8) & 0xFF];
  rtn += ones_lookup[(a>>16) & 0xFF];
  rtn += ones_lookup[(a>>24) & 0xFF];
  return rtn;
}

void acquire_startup(void) {
  int if_band = 525*5000/(N_BANDS/2), i;

  for(i = 0; i < N_BANDS; i++) {
    ncos_step[i] = 0x40000000 + (i-N_BANDS/2)*if_band;
  }
  setup_count_ones();
}

int acquire_start(int sv_id, void (*callback_power)(int sv, uint_32 freq, uint_32 offset, uint_32 power), void (*callback_finished)(int sv, uint_32 power)) {
  int i,s;
  for(s = 0; s < N_PARALLEL; s++) {
    if(states[s].sv_gold_codes == NULL)
      break;
  }
  if(s == N_PARALLEL)
    return 0;

  states[s].power_cb    = callback_power;
  states[s].finished_cb = callback_finished;
  states[s].current_sv = sv_id;
  states[s].sv_gold_codes = &(gold_codes_32_cycles[sv_id][0]);
  for(i = 0; i < N_BANDS; i++) {
    states[s].ones_s[i] = 0;
    states[s].ones_c[i] = 0;
  }
  states[s].tries = 0;
  states[s].max_power = 0;
  return 1;
}
/************************************************
*
************************************************/
static void fast_IF_nco_mask_8(uint_32 nco, uint_32 step, uint_32 *s, uint_32 *c
) {
  int j;
  uint_32 nco_in_8;

  nco_in_8 = nco + step*8;
  if( ((nco^nco_in_8) & 3<<30) == 0) {
    switch(nco>>30) {
      case 0: *s = (*s<<8) | 0xCC;
              *c = (*c<<8) | 0x99;
              break;
      case 1: *s = (*s<<8) | 0x99;
              *c = (*c<<8) | 0x33;
              break;
      case 2: *s = (*s<<8) | 0x33;
              *c = (*c<<8) | 0x66;
              break;
      case 3: *s = (*s<<8) | 0x66;
              *c = (*c<<8) | 0xCC;
              break;
    }
    return;
  }

  for(j = 0; j < 8; j++) {
     switch(nco>>30) {
       case 0: *s = (*s << 1) | 1;
               *c = (*c << 1) | 1;
               break;
       case 1: *s = (*s << 1) | 1;
               *c = (*c << 1) | 0;
               break;
       case 2: *s = (*s << 1) | 0;
               *c = (*c << 1) | 0;
               break;
       case 3: *s = (*s << 1) | 0;
               *c = (*c << 1) | 1;
               break;
     }
     nco += step;
  }
}

/************************************************
*
************************************************/
static void fast_IF_nco_mask_16(uint_32 nco, uint_32 step, uint_32 *s, uint_32 *
c) {
  uint_32 nco_in_16;
  nco_in_16 = nco + step*16;
  if(((nco^nco_in_16) & 3<<30) == 0) {
    switch(nco>>30) {
      case 0: *s = (*s<<16) | 0xCCCC;
              *c = (*c<<16) | 0x9999;
              break;
      case 1: *s = (*s<<16) | 0x9999;
              *c = (*c<<16) | 0x3333;
              break;
      case 2: *s = (*s<<16) | 0x3333;
              *c = (*c<<16) | 0x6666;
              break;
      case 3: *s = (*s<<16) | 0x6666;
              *c = (*c<<16) | 0xCCCC;
              break;
    }
    return;
  }

  fast_IF_nco_mask_8(nco, step, s, c);
  nco += step * 8;
  fast_IF_nco_mask_8(nco, step, s, c);
}

/************************************************
*
************************************************/
static void fast_IF_nco_mask(uint_32 nco, uint_32 step, uint_32 *s, uint_32 *c)
{
  uint_32 nco_in_32;
  nco_in_32 = nco+step*32;
  if( ((nco^nco_in_32) & 3<<30) == 0) {
    switch(nco>>30) {
      case 0: *s = 0xCCCCCCCC;
              *c = 0x99999999;
              break;
      case 1: *s = 0x99999999;
              *c = 0x33333333;
              break;
      case 2: *s = 0x33333333;
              *c = 0x66666666;
              break;
      case 3: *s = 0x66666666;
              *c = 0xCCCCCCCC;
              break;
    }
    return;
  }

  fast_IF_nco_mask_16(nco, step, s, c);
  nco += step*16;
  fast_IF_nco_mask_16(nco, step, s, c);
}


void acquire_update(uint_32 samples) {
  int i;
  int s;

  uint_32 lo_s[N_BANDS], lo_c[N_BANDS];

  for(i = 0; i < N_BANDS; i++) {
    fast_IF_nco_mask(ncos_phase[i], ncos_step[i], lo_s+i, lo_c+i);
  }
  for(s = 0; s < N_PARALLEL; s++) {
    if(!states[s].sv_gold_codes)
     continue;

  /* Advance two code chips */
    if(states[s].max_offset < 2)
      states[s].max_offset += 1023-2;
    else
      states[s].max_offset -= 2;

    if(states[s].code_phase == 1023) {
      states[s].code_phase = 0;
    /* what we do when we have completly processed two cycles
       of the Gold Code */
      for(i = 0; i < N_BANDS; i++) {
        states[s].ones_s[i] -= 16368;
        states[s].ones_c[i] -= 16368;
      }

      for(i = 0; i < N_BANDS; i++) {
        uint_32 p;
      /* See if this is the highest power so far */
        p = states[s].ones_s[i]*states[s].ones_s[i]+states[s].ones_c[i]*states[s].ones_c[i];
        if(p > states[s].max_power) {
          states[s].max_power  = p;
          states[s].max_step   = ncos_step[i];
          if(i == 0) {
            states[s].max_step   = ncos_step[i]*3/4 + ncos_step[i+1]/4;
          } else if (i == N_BANDS-1) {
            states[s].max_step   = ncos_step[i]*3/4 + ncos_step[i-1]/4;
          } else {
#if 0
          uint_32 a,b;
          a = ones_s[i-1]*ones_s[i-1]+ones_c[i-1]*ones_c[i-1];
          b = ones_s[i+1]*ones_s[i+1]+ones_c[i+1]*ones_c[i+1];

          if(a > b) {
             if( p/4 > a)
               max_step   = ncos_step[i]/2 + ncos_step[i-1]/2;
             else
               max_step   = ncos_step[i]*3/4 + ncos_step[i-1]/4;
          }

          if(a < b) {
             if( p/4 > b)
               max_step   = ncos_step[i]/2 + ncos_step[i+1]/2;
             else
               max_step   = ncos_step[i]*3/4 + ncos_step[i+1]/4;
          }
#endif
            if(states[s].power_cb) {
              states[s].power_cb(states[s].current_sv,states[s].max_step,0,states[s].max_power/4);
            }
          }
          states[s].max_offset = 0;
        }
      }

      for(i = 0; i < N_BANDS; i++) {
        states[s].ones_s[i] = 0;
        states[s].ones_c[i] = 0;
      }

      /* Finish after 1023 tries */
      if(states[s].tries == 1023)
      {
       states[s].sv_gold_codes = NULL;
       if(states[s].finished_cb)
          states[s].finished_cb(states[s].current_sv, states[s].max_power);
      }
      else
         states[s].tries++;
    } else {
      uint_32 c = states[s].sv_gold_codes[states[s].code_phase*16];
      /* Normal progress */
      for(i = 0; i < N_BANDS; i++) {
        states[s].ones_s[i] += count_ones(samples ^ c ^ lo_s[i]);
        states[s].ones_c[i] += count_ones(samples ^ c ^ lo_c[i]);
      }
      if(states[s].code_phase == 1022)
        states[s].code_phase += 2-1023; 
      else
        states[s].code_phase += 2;
    }
  }
  
  for(i = 0; i < N_BANDS; i++) {
     ncos_phase[i] += ncos_step[i]*32;
  }
}

int acquire_stop(int sv_id) {
  int s;
  for(s = 0; s < N_PARALLEL; s++) {
    if(states[s].current_sv == sv_id)
      states[s].sv_gold_codes = NULL;
  }
  return -1;
}

int acquire_current_sv(int index) {
  if(index >= N_PARALLEL)
    return -1;
  if(states[index].sv_gold_codes == NULL) {
    return 0;
  }
  return states[index].current_sv;
}

int acquiring(int sv_id) {
  int s;
  for(s = 0; s < N_PARALLEL; s++) {
    if(states[s].sv_gold_codes && states[s].current_sv == sv_id)
      return 1;
  }
  return 0;
}
