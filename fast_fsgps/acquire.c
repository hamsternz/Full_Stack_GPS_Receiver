#include <stdio.h>
#include "types.h"
#include "gold_codes.h"
#include "acquire.h"

#define N_BANDS  41
#define IF_BAND (525*250)   // Bands are 250Hz wide,
uint_32 ncos_phase[N_BANDS];
uint_32 ncos_step[N_BANDS] = {
   0x40000000 -20*IF_BAND,
   0x40000000 -19*IF_BAND,
   0x40000000 -18*IF_BAND,
   0x40000000 -17*IF_BAND,
   0x40000000 -16*IF_BAND,
   0x40000000 -15*IF_BAND,
   0x40000000 -14*IF_BAND,
   0x40000000 -13*IF_BAND,
   0x40000000 -12*IF_BAND,
   0x40000000 -11*IF_BAND,
   0x40000000 -10*IF_BAND,
   0x40000000 - 9*IF_BAND,
   0x40000000 - 8*IF_BAND,
   0x40000000 - 7*IF_BAND,
   0x40000000 - 6*IF_BAND,
   0x40000000 - 5*IF_BAND,
   0x40000000 - 3*IF_BAND,
   0x40000000 - 4*IF_BAND,
   0x40000000 - 2*IF_BAND,
   0x40000000 - 1*IF_BAND,
   0x40000000 + 0*IF_BAND,
   0x40000000 + 1*IF_BAND,
   0x40000000 + 2*IF_BAND,
   0x40000000 + 3*IF_BAND,
   0x40000000 + 4*IF_BAND,
   0x40000000 + 5*IF_BAND,
   0x40000000 + 6*IF_BAND,
   0x40000000 + 7*IF_BAND,
   0x40000000 + 8*IF_BAND,
   0x40000000 + 8*IF_BAND,
   0x40000000 +10*IF_BAND,
   0x40000000 +11*IF_BAND,
   0x40000000 +12*IF_BAND,
   0x40000000 +13*IF_BAND,
   0x40000000 +14*IF_BAND,
   0x40000000 +15*IF_BAND,
   0x40000000 +16*IF_BAND,
   0x40000000 +17*IF_BAND,
   0x40000000 +18*IF_BAND,
   0x40000000 +19*IF_BAND,
   0x40000000 +20*IF_BAND
};

static uint_32 *sv_gold_codes;
static uint_32 code_phase;
static uint_32 ones_s[N_BANDS];
static uint_32 ones_c[N_BANDS];
static uint_32 tries;
static uint_32 max_offset;
static uint_32 max_power;
static uint_32 max_band;
static uint_32 current_sv;
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

void (*callback_finished)(int sv, uint_32 freq, uint_32 offset, uint_32 power);

static int count_ones(uint_32 a) {
  int rtn;

  rtn  = ones_lookup[ a      & 0xFF];
  rtn += ones_lookup[(a>> 8) & 0xFF];
  rtn += ones_lookup[(a>>16) & 0xFF];
  rtn += ones_lookup[(a>>24) & 0xFF];
  return rtn;
}

void acquire_startup(void) {
  setup_count_ones();
}

int acquire_start(int sv_id, void (*callback)(int sv, uint_32 freq, uint_32 offset, uint_32 power)) {
  int i;
  printf("Attempting to acquire sv %i\n",sv_id);
  callback_finished  = callback;
  current_sv = sv_id;
  sv_gold_codes = &(gold_codes_32_cycles[sv_id][0]);
  for(i = 0; i < N_BANDS; i++) {
    ones_s[i] = 0;
    ones_c[i] = 0;
  }
  tries = 0;
  max_power = 0;
  return 0;
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
  if(!sv_gold_codes)
     return;

  /* Advance two code chips */
  if(max_offset < 2)
    max_offset += 1023-2;
  else
    max_offset -= 2;

  if(code_phase == 1023) {
    int new_max = 0;
    code_phase = 0;
    /* what we do when we have completly processed two cycles
       of the Gold Code */
    for(i = 0; i < N_BANDS; i++) {
      uint_32 p;
      ones_s[i] -= 16368;
      ones_c[i] -= 16368;
      /* See if this is the highest power so far */
      p = ones_s[i]*ones_s[i]+ones_c[i]*ones_c[i];
      if(p > max_power) {
        max_power  = p;
        max_band   = i;
        max_offset = 0;
        if(p > 500000) {
          new_max = 1;
        }
      }
    }

    if(new_max) {
      printf("%02i: ",current_sv);
      for(i = 0; i < N_BANDS; i++) {
         uint_32 p;
         p = ones_s[i]*ones_s[i]+ones_c[i]*ones_c[i];
         printf("%4i, ",p/100000); 
      }
      printf("\n");
    }

    for(i = 0; i < N_BANDS; i++) {
      ones_s[i] = 0;
      ones_c[i] = 0;
    }
#if 1
    if(tries == 1023)
    {
       sv_gold_codes = NULL;
       if(callback_finished)
          callback_finished(current_sv,ncos_step[max_band],((1023<<22)-1) - (max_offset<<22),max_power);
    }
    else
       tries++;
#endif
  } else {
    uint_32 c = sv_gold_codes[code_phase*16];
    /* Normal progress */
    for(i = 0; i < N_BANDS; i++) {
      uint_32 lo_s, lo_c;
      fast_IF_nco_mask(ncos_phase[i], ncos_step[i], &lo_s, &lo_c);
      ones_s[i] += count_ones(samples ^ c ^ lo_s);
      ones_c[i] += count_ones(samples ^ c ^ lo_c);
    }
    if(code_phase == 1022)
      code_phase += 2-1023; 
    else
      code_phase += 2;
  }
  
  for(i = 0; i < N_BANDS; i++) {
     ncos_phase[i] += ncos_step[i]*32;
  }
}

int acquire_stop(int sv_id) {
  sv_gold_codes = NULL;
  return -1;
}
