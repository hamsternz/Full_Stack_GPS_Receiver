#include <stdio.h>
#include <memory.h>
#include <math.h>
#include <assert.h>

#include "types.h"
#include "gold_codes.h"
#include "channel.h"

#define SHOW_CHANNEL_POWER 0
#define CALC_NOT_FILTERED  1
struct Channel {
   uint_32 nco_if;
   uint_32 step_if;

   uint_32 nco_code;
   uint_32 step_code;
   uint_32 code_tune;

   int_32 early_sine_count,  early_cosine_count,  early_sample_count;
   int_32 prompt_sine_count, prompt_cosine_count, prompt_sample_count;
   int_32 late_sine_count,   late_cosine_count,   late_sample_count;

   uint_32 early_power_filtered;
   uint_32 prompt_power_filtered;
   uint_32 late_power_filtered;

#if CALC_NOT_FILTERED
   uint_32 early_power_filtered_not_reset;
   uint_32 prompt_power_filtered_not_reset;
   uint_32 late_power_filtered_not_reset;
#endif

   uint_8 last_angle;
   int_32 delta_filtered;
   int_32 angle_filtered;
   uint_8 ms_of_bit;
   uint_8 last_bit;
   uint_8 no_adjust;
   uint_8 channel_allocated;
   uint_8 sv_id;
   uint_8 disable_track;
};

/* Filter factors */
#define LATE_EARLY_IIR_FACTOR       8
#define LOCK_DELTA_IIR_FACTOR       8
#define LOCK_ANGLE_IIR_FACTOR       8

/* For Debug */
#define LOCK_SHOW_ANGLES 0

#define ATAN2_SIZE 128
static uint_8 atan2_lookup[ATAN2_SIZE][ATAN2_SIZE];

#define MAX_CHANNELS 16
static struct Channel channels[MAX_CHANNELS];
static int channels_used = 0;

static uint_32 masks[32] = {
   0x00000000,
   0x00000001,
   0x00000003,
   0x00000007,
   0x0000000F,
   0x0000001F,
   0x0000003F,
   0x0000007F,
   0x000000FF,
   0x000001FF,
   0x000003FF,
   0x000007FF,
   0x00000FFF,
   0x00001FFF,
   0x00003FFF,
   0x00007FFF,
   0x0000FFFF,
   0x0001FFFF,
   0x0003FFFF,
   0x0007FFFF,
   0x000FFFFF,
   0x001FFFFF,
   0x003FFFFF,
   0x007FFFFF,
   0x00FFFFFF,
   0x01FFFFFF,
   0x03FFFFFF,
   0x07FFFFFF,
   0x0FFFFFFF,
   0x1FFFFFFF,
   0x3FFFFFFF,
   0x7FFFFFFF
};

void (*phase_callback)(int sv_id, int phase);

/***************************************
* Space to hold the current bitmaps info,
* to reduce the need to pass pointers
***************************************/
static uint_32 early_code,  early_mask;
static uint_32 prompt_code, prompt_mask;
static uint_32 late_code,   late_mask;
static int early_end_of_repeat;
static int prompt_end_of_repeat;
static int late_end_of_repeat;

/*********************************************************************
* Generate the atan2 table
*********************************************************************/
static void generate_atan2_table(void) {
    int x,y;
    for(x = 0; x < ATAN2_SIZE; x++) {
        for(y = 0; y < ATAN2_SIZE; y++) {
            atan2_lookup[y][x] = 0;
        }
    }

    for(x = -(ATAN2_SIZE-1)/2; x <=ATAN2_SIZE/2-1; x++) {
        for(y = -(ATAN2_SIZE-1)/2; y <= ATAN2_SIZE/2-1; y++) {
            double a = atan2(x,y);
            a *= 256.0 / 3.141592;
            a += 0.5;
            if(a < 0)
                a +=256;
            atan2_lookup[y&(ATAN2_SIZE-1)][x&(ATAN2_SIZE-1)] = a;
        }
    }
#if PRINT_ATAN2_TABLE
    for(x = 0; x < ATAN2_SIZE; x++) {
        for(y = 0; y < ATAN2_SIZE; y++) {
            printf("%5i,", atan2_lookup[y][x]);
        }
        putchar('\n');
    }
#endif
}

/************************************************
* A quick way to calculat the number of set bits
************************************************/
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
#if 0
#define count_ones(a) __builtin_popcountl(a)
#else
static int count_ones(uint_32 a) {
  int rtn;

  rtn  = ones_lookup[ a      & 0xFF];
  rtn += ones_lookup[(a>> 8) & 0xFF];
  rtn += ones_lookup[(a>>16) & 0xFF];
  rtn += ones_lookup[(a>>24) & 0xFF];
  return rtn;
}
#endif

/************************************************
* The NCO that generates the Gold code bistream
************************************************/
void fast_code_nco(uint_32 *gc_epl, uint_32 nco, uint_32 step) {
  uint_32 phaseE, phaseP, phaseL;

  phaseP = nco>>18;

  prompt_code = gc_epl[phaseP]; 
  prompt_end_of_repeat = 0;
  prompt_mask = 0;
  if(phaseP >= 1023*16-32) {
    prompt_end_of_repeat   = 1;
    prompt_mask   = masks[phaseP-1021*16];
  }

  phaseE = phaseP + (phaseP < 16*1023-12 ? 12 : 12-16368);
  early_code  = gc_epl[phaseE]; 
  early_end_of_repeat  = 0;
  early_mask  = 0;
  if(phaseE >= 1023*16-32) {
    early_end_of_repeat   = 1;
    early_mask   = masks[phaseE-1021*16];
  } 

  phaseL = phaseP - (phaseP < 12 ? 12-16368 : 12);
  late_code   = gc_epl[phaseL]; 
  late_end_of_repeat   = 0;
  late_mask   = 0;
  if(phaseL >= 1023*16-32) {
    late_end_of_repeat   = 1;
    late_mask   = masks[phaseL-1021*16];
  }
}
/************************************************
*
************************************************/
static void fast_IF_nco_mask_8(uint_32 nco, uint_32 step, uint_32 *s, uint_32 *c) {
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
static void fast_IF_nco_mask_16(uint_32 nco, uint_32 step, uint_32 *s, uint_32 *c) {
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
static void fast_IF_nco_mask(uint_32 nco, uint_32 step, uint_32 *s, uint_32 *c) {
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

/************************************************
*
************************************************/
static void adjust_prompt(struct Channel *ch) {
    int s, c;
    int_8 delta;
    int adjust = 0;
    uint_8 angle;

    s = ch->prompt_sine_count;
    c = ch->prompt_cosine_count;
    while(c > 15 || c < -15 || s > 15 || s < -15) {
        c /= 2;
        s /= 2;
    }

    s &= ATAN2_SIZE-1;  c &= ATAN2_SIZE-1;
    angle = atan2_lookup[c][s];
    delta = angle -ch->last_angle;
    ch->last_angle = angle;

    ch->delta_filtered -= ch->delta_filtered / LOCK_DELTA_IIR_FACTOR;
    ch->delta_filtered += delta;

    adjust = angle;
    ch->angle_filtered -= ch->angle_filtered / LOCK_ANGLE_IIR_FACTOR;
    ch->angle_filtered += angle;

    if(angle >=128)
        ch->angle_filtered -= 256;

    adjust  = ch->angle_filtered/8;
    adjust  += (1<<24) / 32 / LOCK_DELTA_IIR_FACTOR / 16368 * ch->delta_filtered;
    ch->step_if  -= adjust;

#if LOCK_SHOW_ANGLES
    printf("%6i, %6i, %3i,%4i,%6i, %6i, %6i\n",ch->prompt_sine_count, ch->prompt_cosine_count, angle,delta, ch->delta_filtered, adjust, ch->step_if);
#endif

    /* Pass the phase info to the external design */
    phase_callback(ch->sv_id, ch->prompt_cosine_count);
}

/************************************************
*
************************************************/
int  channel_add(int_8 sv_id, uint_32 step_if, uint_32 nco_code, int_32 code_tune) {
  int i;
  if(sv_id > MAX_SV_ID || sv_id < MIN_SV_ID)
     return -1;

  for(i = 0; i < channels_used; i++ ) {
    if(channels[i].sv_id == sv_id) {
      printf("=========== UPDATE %2i =================\n",sv_id);
      channels[i].step_if   = step_if;
      channels[i].nco_code  = nco_code;
      channels[i].code_tune = code_tune;
      channels[i].step_code = 0x00040000;
      return i;
    }
  } 

  if(channels_used == MAX_CHANNELS)
     return -1;
  channels[channels_used].sv_id     = sv_id;
  channels[channels_used].step_if   = step_if;
  channels[channels_used].nco_code  = nco_code;
  channels[channels_used].code_tune = code_tune;
  channels[channels_used].step_code = 0x00040000;
  channels_used++;
  return channels_used-1;
}
/************************************************
*
************************************************/
int channel_get_count(void) {
  return channels_used;
}
/************************************************
*
************************************************/
int channel_enable_track(int handle) {
  if(handle < 0 || handle >= channels_used)
    return -1;
  channels[handle].disable_track = 0;
  return 1;
}
/************************************************
*
************************************************/
int channel_disable_track(int handle) {
  if(handle < 0 || handle >= channels_used)
    return -1;
  channels[handle].disable_track = 1;
  return 1;
}
/************************************************
*
************************************************/
uint_32 channel_get_sv_id(int handle) {
  if(handle < 0 || handle >= channels_used)
    return -1;
  return channels[handle].sv_id;
}
/************************************************
*
************************************************/
int channel_get_power(int handle, uint_32 *early_power, uint_32 *prompt_power, uint_32 *late_power) {
  if(handle < 0 || handle >= channels_used)
    return -1;
  *prompt_power = channels[handle].prompt_power_filtered;
#if CALC_NOT_FILTERED
  *early_power  = channels[handle].early_power_filtered_not_reset;
  *late_power   = channels[handle].late_power_filtered_not_reset;
#else
  *early_power  = channels[handle].early_power_filtered;
  *late_power   = channels[handle].late_power_filtered;
#endif
  return 1;
}
/************************************************
*
************************************************/
uint_32 channel_get_nco_phase(int handle) {
  if(handle < 0 || handle >= channels_used)
    return -1;
  return channels[handle].nco_code;
}
/************************************************
*
************************************************/
uint_32 channel_get_nco_limit(void) {
   return (1023<<22)-1;
}

/************************************************
*
************************************************/
uint_32 channel_tracking(int sv_id) {
  int i;
  for(i = 0; i < channels_used; i++) {
     if(channels[i].sv_id == sv_id)
       return 1;
  }
  return 0;
}

/************************************************
*
************************************************/
void channel_startup(void (*callback)(int sv_id, int phase)) {
   phase_callback = callback; 
   generate_atan2_table();
   setup_count_ones();
   channels_used = 0;
}

/************************************************
*
************************************************/
void channel_update(uint_32 data) {
  int ch_no;
  struct Channel *c=channels;

  for(ch_no = 0; ch_no < channels_used; ch_no++) {
     uint_32 new_nco_code;
     uint_32 mixed_sine, mixed_cosine;
     uint_32 s_intermediate_freq=0, c_intermediate_freq=0;
  
     fast_IF_nco_mask(c->nco_if, c->step_if, &s_intermediate_freq, &c_intermediate_freq);
     c->nco_if   += c->step_if   * 32;

     /****************************************
     * Generate the gold codes for this set of
     * samples and advance the NCO   
     ****************************************/ 
     fast_code_nco(gold_codes_32_cycles[c->sv_id], c->nco_code, c->step_code);
     new_nco_code = c->nco_code + c->step_code*32;
     if(new_nco_code < c->nco_code)
        new_nco_code += 1<<22;
     if(new_nco_code >= (1023<<22))
        new_nco_code -= 1023<<22;
     c->nco_code = new_nco_code;

     /*****************************
     * Process the early codes
     *****************************/ 
     /* Mix the signals */
     mixed_sine   = data ^ early_code ^ s_intermediate_freq; 
     mixed_cosine = data ^ early_code ^ c_intermediate_freq; 

     /* Count how many bits are set */
     c->early_sine_count   += count_ones(mixed_sine);
     c->early_cosine_count += count_ones(mixed_cosine);
     c->early_sample_count += 32;

     if(early_end_of_repeat) {
         int next_sine, next_cosine, next_sample_count;
         next_sine         = count_ones(mixed_sine   & early_mask);
         next_cosine       = count_ones(mixed_cosine & early_mask);
         next_sample_count = count_ones(early_mask);
         c->early_sample_count -= next_sample_count;
         c->early_sine_count   -= next_sine   + c->early_sample_count/2;
         c->early_cosine_count -= next_cosine + c->early_sample_count/2;
         c->early_power_filtered -= c->early_power_filtered/LATE_EARLY_IIR_FACTOR;
         c->early_power_filtered += c->early_sine_count*c->early_sine_count + c->early_cosine_count*c->early_cosine_count;
#if CALC_NOT_FILTERED
         c->early_power_filtered_not_reset -= c->early_power_filtered_not_reset/LATE_EARLY_IIR_FACTOR;
         c->early_power_filtered_not_reset += c->early_sine_count*c->early_sine_count + c->early_cosine_count*c->early_cosine_count;
#endif
#if SHOW_CHANNEL_POWER
         if(c->sv_id == SHOW_CHANNEL_POWER)
           printf("%7i, ", c->early_power_filtered_not_reset);
#endif
         c->early_sine_count   = next_sine;
         c->early_cosine_count = next_cosine;
         c->early_sample_count = next_sample_count;
     }

     /*****************************
     * Process the prompt codes
     *****************************/ 
     /* Mix the signals */
     mixed_sine   = data ^ prompt_code ^ s_intermediate_freq; 
     mixed_cosine = data ^ prompt_code ^ c_intermediate_freq; 

     /* Count how many bits are set */
     c->prompt_sine_count   += count_ones(mixed_sine);
     c->prompt_cosine_count += count_ones(mixed_cosine);
     c->prompt_sample_count += 32;

     if(prompt_end_of_repeat) {
         int next_sine, next_cosine, next_sample_count;
         next_sine         = count_ones(mixed_sine   & prompt_mask);
         next_cosine       = count_ones(mixed_cosine & prompt_mask);
         next_sample_count = count_ones(prompt_mask);
         c->prompt_sample_count -= next_sample_count;
         c->prompt_sine_count   -= next_sine   + c->prompt_sample_count/2;
         c->prompt_cosine_count -= next_cosine + c->prompt_sample_count/2;
         c->prompt_power_filtered -= c->prompt_power_filtered/LATE_EARLY_IIR_FACTOR;
         c->prompt_power_filtered += c->prompt_sine_count*c->prompt_sine_count + c->prompt_cosine_count*c->prompt_cosine_count;
#if SHOW_CHANNEL_POWER
         if(c->sv_id == SHOW_CHANNEL_POWER)
           printf(" %7i, ", c->prompt_power_filtered);
#endif
           adjust_prompt(c);
         c->prompt_sine_count   = next_sine;
         c->prompt_cosine_count = next_cosine;
         c->prompt_sample_count = next_sample_count;
     }

     /*****************************
     * Process the late codes
     *****************************/ 
     /* Mix the signals */
     mixed_sine   = data ^ late_code ^ s_intermediate_freq; 
     mixed_cosine = data ^ late_code ^ c_intermediate_freq; 

     /* Count how many bits are set */
     c->late_sine_count   += count_ones(mixed_sine);
     c->late_cosine_count += count_ones(mixed_cosine);
     c->late_sample_count += 32;
     if(late_end_of_repeat && c->no_adjust == 0) {
         int next_sine, next_cosine, next_sample_count;
         int adjust;

         next_sine         = count_ones(mixed_sine   & late_mask);
         next_cosine       = count_ones(mixed_cosine & late_mask);
         next_sample_count = count_ones(late_mask);
         c->late_sample_count -= next_sample_count;
         c->late_sine_count   -= next_sine   + c->late_sample_count/2;
         c->late_cosine_count -= next_cosine + c->late_sample_count/2;
         c->late_power_filtered -= c->late_power_filtered/LATE_EARLY_IIR_FACTOR;
         c->late_power_filtered += c->late_sine_count*c->late_sine_count + c->late_cosine_count*c->late_cosine_count;
#if CALC_NOT_FILTERED
         c->late_power_filtered_not_reset -= c->late_power_filtered_not_reset/LATE_EARLY_IIR_FACTOR;
         c->late_power_filtered_not_reset += c->late_sine_count*c->late_sine_count + c->late_cosine_count*c->late_cosine_count;
#endif
#if SHOW_CHANNEL_POWER
         if(c->sv_id == SHOW_CHANNEL_POWER)
           printf(" %7i\n", c->late_power_filtered_not_reset);
#endif
         c->late_sine_count   = next_sine;
         c->late_cosine_count = next_cosine;
         c->late_sample_count = next_sample_count;

         /* Trim the NCO for the Gold Code */
         /* Use the relative power levels of the late and early codes
         * to adjust the code NCO phasing */
         if(!c->disable_track) {
           adjust =  c->code_tune;
           if(c->early_power_filtered/5 > c->late_power_filtered/4) {
             c->early_power_filtered = (c->early_power_filtered*7+c->late_power_filtered)/8;
             adjust += 16368*1;
             c->code_tune+=2;
           } else if(c->late_power_filtered/5 > c->early_power_filtered/4) {
             c->late_power_filtered = (c->late_power_filtered*7+c->early_power_filtered)/8;
             adjust  -= 16368*1;
             c->code_tune-=2;
           }
           c->nco_code += adjust;
  
           c->no_adjust = 1;
         }
      }
      c->no_adjust = 0;
      c++;
   }
}
/******************************************************
* END OF FILE
******************************************************/
