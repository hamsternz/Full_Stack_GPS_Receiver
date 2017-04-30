#include <stdio.h>
#include <memory.h>
#include <math.h>
#include <assert.h>

typedef unsigned char uint_8;
typedef unsigned int  uint_32;
typedef int           int_32;
typedef char          int_8;
#include "channel.h"

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

   uint_32 early_power_filtered_not_reset;
   uint_32 prompt_power_filtered_not_reset;
   uint_32 late_power_filtered_not_reset;

   int prompt_sc_temp;
   int prompt_cc_temp;

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
#define SHOW_CHANNEL_POWER 0

#define ATAN2_SIZE 128
uint_8 atan2_lookup[ATAN2_SIZE][ATAN2_SIZE];

#define MAX_CHANNELS 8
struct Channel channels[MAX_CHANNELS];
int channels_used = 0;

#define MIN_SV_ID 1
#define MAX_SV_ID 32
/* Note - we duplicate bits 0,1,2 at 1023,1024,1025 to speed things up */
uint_8 gold_codes[MAX_SV_ID+1][1027];

struct Space_vehicle {
   uint_8 sv_id;
   uint_8 tap1;
   uint_8 tap2;
} space_vehicles[] = {
  { 1,  2, 6},
  { 2,  3, 7},
  { 3,  4, 8},
  { 4,  5, 9},
  { 5,  1, 9},
  { 6,  2,10},
  { 7,  1, 8},
  { 8,  2, 9},
  { 9,  3,10},
  {10,  2, 3},
  {11,  3, 4},
  {12,  5, 6},
  {13,  6, 7},
  {14,  7, 8},
  {15,  8, 9},
  {16,  9,10},
  {17,  1, 4},
  {18,  2, 5},
  {19,  3, 6},
  {20,  4, 7},
  {21,  5, 8},
  {22,  6, 9},
  {23,  1, 3},
  {24,  4, 6},
  {25,  5, 7},
  {26,  6, 8},
  {27,  7, 9},
  {28,  8,10},
  {29,  1, 6},
  {30,  2, 7},
  {31,  3, 8},
  {32,  4, 9}
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

/**********************************************************************
* Generate the G1 LFSR bit stream
**********************************************************************/
static void g1_lfsr(unsigned char *out) {
  int state = 0x3FF,i;
  for(i = 0; i < 1023; i++) {
    int new_bit;
    out[i]   = (state >>9) & 0x1;
    /* Update the G1 LFSR */
    new_bit = ((state >>9) ^ (state >>2))&1;
    state   = ((state << 1) | new_bit) & 0x3FF;
  }
}

/**********************************************************************
* Generate the G2 LFSR bit stream. Different satellites have different
* taps, which effectively alters the relative phase of G1 vs G2 codes
**********************************************************************/
static void g2_lfsr(unsigned char tap0, unsigned char tap1, unsigned char *out) {
  int state = 0x3FF,i;
  /* Adjust tap number from 1-10 to 0-9 */
  tap0--;
  tap1--;
  for(i = 0; i < 1023; i++) {
    int new_bit;

    out[i] = ((state >> tap0) ^ (state >> tap1)) & 0x1;

    /* Update the G2 LFSR  */
    new_bit = ((state >>9) ^ (state >>8) ^
               (state >>7) ^ (state >>5) ^
               (state >>2) ^ (state >>1))&1;
    state = ((state << 1) | new_bit) & 0x3FF;
  }
}

/**********************************************************************
* Combine the G1 and G2 codes to make each satellites code
**********************************************************************/
static void combine_g1_and_g2(unsigned char *g1, unsigned char *g2, unsigned char *out)
{
  int i;
  for(i = 0; i < 1023; i++ ) {
    out[i] = g1[i] ^ g2[i];
  }
}

/*********************************************************************
* Build the Gold codes for each Satellite from the G1 and G2 streams
*********************************************************************/
void generate_gold_codes(void) {
  int sv;
  static unsigned char g1[1023];
  static unsigned char g2[1023];
  g1_lfsr(g1);
  for(sv = 0; sv < sizeof(space_vehicles)/sizeof(struct Space_vehicle); sv++) {
    g2_lfsr(space_vehicles[sv].tap1, space_vehicles[sv].tap2, g2);
    combine_g1_and_g2(g1, g2, gold_codes[ space_vehicles[sv].sv_id]);
    /* This avoids needing to wrap in the most timing senstive code */
    gold_codes[ space_vehicles[sv].sv_id][1023] = gold_codes[ space_vehicles[sv].sv_id][0];
    gold_codes[ space_vehicles[sv].sv_id][1024] = gold_codes[ space_vehicles[sv].sv_id][1];
    gold_codes[ space_vehicles[sv].sv_id][1025] = gold_codes[ space_vehicles[sv].sv_id][2];
  }
}

/*********************************************************************
* Generate the atan2 table
*********************************************************************/
void generate_atan2_table(void) {
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
void setup_count_ones(void) {
  int i;
  for(i = 0; i < 256; i++) {
    int j;
    for(j = 0; j < 32; j++) {
      if(i&(1<<j))
        ones_lookup[i]++;
    }
  } 
}
#if 0
#define count_ones(a) __builtin_popcountl(a)
#else
int count_ones(uint_32 a) {
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
void fast_code_nco(uint_8 *gc, uint_32 nco, uint_32 step) {
  int i,n0,n1,n2;
  int wrap0, wrap1, wrap2, wrap3;
  uint_8 codeSub1, code0, code1, code2, code3;
  uint_32 mask;

  i = nco>>22;

  wrap0 = (i ==    0) ? 1 : 0;
  wrap1 = (i == 1022) ? 1 : 0;
  wrap2 = (i == 1021) ? 1 : 0;
  wrap3 = (i == 1020) ? 1 : 0;

  codeSub1 = gc[(i==0) ? 1022 : i-1];
  code0 = gc[i+0];
  code1 = gc[i+1];
  code2 = gc[i+2];
  code3 = gc[i+3];
  
  /* Work out how many bits of each code chip */
  n0 = n1 = n2 = 0;  
  
  nco &= ((1<<22)-1);
#if 0
  while(nco < (1<<22)) {
    nco += step;
    n0++;
  }
  while((nco < (2<<22)) && n0+n1 < 32) {
    nco += step;
    n1++;
  }
  n2 = 32 - (n0 + n1);
#else
  n0 = ((1<<22)+step-1-nco) / step;
  n1 = ((2<<22)+step-1-nco) / step - n0;
  if(n0+n1 > 32)
    n1 = 32 - n0;
  n2 = 32-n1-n0;
  nco += step * 32;
#endif


  late_code = prompt_code = early_code = 0;

  /***************************************************
  * First code bit in results 
  ***************************************************/ 
  if(codeSub1) late_code   = 0xFFFFFFFF;
  if(code0)    prompt_code = 0xFFFFFFFF;
  if(code1)    early_code  = 0xFFFFFFFF;


  /***************************************************
  * Second code bit in results 
  ***************************************************/ 
  late_code   <<= n1;
  prompt_code <<= n1;
  early_code  <<= n1;
  mask = (1<<n1)-1;
  if(code0) late_code   |= mask;
  if(code1) prompt_code |= mask;
  if(code2) early_code  |= mask;

  /***************************************************
  * Third (and not always present) code bit in results 
  ***************************************************/ 
  if(n2 > 0) {
	late_code   <<= n2;
	prompt_code <<= n2;
	early_code  <<= n2;
        mask = (1<<n2)-1;
	if(code1) late_code   |= mask;
	if(code2) prompt_code |= mask;
	if(code3) early_code  |= mask;
  }

  early_end_of_repeat  = 0;
  prompt_end_of_repeat = 0;
  late_end_of_repeat   = 0;
  early_mask  = 0;
  prompt_mask = 0;
  late_mask   = 0;

  /****************************************************
  * Now to generate the masks for when we have wrapped
  ****************************************************/
  if(wrap0) {
	  late_mask = 0xFFFFFFFF;
	  late_mask >>= n0;
          late_end_of_repeat = 1;
  } else if(wrap1 && (nco >= (2<<22))) {
	  late_mask = 0xFFFFFFFF;
	  late_mask >>= n0;
	  late_mask >>= n1;
          late_end_of_repeat = 1;
  }

  if(wrap1) {
	  prompt_mask = 0xFFFFFFFF;
	  prompt_mask >>= n0;
          prompt_end_of_repeat = 1;
  } else if(wrap2 && (nco >= (2<<22))) {
	  prompt_mask = 0xFFFFFFFF;
	  prompt_mask >>= n0;
	  prompt_mask >>= n1;
          prompt_end_of_repeat = 1;
  }

  if(wrap2) {
	  early_mask = 0xFFFFFFFF;
	  early_mask >>= n0;
          early_end_of_repeat = 1;
  } else if(wrap3 && (nco >= (2<<22))) {
	  early_mask = 0xFFFFFFFF;
	  early_mask >>= n0;
	  early_mask >>= n1;
          early_end_of_repeat = 1;
  }
}

/************************************************
*
************************************************/
void fast_IF_nco_mask_8(uint_32 nco, uint_32 step, uint_32 *s, uint_32 *c) {
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
void fast_IF_nco_mask_16(uint_32 nco, uint_32 step, uint_32 *s, uint_32 *c) {
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
void fast_IF_nco_mask(uint_32 nco, uint_32 step, uint_32 *s, uint_32 *c) {
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
  if(channels_used == MAX_CHANNELS)
     return -1;
  if(sv_id > MAX_SV_ID || sv_id < MIN_SV_ID)
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
  *early_power  = channels[handle].early_power_filtered_not_reset;
  *prompt_power = channels[handle].prompt_power_filtered;
  *late_power   = channels[handle].late_power_filtered_not_reset;
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
void channel_startup(void (*callback)(int sv_id, int phase)) {
   phase_callback = callback; 
   generate_atan2_table();
   generate_gold_codes();
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
     fast_code_nco(gold_codes[c->sv_id], c->nco_code, c->step_code);
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
         c->early_power_filtered_not_reset -= c->early_power_filtered_not_reset/LATE_EARLY_IIR_FACTOR;
         c->early_power_filtered_not_reset += c->early_sine_count*c->early_sine_count + c->early_cosine_count*c->early_cosine_count;
#if SHOW_CHANNEL_POWER
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
         printf(" %7i, ", c->prompt_power_filtered);
#endif
           adjust_prompt(c);
         c->prompt_sc_temp = c->prompt_sine_count;
         c->prompt_cc_temp = c->prompt_cosine_count;
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
         c->late_power_filtered_not_reset -= c->late_power_filtered_not_reset/LATE_EARLY_IIR_FACTOR;
         c->late_power_filtered_not_reset += c->late_sine_count*c->late_sine_count + c->late_cosine_count*c->late_cosine_count;
#if SHOW_CHANNEL_POWER
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
