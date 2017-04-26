#include <stdio.h>
#include <memory.h>
#include <math.h>
#include <assert.h>

typedef unsigned char uint_8;
typedef unsigned int  uint_32;
typedef int           int_32;
typedef char          int_8;
#include "channel.h"

struct Channel channel;
/************************************************
*
************************************************/
void channel_phase_callback(int sv_id, int ivalue) {
  if(sv_id != 4)
    return;
  if(ivalue > 0)
    putchar('1');
  else
    putchar('0');
}
/************************************************
*
************************************************/
int main(int argc, char *argv[]) {
   static unsigned swap_bits[256];
   FILE *f;
   int q;
   uint_32 step_if, nco_code;
   int_32 code_tune;
   int_8  sv_id;

   for(q = 0; q < 256; q++) {
     swap_bits[q] = 0;
     if(q&0x01) swap_bits[q] |= 0x80;
     if(q&0x02) swap_bits[q] |= 0x40;
     if(q&0x04) swap_bits[q] |= 0x20;
     if(q&0x08) swap_bits[q] |= 0x10;
     if(q&0x10) swap_bits[q] |= 0x08;
     if(q&0x20) swap_bits[q] |= 0x04;
     if(q&0x40) swap_bits[q] |= 0x02;
     if(q&0x80) swap_bits[q] |= 0x01;
   }

   if(argc != 2) {
     printf("Please supply file name\n");
     return 0;
   }
   f = fopen(argv[1],"rb");
   if(f == NULL) {
     printf("Unable to open file\n");
     return 0;
   }
   channel_startup(channel_phase_callback);

   /* Set the start values */
////////////////////////////////////////////////
//  4: Lower band    1360 upper band    1374 Adjust    5   Freq guess 3495
//
// Lock band 17, Lock offset 3494, step 400df8ea,  Code NCO        0
// lock_phase_nco_step  400df8ea
// lock code_nco & step       80000      40000 9506
////////////////////////////////////////////////
   sv_id     = 4;
   step_if   = 0x400df8ea;
   nco_code  = ((3494)<<18);
   code_tune = 9506;
   channel_add(sv_id, step_if, nco_code, code_tune);

////////////////////////////////////////////////
// 14: Lower band    1082 upper band    1125 Adjust   19   Freq guess -1019
// Lock band 8, Lock offset 7252, step 3ffbed1e,  Code NCO        0
// lock_phase_nco_step  3ffbed1e
// lock code_nco & step       80000      40000 -2771
////////////////////////////////////////////////
   sv_id     = 14;
   step_if   = 0x3ffbed1e;
   nco_code  = ((7252)<<18);
   code_tune = 2771;
   channel_add(sv_id, step_if, nco_code, code_tune);

////////////////////////////////////////////////
// 22: Lower band     409 upper band     623 Adjust  171   Freq guess 1329
// 
// Lock band 13, Lock offset 14091, step 40055026,  Code NCO        0
// lock_phase_nco_step  40055026
// lock code_nco & step       80000      40000 3614
////////////////////////////////////////////////
   sv_id     = 22;
   step_if   = 0x40055026;
   nco_code  = ((14091)<<18);
   code_tune = 3614;
   channel_add(sv_id, step_if, nco_code, code_tune);

////////////////////////////////////////////////
// 25: Lower band     812 upper band     690 Adjust   75   Freq guess -1075
// 
// Lock band 8, Lock offset 8153, step 3ffbb3ce,  Code NCO        0
// lock_phase_nco_step  3ffbb3ce
// lock code_nco & step       80000      40000 -2924
////////////////////////////////////////////////
   sv_id     = 25;
   step_if   = 0x3ffbb3ce;
   nco_code  = ((8153)<<18);
   code_tune = -2924;
   channel_add(sv_id, step_if, nco_code, code_tune);

////////////////////////////////////////////////
// 26: Lower band     679 upper band     657 Adjust   16   Freq guess 3984
// 
// Lock band 18, Lock offset 389, step 400fed60,  Code NCO        0
// lock_phase_nco_step  400fed60
// lock code_nco & step       80000      40000 10836
////////////////////////////////////////////////
   sv_id     = 26;
   step_if   = 0x40104800;
   nco_code  = ((382)<<18);
   code_tune = 10880;
   channel_add(sv_id, step_if, nco_code, code_tune);

////////////////////////////////////////////////
// 31: Lower band     384 upper band    1599 Adjust  379   Freq guess 121
// 
// Lock band 11, Lock offset 6145, step 40007bd6,  Code NCO        0
// lock_phase_nco_step  40007bd6
// lock code_nco & step       80000      40000 329
////////////////////////////////////////////////
   sv_id     = 31;
   step_if   = 0x40007bd6;
   nco_code  = ((6145)<<18);
   code_tune = 329;
   channel_add(sv_id, step_if, nco_code, code_tune);

////////////////////////////////////////////////
// 32: Lower band     491 upper band     513 Adjust   21   Freq guess -1521
// 
// Lock band 7, Lock offset 11438, step 3ff9eb5a,  Code NCO        0
// lock_phase_nco_step  3ff9eb5a
// lock code_nco & step       80000      40000 -4137
////////////////////////////////////////////////
   sv_id     = 32;
   step_if   = 0x3ff9eb5a;
   nco_code  = ((11438)<<18);
   code_tune = -4137;
   channel_add(sv_id, step_if, nco_code, code_tune);

   while(1) {
     uint_32 data;
     int ch;

     /* Read the data */
     ch  = getc(f);
     if(ch == EOF) break;
     data  = swap_bits[ch]<<24;
     ch  = getc(f);
     if(ch == EOF) break;
     data |= swap_bits[ch]<<16;
     ch = getc(f);
     if(ch == EOF) break;
     data |= swap_bits[ch]<<8;
     ch  = getc(f);
     if(ch == EOF) break;
     data |= swap_bits[ch]<<0;
     channel_update(data);
   }
   return 0;
}
