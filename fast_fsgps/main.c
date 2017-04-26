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
   step_if   = 0x40104800;
   nco_code  = ((382)<<18);
   code_tune = 10880;
   channel_add(26, step_if, nco_code, code_tune);
#if 0
   step_if   = 0x3ff9eb5a;
   nco_code  = ((11438)<<18);
   code_tune = -4137;
   channel_add(32, step_if, nco_code, code_tune);
#endif
//////////////////////////////////////////////////////////////////////////
// 32: Lower band     491 upper band     513 Adjust   21   Freq guess -1521
// 
// Lock band 7, Lock offset 11438, step 3ff9eb5a,  Code NCO        0
// lock_phase_nco_step  3ff9eb5a
// lock code_nco & step       80000      40000 -4137
//////////////////////////////////////////////////////////////////////////

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
