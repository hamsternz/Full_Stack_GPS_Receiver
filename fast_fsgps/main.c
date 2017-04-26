#include <stdio.h>
#include <memory.h>
#include <math.h>
#include <assert.h>

typedef unsigned char uint_8;
typedef unsigned int  uint_32;
typedef int           int_32;
typedef char          int_8;
#include "channel.h"

/************************************************
*
************************************************/
struct Channel channel;
int main(int argc, char *argv[]) {
   static unsigned swap_bits[256];
   FILE *f;
   struct Channel *c;
   int q = 0;

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

   c = & channel;

   if(argc != 2) {
     printf("Please supply file name\n");
     return 0;
   }
   f = fopen(argv[1],"rb");
   if(f == NULL) {
     printf("Unable to open file\n");
     return 0;
   }
   channel_startup();

   /* Set the start values */
   memset(c,0,sizeof(struct Channel));
   c->nco_if    = 0;
   c->step_if   = 0x40104800;
   c->nco_code  = ((382)<<18);
   c->step_code = 0x00040000;
   c->code_tune = 10880;
#if 0
   c->nco_if    = 0;
   c->step_if   = 0x3ff9eb5a;
   c->nco_code  = ((11438)<<18);
   c->step_code = 0x00040000;
   c->code_tune = -4137;
#endif
//////////////////////////////////////////////////////////////////////////
// 32: Lower band     491 upper band     513 Adjust   21   Freq guess -1521
// 
// Lock band 7, Lock offset 11438, step 3ff9eb5a,  Code NCO        0
// lock_phase_nco_step  3ff9eb5a
// lock code_nco & step       80000      40000 -4137
//////////////////////////////////////////////////////////////////////////

   for(q = 0; q < 16368*1000/32*20; q++) {
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
     channel_update(c,data);
   }
   return 0;
}
