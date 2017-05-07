#include <stdio.h>
#include <memory.h>
#include <math.h>
#include <assert.h>

#include "types.h"
#include "gold_codes.h"
#include "channel.h"
#include "nav.h"
#include "solve.h"
#include "acquire.h"
#include "status.h"

#define DROP_LOW_POWER 0
static const double PI             = 3.1415926535898;

static uint_32 priorities[33];

/*************************************
*
*************************************/
void power_callback(int sv_id, uint_32 step_if, uint_32 offset, uint_32 power) {
  if(power > 70000) {
     uint_32 p;
     printf("Adding %02i, %08x, %08x\n",sv_id, step_if, offset);
     if(channel_get_power_by_sv_id(sv_id, &p)) {
         if(power > p*2) {
           channel_add(sv_id, step_if, offset, 0);
           nav_clear_bit_errors_count(sv_id);
         }
     } else {
       channel_add(sv_id, step_if, offset, 0);
     }
   }
}

/*************************************
*
*************************************/
void finished_callback(int sv_id, uint_32 power) {
  int max_sv = sv_id;
  int max_p = 0;
  int i;

  /************************
  * Reset priorities 
  ************************/
  if(power > 500000)
     priorities[sv_id] -= 4;
  else if(power > 300000)
     priorities[sv_id] -= 8;
  else
     priorities[sv_id] -= 32;
  if(priorities[sv_id] <0 )
     priorities[sv_id] = 0;

  /* Bump up everybody */
  for(i = 1; i < 33; i++) {
    if(channel_tracking(i) && nav_get_bit_errors_count(i) < 10)
      priorities[i]=0; 
    else if(nav_get_bit_errors_count(i) > 500)
      priorities[i]+=4; 
    else
      priorities[i]++; 
  }

  /* Find the next highest */
  for(i = 1; i < 33; i++) {
    sv_id++;  
    if(sv_id > 32)
      sv_id = 1;
    if(!channel_tracking(sv_id)) {
      if(max_p <  priorities[i]) {
         max_p =  priorities[i];
         max_sv = i;
      }
    }
  }
  acquire_start(max_sv, power_callback, finished_callback);
}

/************************************************
*
************************************************/
int main(int argc, char *argv[]) {
   static unsigned swap_bits[256];
   FILE *f;
   int q, start_acq=1;

   for(q = 0; q < 33; q++) {
     priorities[q] = 32;
   }

   printf("Loading prorities\n");
   f = fopen("priority.txt","r");
   if(f != NULL) {
     int i;
     while(fscanf(f,"%i",&i) == 1) {
       if(i > 0 && i < 33) {
         printf("Bumping for SV %i\n",i);
         priorities[i] += 32;
         start_acq = i;
       }
     }
     fclose(f);
   }
   printf("Finished prorities\n");

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
   gold_code_startup();
   nav_startup();
   acquire_startup();
   channel_startup(nav_add_bit);

   acquire_start(start_acq, power_callback, finished_callback);

   while(1) {
     uint_32 data;
     int ch;
     static int processed = 0;
     if(processed % ((16368000/32)/1) == 0) {
       show_status(processed*32.0/16368000);
     }
     processed++;

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
     acquire_update(data);  /* This has to go first ! */
     channel_update(data);
   }
   f = fopen("priority.txt","w+");
   for(q = 0; q < channel_get_count(); q++) { 
      int sv = channel_get_sv_id(q);
      if(sv > 0)
        fprintf(f,"%i\n",sv);
   }
   fclose(f);
   return 0;
}
