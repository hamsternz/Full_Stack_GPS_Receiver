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

static uint_32 priorities[33];

/*************************************
*
*************************************/
static void power_callback(int sv_id, uint_32 step_if, uint_32 offset, uint_32 power) {
  if(power > 70000) {
     uint_32 p;
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
static void finished_callback(int sv_id, uint_32 power) {
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
void schedule_startup(void) {
   FILE *f;
   int q, start_acq=1;

   for(q = 0; q < 33; q++) {
     priorities[q] = 32;
   }

   f = fopen("priority.txt","r");
   if(f != NULL) {
     int i;
     printf("Loading prorities\n");
     while(fscanf(f,"%i",&i) == 1) {
       if(i > 0 && i < 33) {
         printf("Bumping for SV %i\n",i);
         priorities[i] += 32;
         start_acq = i;
       }
     }
     fclose(f);
     printf("Finished prorities\n");
   }

   acquire_start(start_acq, power_callback, finished_callback);
}
/************************************************
*
************************************************/
void schedule_shutdown(void) {
   FILE *f;
   int q;

   f = fopen("priority.txt","w+");
   for(q = 0; q < channel_get_count(); q++) { 
      int sv = channel_get_sv_id(q);
      if(sv > 0)
        fprintf(f,"%i\n",sv);
   }
   fclose(f);
}
/************************************************
*
************************************************/
