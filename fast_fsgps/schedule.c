/****************************************************************************
* schedule.c - Scheduler for the attempts to acquire GPS Space Vehicles
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

static int acquire_launch_highest_priority(int sv_id);
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
  int i;

  /************************
  * Bump up priorities if
  * power was seen
  ************************/
  if(power > 500000)
     priorities[sv_id] += 24;
  else if(power > 300000)
     priorities[sv_id] += 28;
   

  /* Bump up everybody */
  for(i = 1; i < 33; i++) {
    if(channel_tracking(i) && nav_get_bit_errors_count(i) < 10)
      priorities[i]=0; 
    else if(nav_get_bit_errors_count(i) > 500)
      priorities[i]+=4; 
    else
      priorities[i]++; 
  }
  acquire_launch_highest_priority(sv_id);
}
/************************************************
*
************************************************/
void schedule_startup(void) {
   FILE *f;
   int i, q;

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
       }
     }
     fclose(f);
     printf("Finished prorities\n");
   }

   /**********************************
   * Launch as many acquire sessions as
   * possible
   ***********************************/
   for(i = 0; i < 10; i++) {
     if(!acquire_launch_highest_priority(1))
       break;
   } 
}
/************************************************
*
************************************************/
static int acquire_launch_highest_priority(int sv_id) 
{
  int max_sv = sv_id;
  int max_p = 0;
  int i;
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
  /* Drop 32 priority places before start */
  priorities[max_sv] -= 32;
  if(priorities[max_sv] <0 )
     priorities[max_sv] = 0;
  return acquire_start(max_sv, power_callback, finished_callback);
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
