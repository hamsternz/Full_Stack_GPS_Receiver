/****************************************************************************
* status.c - Show the status of the recevier (and also generate location fix)
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
#include "solve.h"
#include "nav.h"
#include "channel.h"
#include "status.h"
#include "acquire.h"

#define MAX_POS 10
static const double PI             = 3.1415926535898;

void show_status(double timestamp) {       
  int c, pos_sv[MAX_POS], lines;
       int bad_time_detected = 0,i;
       double lat,lon,alt;
       double pos_x[MAX_POS], pos_y[MAX_POS], pos_z[MAX_POS], pos_t[MAX_POS];
       
       int pos_used = 0;
       double sol_x=0.0, sol_y=0.0, sol_z=0.0, sol_t=0.0;

       printf("\n");
       printf("\n");
       printf("Update at %8.3f    Acquiring:", timestamp);
       for(i = 0; i < 32; i++) {
          int sv;
          sv =  acquire_current_sv(i);
          if(sv > 0)
            printf(" %02i",sv);
       }
       printf("\n");
       printf("Channel status:\n");
       printf("SV, WeekNum, FrameOfWeek,     msOfFrame,  earlyPwr, promptPwr,   latePwr, frame, bitErrors\n");
       lines = 0;
       for(c = 0; c < channel_get_count(); c++) {
         int sv,frames;
         uint_32 early_power, prompt_power, late_power;
         sv = channel_get_sv_id(c);
         if(sv == 0)
           continue;
         channel_get_power(c, &early_power, &prompt_power, &late_power);
         frames = nav_known_frames(sv); 
         printf("%02i, %7i,  %10i,  %12.7f, %9u, %9u, %9u,  %c%c%c%c%c  %6i\n", 
             channel_get_sv_id(c),
             nav_week_num(sv),
             nav_subframe_of_week(sv),
             nav_ms_of_frame(sv) + channel_get_nco_phase(c)/(channel_get_nco_limit()+1.0),
             early_power,prompt_power,late_power,
             frames & 0x01 ? '1' : '-',
             frames & 0x02 ? '2' : '-',
             frames & 0x04 ? '3' : '-',
             frames & 0x08 ? '4' : '-',
             frames & 0x10 ? '5' : '-',
             nav_get_bit_errors_count(sv)
         ); 
         lines++;
       }
       while(lines < 16) {
         printf("\n");
         lines++;
       }
       printf("\n");

       for(c = 0; c < channel_get_count() && pos_used < MAX_POS; c++) {
         double raw_time;
         int sv;
#if DROP_LOW_POWER
         uint_32 early_power, prompt_power, late_power;
         channel_get_power(c, &early_power, &prompt_power, &late_power);
         if(prompt_power < 1000000)
           continue;
#endif
         sv = channel_get_sv_id(c);
         if(sv == 0)
           continue;

         if(nav_week_num(sv) < 0 )
           continue;
         if(nav_ms_of_frame(sv) <0 )
           continue;
 
         raw_time = nav_ms_of_frame(sv) + channel_get_nco_phase(c)/(channel_get_nco_limit()+1.0);
         raw_time += nav_subframe_of_week(sv)*6000.0;
         raw_time /= 1000;
         if(!nav_calc_corrected_time(sv,raw_time, pos_t+pos_used))
           continue;
         if(pos_t[pos_used] < 0 || pos_t[pos_used] >= 7*24*3600)
           continue;
         if(!nav_calc_position(sv,pos_t[pos_used], pos_x+pos_used, pos_y+pos_used, pos_z+pos_used))
           continue;
         pos_sv[pos_used] = sv;
         pos_used++;
       }

       for(c = 0; c < pos_used; c++) {
         double diff = 0.0;
         int n = 0,c2;
         for(c2 = 0; c2 < pos_used; c2++) {
           if(c2 != c) {
             double d = pos_t[c2] - pos_t[c];
             /* Remove weekly wraps */
             if(d > 7*24*3600/2)
                d -= 7*24*3600/2;
             if(d < -7*24*3600/2)
                d += 7*24*3600/2;
             diff += d;
             n++;
           }
         }
         if(diff/n > 0.1) {
           bad_time_detected = 1;
           for(c2 = c; c < pos_used-1; c++) {
             pos_sv[c2] = pos_sv[c2+1];
             pos_t[c2]  = pos_t[c2+1];
             pos_x[c2]  = pos_x[c2+1];
             pos_y[c2]  = pos_y[c2+1];
             pos_z[c2]  = pos_z[c2+1];
           }
           pos_used--; 
         }
       }

       printf("Space Vehicle Positions:   %s\n", bad_time_detected ? "BAD TIME DETECTED - SV position dropped" : "");
       printf("sv,            x,            y,            z,            t\n"); 
       for(c = 0; c < pos_used; c++) {
         printf("%2i, %12.2f, %12.2f, %12.2f, %12.8f\n",pos_sv[c], pos_x[c], pos_y[c], pos_z[c], pos_t[c]);
       }
       while(c < 8) {
         printf("\n");
         c++;
       }

       if(pos_used > 3) { 
         printf("\n");
         solve_location(pos_used, pos_x, pos_y, pos_z, pos_t, &sol_x,&sol_y,&sol_z,&sol_t);
         solve_LatLonAlt(sol_x, sol_y, sol_z, &lat, &lon, &alt);

         printf("Solution ECEF: %12.2f, %12.2f, %12.2f, %11.5f\n", sol_x, sol_y, sol_z, sol_t);
         printf("Solution LLA:  %12.5f, %12.5f, %12.2f\n", lat*180/PI, lon*180/PI, alt);
       }
       else {
         printf("\n\n\n");
       }
     }
