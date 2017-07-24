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
#include <stdlib.h>
#include <math.h>
#include <string.h>
#define USE_NCURSES 1
#if USE_NCURSES 
#include <ncurses.h>
#endif
#include "types.h"
#include "solve.h"
#include "nav.h"
#include "channel.h"
#include "status.h"
#include "acquire.h"

#define MAX_POS 12
static const double PI             = 3.1415926535898;

#define AVERAGE_LEN 100
static double average_x[AVERAGE_LEN];
static double average_y[AVERAGE_LEN];
static double average_z[AVERAGE_LEN];
static int average_index = -1;

static int using_ncurses;
#if USE_NCURSES 
static int using_color;
static int row;
#endif

double tot_error = 0.0;
int tot_count = 0;

void status_startup(void) {
#if USE_NCURSES 
  /* This sets up the screen */
  if(initscr()==NULL) {
    using_ncurses = 0;
    return;
  }

  using_ncurses = 1;
  if(has_colors()) {
    using_color = 1;
    start_color();
    init_pair(1, COLOR_GREEN,COLOR_BLACK);
    init_pair(2, COLOR_WHITE, COLOR_BLACK);
  }
  /* This sets up the keyboard to send character by
     character, without echoing to the screen */
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nonl();
  intrflush(stdscr, FALSE);
#endif
}

static void move_top_left(void) {
  if(!using_ncurses) {
    printf("\n");
    printf("\n");
    return;
  }  
#if USE_NCURSES 
   clear();
   move(0,0);
   row = 0;
#endif
}

int status_printf_ok(void) {
  return !using_ncurses;
}

static void show_line(char *h) {
  if(!using_ncurses) {
    printf("%s\n",h);
    return;
  }  
#if USE_NCURSES 
  move(row,0);
  if(!using_color) {
    printw("%s",h);
  } else {
    while(*h) {
      attron(COLOR_PAIR(1));
      while(*h && (*h < '0' || *h > '9') && *h != '-')  {
        printw("%c",*h);
        h++;
      }
      attron(COLOR_PAIR(2));
      while((*h >= '0' && *h <= '9') || *h == '-' || *h == '.' || *h == ' ')  {
        printw("%c",*h);
        h++;
      }
    }
  }
  row++;
#endif
}

static void update_screen(void) {
#if USE_NCURSES 
  refresh();
#endif
}

int vote_week_num(void) {
  int *week_nums;
  int *votes;
  int count = channel_get_count();
  int i, c;
  int max_week, max_votes;

  if(count == 0)
    return -1;

  week_nums = malloc(sizeof(int) * count); 
  if(week_nums == NULL)
    return -1;

  votes     = malloc(sizeof(int) * count); 
  if(votes == NULL) {
    free(week_nums);
    return -1;
  }

  for(i = 0; i < count; i++) {
    week_nums[i] = -1;
    votes[i]     = 0;
  }

  for(c = 0; c < count; c++) {
    int sv, week;

    sv = channel_get_sv_id(c);
    if(sv == 0)
      continue;

    /* Valid week? */
    week = nav_week_num(sv);
    if(week == -1)
       continue;

    /* Add a vote */
    for(i = 0; i < count; i++) {
      if(week_nums[i] == -1) {
        week_nums[i] = week;
        votes[i] = votes[i] + 1;
        break;
      } else if(week_nums[i] == week) {
        votes[i] = votes[i] + 1;
        break;
      }
    }
  }  

  max_votes = votes[0];
  max_week  = week_nums[0];
  for(i = 1; i < count; i++) {
    if(votes[i] > max_votes) {
      max_votes = votes[i];
      max_week  = week_nums[i];
    }
  }
  free(week_nums);
  free(votes);
  return max_week;
}

void status_show(double timestamp) {       
  char line[256];
  int c, pos_sv[MAX_POS], lines;
  int bad_time_detected = 0,i;
  int week_num;
  double x, y, z, error;
  double lat,lon,alt;
  double pos_x[MAX_POS], pos_y[MAX_POS], pos_z[MAX_POS], pos_t[MAX_POS];
  double agreed_time;
       
  int pos_used = 0;
  double sol_x=0.0, sol_y=0.0, sol_z=0.0, sol_t=0.0;
 
  move_top_left();
  sprintf(line,"Update at %8.3f    Acquiring:", timestamp);
  for(i = 0; i < 32; i++) {
     int sv;
     char text[10];
     sv =  acquire_current_sv(i);
     if(sv > 0) {
       sprintf(text," %02i",sv);
       strcat(line,text);
     }
  }
  show_line(line);

  show_line("Channel status:");
  show_line("SV, WeekNum, FrameOfWeek,  msOfFrame, early,prompt,  late, frame, bitErrs, codeTune");
  lines = 0;
  for(c = 0; c < channel_get_count(); c++) {
    int sv, frames;
    uint_32 early_power, prompt_power, late_power;

    sv = channel_get_sv_id(c);
    if(sv == 0)
      continue;

    channel_get_power(c, &early_power, &prompt_power, &late_power);
    frames = nav_known_frames(sv); 
    sprintf(line,"%02i, %7i,  %10i,  %9.4f, %5u, %5u, %5u,  %c%c%c%c%c  %6i %6i", 
        channel_get_sv_id(c),
        nav_week_num(sv),
        nav_subframe_of_week(sv),
        nav_ms_of_frame(sv) + channel_get_nco_phase(c)/(channel_get_nco_limit()+1.0),
        early_power>>10,prompt_power>>10,late_power>>10,
        frames & 0x01 ? '1' : '-',
        frames & 0x02 ? '2' : '-',
        frames & 0x04 ? '3' : '-',
        frames & 0x08 ? '4' : '-',
        frames & 0x10 ? '5' : '-',
        nav_get_bit_errors_count(sv),
        channel_get_code_tune(c)
    ); 
    show_line(line);
    lines++;
  }
  while(lines < 16) {
    show_line("");
    lines++;
  }
  week_num = vote_week_num();
  show_line("");

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

    if(nav_week_num(sv) != week_num )
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
    int n = 1, c2;
    for(c2 = c+1; c2 < pos_used; c2++) {
      double d = pos_t[c2] - pos_t[c];
      /* Remove weekly wraps */
      if(d > 7*24*3600/2)
         d -= 7*24*3600/2;
      if(d < -7*24*3600/2)
         d += 7*24*3600/2;
      if(fabs(d)< 0.1) {
        n++; 
      }
    }
    /* Do we have enough in agreement? */
    if(n > 3)
      break;
  }
  agreed_time = pos_t[c];
  for(c = 0; c < pos_used; c++) {
    int c2;
    double d = agreed_time - pos_t[c];
    /* Remove weekly wraps */
    if(d > 7*24*3600/2)
       d -= 7*24*3600/2;
    if(d < -7*24*3600/2)
       d += 7*24*3600/2;

    /* Is this an OK entry? */
    if(fabs(d) < 0.1)
       continue; 

    bad_time_detected = 1;

    /* Remove the bad time entry */
    for(c2 = c+1; c2 < pos_used && c2 < MAX_POS; c2++) {
      pos_sv[c2-1] = pos_sv[c2];
      pos_t[c2-1]  = pos_t[c2];
      pos_x[c2-1]  = pos_x[c2];
      pos_y[c2-1]  = pos_y[c2];
      pos_z[c2-1]  = pos_z[c2];
    }
    pos_used--; 
  }

  sprintf(line, "Space Vehicle Positions:   %s", bad_time_detected ? "BAD TIME DETECTED - SV position dropped" : "");
  show_line(line);
  show_line("sv,            x,            y,            z,            t"); 
  for(c = 0; c < pos_used; c++) {
    sprintf(line, "%02i, %12.2f, %12.2f, %12.2f, %12.8f",pos_sv[c], pos_x[c], pos_y[c], pos_z[c], pos_t[c]);
    show_line(line);
  }
  while(c < 12) {
    show_line("");
    c++;
  }

  if(pos_used > 3) { 
    int i;
    show_line("");
    solve_location(pos_used, pos_x, pos_y, pos_z, pos_t, &sol_x,&sol_y,&sol_z,&sol_t);
    solve_LatLonAlt(sol_x, sol_y, sol_z, &lat, &lon, &alt);

    sprintf(line,"Solution ECEF:   %12.2f, %12.2f, %12.2f, %11.5f", sol_x, sol_y, sol_z, sol_t);
    show_line(line);
    sprintf(line,"Solution LLA:    %12.7f, %12.7f, %12.2f", lat*180/PI, lon*180/PI, alt);
    show_line(line);
    {
      static FILE *solution_file = NULL;
      if(solution_file == NULL) {
         solution_file = fopen("solutions.txt","w");
      }
      if(solution_file != NULL) {
         fprintf(solution_file,"%12.7f, %12.7f, %12.2f\n",lat*180/PI, lon*180/PI, alt);
      }
    }
    /* Is this the first fix? */
    if(average_index == -1) {    
      int i;
      /* Set the initial values to the first fix */
      for(i = 0; i < AVERAGE_LEN; i++) {
         average_x[i] = sol_x;
         average_y[i] = sol_y;
         average_z[i] = sol_z;
      }
      average_index = 0;
    }
    average_x[average_index] = sol_x;
    average_y[average_index] = sol_y;
    average_z[average_index] = sol_z;

      /* Update and wrap the counter */
    average_index++;
    if(average_index == AVERAGE_LEN) average_index = 0;

    x = y = z = 0.0;
    for(i = 0; i < AVERAGE_LEN; i++) {
       x += average_x[i];
       y += average_y[i];
       z += average_z[i];
    }
    x /= AVERAGE_LEN;
    y /= AVERAGE_LEN;
    z /= AVERAGE_LEN;
    error = 0;
    for(i = 0; i < AVERAGE_LEN; i++) {
       double d;
       d  = (average_x[i]-x)*(average_x[i]-x);
       d += (average_y[i]-y)*(average_y[i]-y);
       d += (average_z[i]-z)*(average_z[i]-z);
       error += d;
    }
    error /= AVERAGE_LEN;

    solve_LatLonAlt(x, y, z, &lat, &lon, &alt);

    sprintf(line,"Average %3i LLA: %12.7f, %12.7f, %12.2f  (noise %6.2f)", AVERAGE_LEN, lat*180/PI, lon*180/PI, alt, sqrt(error));

    tot_error += error;
    tot_count++;

    show_line(line);
  } else {
    show_line("");
    show_line("");
    show_line("");
    show_line("");
  }
  update_screen();
}


void status_shutdown(void) {
  if(!using_ncurses)
    return;
#if USE_NCURSES 
  refresh();
  getch();
  endwin();
  printf("RMS Error %6.2f\n",sqrt(tot_error/tot_count));
#endif
}

