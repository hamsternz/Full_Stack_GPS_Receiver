#include "nav.h"
#define MAX_SV 33

struct Nav_data {
  int week_num;
  int frame_of_week;
  int ms_of_frame;
} nav_data[MAX_SV+1];

int nav_startup(void) {
  int i;
  for(i = 0; i <= MAX_SV; i++) {
    nav_data[i].week_num      = -1;
    nav_data[i].frame_of_week = 0;//-1;
    nav_data[i].ms_of_frame   = -1;
  } 
  return 1;
}

int nav_frame_of_week(int sv) {
  if(sv < 0 || sv > MAX_SV)
    return -1;
  return nav_data[sv].frame_of_week;
}

int nav_week_num(int sv) {
  if(sv < 0 || sv > MAX_SV)
    return -1;
  return nav_data[sv].week_num;
}

int nav_ms_of_frame(int sv) {
  if(sv < 0 || sv > MAX_SV)
    return -1;
  return nav_data[sv].ms_of_frame;
}

void nav_add_bit(int sv, int power) {
  if(sv < 0 || sv > MAX_SV)
    return;

  /* Update the counters so we can reinitialise
     if we need to while processing the power infor */
  if(nav_data[sv].ms_of_frame == 5999) {
     if(nav_data[sv].frame_of_week != -1)
       nav_data[sv].frame_of_week++;
     nav_data[sv].ms_of_frame = 0;
  } else
    nav_data[sv].ms_of_frame++;
}
