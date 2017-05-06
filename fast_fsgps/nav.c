/****************************************************************************
* nav.c - Processing the GPS NAV data
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
#include <malloc.h>
#include <string.h>
#include <memory.h>
#include <math.h>
#include <time.h>
#include "types.h"
#include "nav.h"

#define MAX_SV 33

#define MS_PER_BIT                (20)
#define BIT_LENGTH                (MS_PER_BIT)
#define BITS_PER_FRAME            (300)

static const double EARTHS_RADIUS  =   6317000.0;
static const double SPEED_OF_LIGHT = 299792458.0;
static const double PI             = 3.1415926535898;
static const double mu             = 3.986005e14;      /* Earth's universal gravitation parameter */
static const double omegaDot_e     = 7.2921151467e-5;  /* Earth's rotation (radians per second) */
static const int    TIME_EPOCH     = 315964800;


/*************************************************
* Raw data recevied from the Space Vehicles
*************************************************/
struct Raw_navdata {
  uint_32 seq;
  uint_8  synced;
  uint_32 subframe_of_week;

  /* Last 32 bits of data received*/
  uint_8  part_in_bit;
  uint_8  subframe_in_frame;
  uint_32 new_word;
  
  /* A count of where we are upto in the subframe (-1 means unsynced) */
  int_32  valid_bits;
  int_32  bit_errors;

  /* Uncoming, incomplete frame */
  uint_32 new_subframe[10];

  /* Complete NAV data frames (only 1 to 5 is used, zero is empty) */
  uint_8  valid_subframe[6];
  uint_32 subframes[6][10];  
};

/*************************************************
* time/clock data received from the Space Vehicles
*************************************************/
struct Nav_time {
  uint_8   time_good;
  uint_32  week_num;
  uint_32  user_range_accuracy;
  uint_32  health;
  uint_32  issue_of_data;
  double   group_delay;    /* Tgd */
  double   reference_time; /* Toc */
  double   correction_f2;  /* a_f2 */
  double   correction_f1;  /* a_f1 */
  double   correction_f0;  /* a_f0 */
};

/*************************************************
* Orbit parameters recevied from the Space Vehicles
*************************************************/
struct Nav_orbit {
  /* Orbit data */
  uint_8  fit_flag;
  uint_8  orbit_valid;
  uint_32 iode; 
  uint_32 time_of_ephemeris; 
  uint_32 aodo;
  double  mean_motion_at_ephemeris; 
  double  sqrt_A;
  double  Cus;
  double  Cuc;
  double  Cis;
  double  Cic;
  double  Crc;
  double  Crs;
  double  delta_n;
  double  e;
  double  omega_0;
  double  idot;
  double  omega_dot;
  double  inclination_at_ephemeris;
  double  w;

};
/*************************************************
*                                             
*************************************************/
struct Nav_data {
  int sv_id;
  int ms_of_frame;
  uint_32 subframe_of_week;
  FILE *nav_file;
  struct Raw_navdata raw_navdata;
  struct Nav_time    nav_time;
  struct Nav_orbit   nav_orbit;
} nav_data[MAX_SV+1];

/*************************************************
*                                             
*************************************************/
int nav_bit_sync(int sv) {
   if(nav_data[sv].raw_navdata.valid_bits > 1)
      return 1;
   return 0;
}

/*************************************************
*                                             
*************************************************/
int nav_subframe_of_week(int sv) {
  if(sv < 0 || sv > MAX_SV)
    return -1;
  if(!nav_data[sv].nav_time.time_good)
    return -1;

  return nav_data[sv].subframe_of_week;
}

/*************************************************
*                                             
*************************************************/
int nav_week_num(int sv) {
  if(sv < 0 || sv > MAX_SV)
    return -1;
  if(!nav_data[sv].nav_time.time_good)
    return -1;
  return nav_data[sv].nav_time.week_num;
}

/*************************************************
*                                             
*************************************************/
int nav_known_frames(int sv) {
  int rtn;
  if(sv < 0 || sv > MAX_SV)
    return -1;
  rtn = 0;
  if(nav_data[sv].raw_navdata.valid_subframe[1]) rtn |= 0x01;
  if(nav_data[sv].raw_navdata.valid_subframe[2]) rtn |= 0x02;
  if(nav_data[sv].raw_navdata.valid_subframe[3]) rtn |= 0x04;
  if(nav_data[sv].raw_navdata.valid_subframe[4]) rtn |= 0x08;
  if(nav_data[sv].raw_navdata.valid_subframe[5]) rtn |= 0x10;
  return rtn;
}

/*************************************************
*                                             
*************************************************/
int nav_ms_of_frame(int sv) {
  if(sv < 0 || sv > MAX_SV)
    return -1;
  return nav_data[sv].ms_of_frame;
}




/************************************************************
*
************************************************************/
static double  orbit_ecc_anom(struct Nav_data *nd, double t) {
    int    iterations  = 200;
    double delta       = pow(10,-10);
    double estimate, correction, semi_major_axis,     computed_mean_motion;
    double time_from_ephemeris,  correct_mean_motion, mean_anomaly;

    semi_major_axis      = nd->nav_orbit.sqrt_A * nd->nav_orbit.sqrt_A;
    computed_mean_motion = sqrt(mu / pow(semi_major_axis,3.0));

    time_from_ephemeris  = t - nd->nav_orbit.time_of_ephemeris;
    if(time_from_ephemeris >  302400) time_from_ephemeris  -= 604800;
    if(time_from_ephemeris < -302400) time_from_ephemeris  += 604800;
    correct_mean_motion  = computed_mean_motion + nd->nav_orbit.delta_n;

    /* Add on how much we have moved through the orbit since ephemeris */
    mean_anomaly         = nd->nav_orbit.mean_motion_at_ephemeris + correct_mean_motion * time_from_ephemeris;

    /* First estimate */
    estimate   = (nd->nav_orbit.e<0.8) ? mean_anomaly :  PI;
    correction = estimate - (mean_anomaly + nd->nav_orbit.e*sin(mean_anomaly));

    /* Solve iteratively */
    while ((fabs(correction)>delta) && iterations > 0) {
        double last = estimate;
        estimate  = mean_anomaly  + nd->nav_orbit.e * sin(estimate);
        correction = estimate - last;
        iterations--;
    }

    if(iterations == 0) {
        printf("Error calculating Eccentric Anomaly\n");
    }
    return estimate;
}

/************************************************************
*
************************************************************/
int nav_calc_corrected_time(int sv_id, double raw_t, double *t) {
  double delta_t, delta_tr, ek, time_correction;
  struct Nav_data *nd;
  if(sv_id < 0 || sv_id > MAX_SV)
    return 0;

  if(!nav_data[sv_id].nav_time.time_good)
    return -1;

  nd = nav_data+sv_id;

  /* Calulate the time for the adjustment */
  delta_t = raw_t - nd->nav_time.reference_time;
  if(delta_t >  302400)  delta_t -= 604800;
  if(delta_t < -302400)  delta_t += 604800;

  /* Relativistic term */
  ek = orbit_ecc_anom(nd, raw_t);

  delta_tr = -4.442807633e-10 * nd->nav_orbit.e * nd->nav_orbit.sqrt_A * sin(ek);

  time_correction = nd->nav_time.correction_f0 
                  + (nd->nav_time.correction_f1 * delta_t) 
                  + (nd->nav_time.correction_f2 * delta_t * delta_t) 
                  + delta_tr
                  - nd->nav_time.group_delay;
  *t = raw_t - time_correction;
  return 1;
}

/**************************************************************************
* Calculate where the Space Vehicle will be at time "pos_t"
**************************************************************************/
int nav_calc_position(int sv_id, double t, double *x, double *y, double *z)
{
  double time_from_ephemeris,   semi_major_axis;
  double ek, true_anomaly,      corrected_argument_of_latitude;
  double argument_of_latitude,  argument_of_latitude_correction;
  double radius_correction,     corrected_radius;
  double correction_of_inclination;
  double pos_in_orbial_plane_x, pos_in_orbial_plane_y;
  double corrected_inclination, corrected_angle_of_ascending_node;
  struct Nav_data *nd;
  if(sv_id < 0 || sv_id > MAX_SV)
    return 0;
  nd = nav_data+sv_id;
  if(!nd->nav_orbit.orbit_valid)
    return 0;
    
  
  /***********************
  * Calculate orbit
  ***********************/
  time_from_ephemeris  = t - nd->nav_orbit.time_of_ephemeris;
  if(time_from_ephemeris >  302400) time_from_ephemeris  -= 604800;
  if(time_from_ephemeris < -302400) time_from_ephemeris  += 604800;

  semi_major_axis      = nd->nav_orbit.sqrt_A * nd->nav_orbit.sqrt_A;
  ek = orbit_ecc_anom(nd, t);

  /* Now calculate the first approximation of the latitude */
  true_anomaly = atan2( sqrt(1-nd->nav_orbit.e * nd->nav_orbit.e) * sin(ek), cos(ek) - nd->nav_orbit.e);
  argument_of_latitude = true_anomaly + nd->nav_orbit.w;

  /*****************************************
  * Second Harmonic Perbturbations 
  *****************************************/
  argument_of_latitude_correction = nd->nav_orbit.Cus * sin(2*argument_of_latitude) 
                                  + nd->nav_orbit.Cuc * cos(2*argument_of_latitude);

  radius_correction               = nd->nav_orbit.Crc * cos(2*argument_of_latitude) 
                                  + nd->nav_orbit.Crs * sin(2*argument_of_latitude);
  
  correction_of_inclination       = nd->nav_orbit.Cic * cos(2*argument_of_latitude) 
                                  + nd->nav_orbit.Cis * sin(2*argument_of_latitude);

  corrected_argument_of_latitude  = argument_of_latitude + argument_of_latitude_correction;
  corrected_radius                = semi_major_axis * (1- nd->nav_orbit.e * cos(ek)) + radius_correction;
  corrected_inclination           = nd->nav_orbit.inclination_at_ephemeris + correction_of_inclination 
                                  + nd->nav_orbit.idot*time_from_ephemeris;

  pos_in_orbial_plane_x = corrected_radius * cos(corrected_argument_of_latitude);
  pos_in_orbial_plane_y = corrected_radius * sin(corrected_argument_of_latitude);
  

  corrected_angle_of_ascending_node = nd->nav_orbit.omega_0
                                    + (nd->nav_orbit.omega_dot - omegaDot_e)*time_from_ephemeris 
                                    - omegaDot_e * nd->nav_orbit.time_of_ephemeris;

  /******************************************************
  * Project into Earth Centered, Earth Fixed coordinates
  ******************************************************/
  *x = pos_in_orbial_plane_x * cos(corrected_angle_of_ascending_node)
     - pos_in_orbial_plane_y * cos(corrected_inclination) * sin(corrected_angle_of_ascending_node);
  *y = pos_in_orbial_plane_x * sin(corrected_angle_of_ascending_node) 
     + pos_in_orbial_plane_y * cos(corrected_inclination) * cos(corrected_angle_of_ascending_node);
  *z = pos_in_orbial_plane_y * sin(corrected_inclination);

  return 1;
}

/*****************************************************************************
******************************************************************************
* Routines for processing NAV messages (50 bps BSPK from the satellites)     *
******************************************************************************
*****************************************************************************/
static const unsigned char parity[32] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x25,
    0x0B, 0x16, 0x2C, 0x19, 0x32, 0x26, 0x0E, 0x1F,
    0x3E, 0x3D, 0x38, 0x31, 0x23, 0x07, 0x0D, 0x1A,
    0x37, 0x2F, 0x1C, 0x3B, 0x34, 0x2A, 0x16, 0x29};   

static int nav_test_parity(uint_32 d)
{
    int i;
    /* 'd' holds the 32 bits received, and this function will
    * return true if the low 30 bits is a valid subframe - 
    * the most recent bit is held in bit zero */
    i = d>>30;

    /* If the last bit of the last subframe is set
    * then the data in this frame is flipped */
    if(i & 1)
        d ^= 0x3FFFFFC0;

    for(i = 6; i < 32; i++) { 
        if(d&(1<<i)) d ^= parity[i];
    }
    if((d & 0x3F) != 0) return 0;

    return 1; /* Success! */
}


/*****************************************************************************
* Used for pulling data out of the raw NAV frames 
******************************************************************************/
static unsigned int mask(unsigned u, int n_bits) {
  return u & ((1<<n_bits)-1);
}

/******************************************************************************/
static int sign_extend(unsigned u,int len) {
  if(len < 32 && u >0)
    if(u>>(len-1)&1) u |= 0xFFFFFFFF << len;
  return (int)u;
}

/******************************************************************************/
static unsigned int bits(int val, int offset, int len) {
  return mask(val >> offset,len);
}

/******************************************************************************/
static unsigned  join_bits_u(int val1, int offset1, int len1, int val2, int offset2, int len2) {
  return (bits(val1, offset1, len1) << len2) | bits(val2, offset2, len2);
}

/******************************************************************************/
static signed  join_bits_s(int val1, int offset1, int len1, int val2, int offset2, int len2) {
  return sign_extend(join_bits_u(val1, offset1, len1, val2, offset2, len2),len1+len2);
}

/*****************************************************************************
******************************************************************************
* Routines for dumping out data while debugging                              *
******************************************************************************
*****************************************************************************/
static void debug_print_orbit(struct Nav_data *nd) {
  printf("\nOrbit parameters for SV %i:\n",   nd->sv_id);
  printf("iode      %02X\n",                  nd->nav_orbit.iode);
  printf("double M0        = %2.30g;\n", nd->nav_orbit.mean_motion_at_ephemeris);
  printf("double delta_n   = %2.30g;\n", nd->nav_orbit.delta_n);
  printf("double e         = %2.30g;\n", nd->nav_orbit.e);
  printf("double sqrt_A    = %2.30g;\n", nd->nav_orbit.sqrt_A);
  printf("double omega_0   = %2.30g;\n", nd->nav_orbit.omega_0);
  printf("double i_0       = %2.30g;\n", nd->nav_orbit.inclination_at_ephemeris);
  printf("double w         = %2.30g;\n", nd->nav_orbit.w);
  printf("double omega_dot = %2.30g;\n", nd->nav_orbit.omega_dot);
  printf("double idot      = %2.30g;\n", nd->nav_orbit.idot);
  printf("double Cuc       = %2.30g;\n", nd->nav_orbit.Cuc);
  printf("double Cus       = %2.30g;\n", nd->nav_orbit.Cus);
  printf("double Crc       = %2.30g;\n", nd->nav_orbit.Crc);
  printf("double Crs       = %2.30g;\n", nd->nav_orbit.Crs);
  printf("double Cic       = %2.30g;\n", nd->nav_orbit.Cic);
  printf("double Cis       = %2.30g;\n", nd->nav_orbit.Cis);
  printf("unsigned Toe     = %u;\n", nd->nav_orbit.time_of_ephemeris);
  printf("Fit       %01x\n",                  nd->nav_orbit.fit_flag);
  printf("aodo      %02X\n",                  nd->nav_orbit.aodo);    
  printf("\n");
}

void debug_print_time(struct Nav_data *nd) {
  struct tm  ts;
  char       buf[80];
  time_t     timestamp;

  printf("\nTime parameters for SV %i:\n",   nd->sv_id);
  printf("Week No    %i\n", nd->nav_time.week_num);
  printf("Accuracy   %i\n", nd->nav_time.user_range_accuracy);
  printf("Health     %i\n", nd->nav_time.health);
  printf("IDOC       %i\n", nd->nav_time.issue_of_data);
  printf("double T_gd       = %2.30g;\n", nd->nav_time.group_delay);
  printf("double T_oc       = %2.30g;\n", nd->nav_time.reference_time);
  printf("double a_f2       = %2.30g;\n", nd->nav_time.correction_f2);
  printf("double a_f1       = %2.30g;\n", nd->nav_time.correction_f1);
  printf("double a_f0       = %2.30g;\n", nd->nav_time.correction_f0);
  printf("\n");
  
  timestamp = TIME_EPOCH + nd->nav_time.week_num * 604800;
  ts = *localtime(&timestamp);
  strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &ts);
  printf("Epoch is %u (%s)\n", (unsigned)timestamp, buf);
}

/******************************************************************************
* This is called everytime we have a frame of 300 bits, that passes validation
******************************************************************************/
static void nav_save_frame(struct Nav_data *nd) {
  unsigned int unflipped[10];
  unsigned int handover_word;
  int i;
  int frame_type = 0;
  
  /* Key the initial inversion for subframe 0 off the preamble. 
     If the LSB is 0, then it is flipped */
  if(nd->raw_navdata.new_subframe[0] & 1)
    unflipped[0] = nd->raw_navdata.new_subframe[0];
  else
    unflipped[0] = ~nd->raw_navdata.new_subframe[0];

  /* The next 9 depend on the value of bit 0 of the previous subframe */
  for(i = 1; i < 10; i++) {
    if(nd->raw_navdata.new_subframe[i-1] & 1)
      unflipped[i] = ~nd->raw_navdata.new_subframe[i];
    else
      unflipped[i] = nd->raw_navdata.new_subframe[i];   
  }

  /* Handover word is in subframe 1 of every frame. It includes 
     the time of start for the NEXT frame. It gets held in 
     next_time_of_week till the end of frame occurs */
  handover_word = unflipped[1];
  nd->raw_navdata.subframe_of_week = (handover_word >> 13) & 0x1FFFF; 
  nd->subframe_of_week             = (handover_word >> 13) & 0x1FFFF; 
  nd->ms_of_frame = 0;

  {
  char buf[100];
  time_t timestamp;
  struct tm  ts;

  timestamp = TIME_EPOCH + nd->nav_time.week_num * 604800 + nd->raw_navdata.subframe_of_week * 6;
  ts = *localtime(&timestamp);
  strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &ts);

#if 0  
  printf("Time of next frame %i %s\n",nd->raw_navdata.subframe_of_week, buf);
#endif
  /* Handover word also includes the subframe type */
  frame_type = (handover_word >>  8) & 0x7;
  }

#if 0
  /* Now Save the required frames for later */
  printf("Frame type is %i: ",frame_type);
  for(i=0; i < 10; i++)
    printf(" %08X", unflipped[i]);
  printf("\n");
#endif

  if(frame_type > 0 && frame_type < 6) {
      if(nd->nav_file == NULL) {
          char name[100];
          sprintf(name, "NAV_%02i.dat",nd->sv_id);
          nd->nav_file = fopen(name,"r+b");
          if(nd->nav_file == NULL) {
             nd->nav_file = fopen(name,"wb");
          } 
          if(nd->nav_file == NULL) {
              printf("Unable to open NAV file '%s'\n",name);
          }
      }
      if(nd->nav_file != NULL) {
          fseek(nd->nav_file, sizeof(nd->raw_navdata.new_subframe)*(frame_type-1), SEEK_SET);
          fwrite(nd->raw_navdata.new_subframe,sizeof(nd->raw_navdata.new_subframe),1,nd->nav_file);
          fflush(nd->nav_file);
      }
  }
      
  if(frame_type == 1) {
    nd->raw_navdata.valid_subframe[1] = 1;
      for(i = 0; i < 10; i++)
         nd->raw_navdata.subframes[1][i] = unflipped[i];
  
      nd->nav_time.week_num              = join_bits_u(0,           0, 0,nd->raw_navdata.subframes[1][2], 20,10);
      /* Week 524+1024 is sometime in late 2010. This will work for about 20 years after that */
      if(nd->nav_time.week_num < 524) {
        nd->nav_time.week_num += 1024*2;
      } else {
        nd->nav_time.week_num += 1024;
      }
      
      nd->nav_time.user_range_accuracy = join_bits_u(0,           0, 0, nd->raw_navdata.subframes[1][2], 14, 4);
      nd->nav_time.health              = join_bits_u(0,           0, 0, nd->raw_navdata.subframes[1][2],  8, 6);
      nd->nav_time.group_delay         = join_bits_s(0,           0, 0, nd->raw_navdata.subframes[1][6],  6, 8) * pow(2.0, -31.0);
      nd->nav_time.issue_of_data       = join_bits_u(nd->raw_navdata.subframes[1][2],6, 2, nd->raw_navdata.subframes[1][7], 22, 8);
      nd->nav_time.reference_time      = join_bits_u(0,           0, 0, nd->raw_navdata.subframes[1][7],  6,16) * pow(2.0, 4.0);
      nd->nav_time.correction_f2       = join_bits_s(0,           0, 0, nd->raw_navdata.subframes[1][8], 22, 8) * pow(2.0, -55.0);
      nd->nav_time.correction_f1       = join_bits_s(0,           0, 0, nd->raw_navdata.subframes[1][8],  6,16) * pow(2.0, -43.0);
      nd->nav_time.correction_f0       = join_bits_s(0,           0, 0, nd->raw_navdata.subframes[1][9],  8,22) * pow(2.0, -31.0);
      debug_print_time(nd);
      nd->nav_time.time_good = 1;
  } else if(frame_type == 2 || frame_type == 3) {  
    unsigned iode2, iode3; 
    /* Frame 2 and 3 share common processsing */
    if(frame_type == 2) {
      nd->raw_navdata.valid_subframe[2] = 1;
      for(i = 0; i < 10; i++)
        nd->raw_navdata.subframes[2][i] = unflipped[i];
    } else {
      nd->raw_navdata.valid_subframe[3] = 1;
      for(i = 0; i < 10; i++)
        nd->raw_navdata.subframes[3][i] = unflipped[i];
    }

    /**************************************************
    * First, check that both frames have the same Issue
    * Of Data Ephemeris values, i.e. they are havles of
    * the same data set, and that we have not already 
    * extracted the orbit parameters
    **************************************************/
    iode2               = join_bits_u(0,            0, 0, nd->raw_navdata.subframes[2][2], 22, 8);
    iode3               = join_bits_u(0,            0, 0, nd->raw_navdata.subframes[3][9], 22, 8);
    if(iode2 == iode3 && iode2 != nd->nav_orbit.iode) {
      /* Great! We can start extracting the fields out of the frames */
      /****************************************************
      * At most, fields are split over two subframes - so
      * we can extract them all the same, consistent way
      ****************************************************/
      /******** From FRAME 2 *******/
      nd->nav_orbit.iode        = iode2;
      nd->nav_orbit.Crs                      = join_bits_s(0,            0, 0, nd->raw_navdata.subframes[2][2],  6,16) * pow(2.0,-5.0);
      nd->nav_orbit.delta_n                  = join_bits_s(0,            0, 0, nd->raw_navdata.subframes[2][3], 14,16) * pow(2.0,-43.0) * PI;
      nd->nav_orbit.mean_motion_at_ephemeris = join_bits_s(nd->raw_navdata.subframes[2][3], 6, 8, nd->raw_navdata.subframes[2][4],  6,24) * pow(2.0,-31.0) * PI;
      nd->nav_orbit.Cuc                      = join_bits_s(0,            0, 0, nd->raw_navdata.subframes[2][5], 14,16) * pow(2.0,-29.0);
      nd->nav_orbit.e                        = join_bits_u(nd->raw_navdata.subframes[2][5], 6, 8, nd->raw_navdata.subframes[2][6],  6,24) * pow(2.0,-33.0);
      nd->nav_orbit.Cus                      = join_bits_s(0,            0, 0, nd->raw_navdata.subframes[2][7], 14,16) * pow(2.0,-29.0);
      nd->nav_orbit.sqrt_A                   = join_bits_u(nd->raw_navdata.subframes[2][7], 6, 8, nd->raw_navdata.subframes[2][8],  6,24) * pow(2.0,-19.0);
      nd->nav_orbit.time_of_ephemeris        = join_bits_u(0,            0, 0, nd->raw_navdata.subframes[2][9], 14,16) * 16.0;
      nd->nav_orbit.fit_flag                 = join_bits_u(0,            0, 0, nd->raw_navdata.subframes[2][9], 13, 1);
      nd->nav_orbit.aodo                     = join_bits_u(0,            0, 0, nd->raw_navdata.subframes[2][9],  8, 6);

      /******** From FRAME 3 *******/
      nd->nav_orbit.Cic                      = join_bits_s(0,            0, 0, nd->raw_navdata.subframes[3][2], 14,16) * pow(2.0,-29.0);
      nd->nav_orbit.omega_0                  = join_bits_s(nd->raw_navdata.subframes[3][2], 6, 8, nd->raw_navdata.subframes[3][3],  6,24) * pow(2.0,-31.0) * PI;
      nd->nav_orbit.Cis                      = join_bits_s(0,            0, 0, nd->raw_navdata.subframes[3][4], 14,16) * pow(2.0,-29.0);
      nd->nav_orbit.inclination_at_ephemeris = join_bits_s(nd->raw_navdata.subframes[3][4], 6, 8, nd->raw_navdata.subframes[3][5],  6,24) * pow(2.0,-31.0) * PI;
      nd->nav_orbit.Crc                      = join_bits_s(0,            0, 0, nd->raw_navdata.subframes[3][6], 14,16) * pow(2.0,-5.0);
      nd->nav_orbit.w                        = join_bits_s(nd->raw_navdata.subframes[3][6], 6, 8, nd->raw_navdata.subframes[3][7],  6,24) * pow(2.0,-31.0) * PI;
      nd->nav_orbit.omega_dot                = join_bits_s(0,            0, 0, nd->raw_navdata.subframes[3][8],  6,24) * pow(2.0,-43.0) * PI;
      nd->nav_orbit.idot                     = join_bits_s(0,            0, 0, nd->raw_navdata.subframes[3][9],  8,14) * pow(2.0,-43.0) * PI;
      nd->nav_orbit.orbit_valid = 1;
      debug_print_orbit(nd);
    }
  } else if(frame_type == 4) {
      nd->raw_navdata.valid_subframe[4] = 1;
    /* Frame not used - holds almanac data */
  } else if(frame_type == 5) {
      nd->raw_navdata.valid_subframe[5] = 1;
    /* Frame not used - holds almanac data */
  } else {
    printf("Invalid subframe type\n");
  }
}

/******************************************************************************
* Read in any cached NAV data for one Space Vehicles. This will allow results 
* quicker as we don't have to wait for all the orbit info to be received
******************************************************************************/
static int nav_read_in_cached_data(struct Nav_data *nd) {
    FILE *f;
    char name[22];

    sprintf(name, "NAV_%02i.dat",nd->sv_id);
    f = fopen(name,"r+");
    if(f == NULL) {
        printf("Unable to open NAV file '%s'\n",name);
        return 0;
    }

    while(fread(nd->raw_navdata.new_subframe,40,1,f) == 1) {
        printf("Read in subframe\n");
        nav_save_frame(nd);
    }
    fclose(f);
    /* Reset the time_good flag, as the frame_of_week will be wrong */
    nd->nav_time.time_good = 0;
    return 1;
}

/******************************************************************************
* Read in any cached NAV data for all Space Vehicles. This will allow results
* quicker as we don't have to wait for all the orbit info to be received
******************************************************************************/
static void nav_read_in_all_cached_data(void) {
    int i;
    for(i = 0; i < MAX_SV+1; i++) {
        nav_read_in_cached_data(nav_data+i);
    }
}

/******************************************************************************
* Determine if the last 32 bits holds a valid preamble value (tests not only
* bits 29 to 24 of the current frame, but also bit 30, which is the last bit
* of the previous frame, indicating if the following subframe is flipped)
******************************************************************************/
static int nav_test_telemetry(struct Nav_data *nd) {
    /* TODO - mask should be 0x7FC00000A, and the first test should be against 0x5D000000 */
    
    unsigned int temp = nd->raw_navdata.new_word & 0x7FC00000;

    /* Test the first 8 bits for for the preamble, bur also check the 
    * last bit of the previous frame to see if this one is inverted  */
    if(temp == 0x5D000000)
        return 1;
    if(temp == 0x22C00000)
        return 1;
    return 0;
}
    
/************************************************************
* Does new_word hold a valid NAV subframe? 
***********************************************************/
static int nav_valid_subframes(struct Nav_data *nd) {
    if(!nav_test_parity(nd->raw_navdata.new_word)) {
#if SHOW_NAV_FRAMING
        printf("%02i:%5i: Parity fail\n", sv->id, sv->navdata.seq);
#endif
        return 0;
    }

    if(!nd->raw_navdata.synced) {
        if(!nav_test_telemetry(nd)) {
#if SHOW_NAV_FRAMING
            printf("%02i:%5i: Telemetry fail 0x%08X\n", sv->id, sv->navdata.seq, sv->navdata.new_word );
#endif
            return 0;
        }
#if SHOW_NAV_FRAMING
        printf("%02i:%5i: Valid telemetry 0x%08X - synced\n", sv->id, sv->navdata.seq, sv->navdata.new_word );
#endif
        nd->raw_navdata.new_subframe[nd->raw_navdata.subframe_in_frame] = nd->raw_navdata.new_word;
        nd->raw_navdata.synced      = 1;
        nd->raw_navdata.subframe_in_frame = 1;
        nd->ms_of_frame      = 600;
    } else {
        if(nd->raw_navdata.subframe_in_frame == 0) {
            if(!nav_test_telemetry(nd)) {
#if SHOW_NAV_FRAMING
                printf("%02i:%5i: Telemetry fail 0x%08X\n", sv->id, sv->navdata.seq, sv->navdata.new_word );
#endif
                nd->raw_navdata.synced = 0;
                return 0;
            }
#if SHOW_NAV_FRAMING
            printf("%02i:%5i: Valid telemetry\n", sv->id, sv->navdata.seq);
#endif
            nd->raw_navdata.new_subframe[nd->raw_navdata.subframe_in_frame] = nd->raw_navdata.new_word;
            nd->raw_navdata.subframe_in_frame = 1;
        } else {
#if SHOW_NAV_FRAMING
            printf("%02i:%5i:    Subframe %i\n", sv->navdata.seq, sv->id, sv->navdata.subframe_in_frame);
#endif
            nd->raw_navdata.new_subframe[nd->raw_navdata.subframe_in_frame] = nd->raw_navdata.new_word;
            if(nd->raw_navdata.subframe_in_frame  == 9) {
                nav_save_frame(nd);
                nd->raw_navdata.subframe_in_frame = 0;
            } else {
                nd->raw_navdata.subframe_in_frame++;
            }
        }
    }
    return 1;
}

/************************************************************************
* Receive a new bit of NAV data
************************************************************************/
static void nav_new_bit(struct Nav_data *nd, uint_8 s) {
    /* Shift in the next bit */
    nd->raw_navdata.new_word <<= 1;
    if(s) nd->raw_navdata.new_word |= 1;

    nd->raw_navdata.valid_bits++;
    if(nd->raw_navdata.valid_bits == 32) {
        if(nav_valid_subframes(nd)) {
            nd->raw_navdata.valid_bits = 2;
        } else {
            nd->raw_navdata.valid_bits = 31;
        }
    }
    nd->raw_navdata.seq++;
}

/*************************************************************************
* Restart the reception of NAV data. This happens if we have an
* unexpected BPSK phase flip
*************************************************************************/
static void nav_abandon(struct Nav_data *nd) {
//    printf("%2i: Abandon - %5i errors\n", nd->sv_id, nd->raw_navdata.bit_errors);
    nd->raw_navdata.valid_bits = 0;
    nd->raw_navdata.synced = 0;
    nd->raw_navdata.bit_errors++;
}

/*************************************************************************
*
*************************************************************************/
int nav_get_bit_errors_count(int sv_id) {
  if(sv_id <1 || sv_id > 32)
    return -1;
  return nav_data[sv_id].raw_navdata.bit_errors;
} 
/*************************************************************************
*
*************************************************************************/
int nav_clear_bit_errors_count(int sv_id) {
  if(sv_id <1 || sv_id > 32)
    return -1;
  nav_data[sv_id].raw_navdata.bit_errors = 0;
  return 1;
} 
/*************************************************************************
* We have a new MS of signal. Work towards decoding a bit of BPSH data
*************************************************************************/
static void nav_process(struct Nav_data *nd, uint_8 s) {
    if(nd->raw_navdata.part_in_bit == BIT_LENGTH-1) {
        if(nd->raw_navdata.bit_errors>0)
            nd->raw_navdata.bit_errors--;
        nd->raw_navdata.part_in_bit = 0;
    } else if(s != (nd->raw_navdata.new_word&1)) {
        nav_abandon(nd);
        nd->raw_navdata.part_in_bit = 0;
    } else
        nd->raw_navdata.part_in_bit++;
    
    if(nd->raw_navdata.part_in_bit == 0) {
        nav_new_bit(nd,s);
    }
}

/*************************************************
*                                             
*************************************************/
void nav_add_bit(int sv, int power) {
  struct Nav_data *nd;

  if(sv < 0 || sv > MAX_SV)
    return;
  nd = nav_data + sv;

  /* Update the counters so we can reinitialise
     if we need to while processing the power infor */
  if(nd->ms_of_frame == 5999) {
     if(nd->subframe_of_week != -1)
       nd->subframe_of_week++;
     nd->ms_of_frame = 0;
  } else
    nd->ms_of_frame++;

  nav_process(nd, power > 0 ? 1: 0);
}

/*************************************************
*                                             
*************************************************/
int nav_startup(void) {
  int i;
  for(i = 0; i <= MAX_SV; i++) {
    nav_data[i].sv_id             =  i;
    nav_data[i].nav_time.week_num = -1;
    nav_data[i].subframe_of_week  = -1;
    nav_data[i].ms_of_frame       = -1;
  } 
  nav_read_in_all_cached_data();
  return 1;
}
