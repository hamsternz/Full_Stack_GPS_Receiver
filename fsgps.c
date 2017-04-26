/****************************************************************************
* fsgps.c - A 'full stack' GPS recevier - goes from raw 
*           samples to a position fix
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

/*******************************************************************
* What information should be printed, and how often
*******************************************************************/
/* How often should the table of channel states be printed */
#define SHOW_TIMING_SNAPSHOT_FREQ_MS     (20)

/* How often should an attempt to solve for position be made */
#define SHOW_SOLUTION_MS              (20)

/*******************************************************************
* Constants based on the GPS signals for decoding BPSK data
*******************************************************************/
/* How many 'chips' are sent each milliseconds */
#define CHIPS_PER_MS              (1023)
/* Each BPSH bit is MS_PER_BIT milliseconds long */
#define MS_PER_BIT                (20)
#define BIT_LENGTH                (MS_PER_BIT)

/* There are BITS_PER_FRAME bits in each NAV frame */
#define BITS_PER_FRAME            (300)

/* How many unexpected phase transitions will we receive
   until we consider the signal lost */
#define MAX_BAD_BIT_TRANSITIONS   (500)


/*******************************************************************
* Factors that control the feedback loops for tracking the signals
*******************************************************************/
/* The factor used to smooth the early and late power levels */
#define LATE_EARLY_IIR_FACTOR      16

/* Filter constants for the angle and change in angle 
* used for carrier locking */
#define LOCK_DELTA_IIR_FACTOR       8
#define LOCK_ANGLE_IIR_FACTOR       8

/*******************************************************************
* To print out debugging information
*******************************************************************/
/* Print the ATAN2 lookup table as it is generated */
#define PRINT_ATAN2_TABLE           0


/* Set PRINT_SAMPLE_NUMBERS to 1 to Print out the number
*  of samples processed every PRINT_SAMPLE_FREQUENCY */
#define PRINT_SAMPLE_NUMBERS        0
#define PRINT_SAMPLE_FREQUENCY   1000

/* Set PRINT_ACQUIRE_POWERS to 1 to see the power levels 
*  during the acqusition phase. Powers less than
*  PRINT_ACQUIRE_SQUETCH are not printed */
#define PRINT_ACQUIRE_POWERS        0
#define PRINT_ACQUIRE_SQUETCH   50000

/* Set PRINT_TRACK_POWER to '1' to print out the 
* power levels during the track phase */
#define PRINT_TRACK_POWER           0
#define PRINT_TRACK_SQUETCH         0

/* Print out if a space vheicle falls out of lock */
#define SHOW_LOCK_UNLOCK        1

/* Do we show the code NCO values? */
#define PRINT_LOCKED_NCO_VALUES     0

/* Show the initial values used for the NCOs when a lock
*  is obtained */
#define LOCK_SHOW_INITIAL           1

/* Show the power levels each millisecond */
#define LOCK_SHOW_PER_MS_POWER      0

/* Show each bit as it is received */
#define LOCK_SHOW_PER_BIT           0

/* Show the state of the late/early filters */
#define LOCK_SHOW_EARLY_LATE_TREND  0

/* Show the I+Q power levels each millisecond */
#define LOCK_SHOW_PER_MS_IQ         0

/* Show the I+Q vector each millisecond */
#define LOCK_SHOW_ANGLES            0

/* Show each BSPK Bit is it is received */
#define LOCK_SHOW_BITS              0
#define LOCK_SHOW_BPSK_PHASE_PER_MS 0
#define SHOW_NAV_FRAMING            0

/* Do we want to print out each timing snapshot */
#define SHOW_TIMING_SNAPSHOT_DETAILS      1

/* Do we want to write the snapshots to a file, for later analysis? */
#define LOG_TIMING_SNAPSHOT_TO_FILE      1
#define LOG_POSITION_FIX_TO_FILE         1

/* Add extra checks to make sure nothing is awry */
#define DOUBLECHECK_PROMPT_CODE_INDEX    0
/*************************************************************/
/******************** END OF DEFINES *************************/
/*************************************************************/

/* Standard data types */
typedef int                int_32;
typedef unsigned int       uint_32;
typedef unsigned char      uint_8;
typedef signed char        int_8;
typedef unsigned long long uint_64;

uint_32 samples_per_ms;
uint_32 if_cycles_per_ms;
uint_32 samples_for_acquire;
uint_32 acquire_bitmap_size_u32;
uint_32 code_offset_in_ms;
uint_32 ms_for_acquire = 2;
uint_32 acquire_min_power;
uint_32 track_unlock_power;
uint_32 lock_lost_power;
uint_64 sample_count = 0;

enum e_state {state_acquiring, state_tracking, state_locked};

FILE *snapshot_file;
FILE *position_file;
/***************************************************************************
* Physical constants and others defined in the GPS specifications
***************************************************************************/
static const double EARTHS_RADIUS  =   6317000.0;
static const double SPEED_OF_LIGHT = 299792458.0;
static const double PI             = 3.1415926535898;
static const double mu             = 3.986005e14;      /* Earth's universal gravitation parameter */
static const double omegaDot_e     = 7.2921151467e-5;  /* Earth's rotation (radians per second) */
static const int    TIME_EPOCH     = 315964800;

/**************************************************************************/
/* time/clock data received from the Space Vehicles */
struct Nav_time {
  uint_8   time_good;
  uint_32  week_no;
  uint_32  user_range_accuracy;
  uint_32  health;
  uint_32  issue_of_data;
  double   group_delay;    /* Tgd */
  double   reference_time; /* Toc */
  double   correction_f2;  /* a_f2 */
  double   correction_f1;  /* a_f1 */
  double   correction_f0;  /* a_f0 */
};

/* Orbit parameters recevied from the Space Vehicles */
struct Nav_orbit {
  /* Orbit data */
  uint_8  fit_flag;
  uint_8  orbit_valid;
  uint_32 iode; 
  uint_32 time_of_ephemeris;   // Toe
  uint_32 aodo;
  double  mean_motion_at_ephemeris;    // M0
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
  double  inclination_at_ephemeris;    // i_0
  double  w;

};

/* Data used during the acquire phase */
struct Acquire {
  uint_32 *gold_code_stretched;
  uint_32 **seek_in_phase;
  uint_32 **seek_quadrature;

  uint_32 max_power;
  uint_8  max_band;
  uint_32 max_offset;
};

/* Data used during the tracking phase */
struct Track {
  uint_8  percent_locked;
  uint_8  band;
  uint_32 offset; 
  uint_32 power[3][3];
  uint_32 max_power;
  uint_8  max_band;
  uint_32 max_offset;
};

/* Data used once the signal is locked */
struct Lock {
  uint_32 filtered_power;
  uint_32 phase_nco_sine;
  uint_32 phase_nco_cosine;
  int_32  phase_nco_step;

  uint_32 code_nco_step;
  uint_32 code_nco_filter;
  uint_32 code_nco;
  int_32  code_nco_trend;
  
  int_32  early_sine;
  int_32  early_cosine;
  uint_32 early_sine_count;  
  uint_32 early_cosine_count;  
  uint_32 early_sine_count_last;  
  uint_32 early_cosine_count_last;  

  int_32  prompt_sine;
  int_32  prompt_cosine;
  uint_32 prompt_sine_count;  
  uint_32 prompt_cosine_count;  
  uint_32 prompt_sine_count_last;  
  uint_32 prompt_cosine_count_last;  

  int_32  late_sine;
  int_32  late_cosine;
  uint_32 late_sine_count;  
  uint_32 late_cosine_count;  
  uint_32 late_sine_count_last;  
  uint_32 late_cosine_count_last;  
  
  int_32 early_power;
  int_32 early_power_not_reset;
  int_32 prompt_power;
  int_32 late_power;
  int_32 late_power_not_reset;
  uint_8 last_angle;
  int_32 delta_filtered;
  int_32 angle_filtered;
  uint_8 ms_of_bit;
  int_32 ms_of_frame;
  uint_8 last_bit;
};

struct Navdata {
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
  uint_32 subframes[7][10];  
};
  


struct Space_vehicle {
  /* ID of the space vehicle, and the taps used to generate the Gold Codes */
  uint_8 id;
  uint_8 tap1;
  uint_8 tap2;

  enum   e_state state;
  
  uint_8 gold_code[CHIPS_PER_MS];

  /* For holding data in each state */
  struct Acquire acquire;
  struct Track track;
  struct Lock lock; 
  struct Navdata navdata;
  struct Nav_time  nav_time;
  struct Nav_orbit nav_orbit;
  /* For calculating Space vehicle position */
  double time_raw;
  double pos_x;
  double pos_y;
  double pos_z;
  double pos_t;
  double pos_t_valid;

  /* Handles for writing data out */
  FILE *bits_file; 
  FILE *nav_file;
} space_vehicles[] = {
  { 1,  2, 6},
  { 2,  3, 7},
  { 3,  4, 8},
  { 4,  5, 9},
  { 5,  1, 9},
  { 6,  2,10},
  { 7,  1, 8},
  { 8,  2, 9},
  { 9,  3,10},
  {10,  2, 3},
  {11,  3, 4},
  {12,  5, 6},
  {13,  6, 7},
  {14,  7, 8},
  {15,  8, 9},
  {16,  9,10},
  {17,  1, 4},
  {18,  2, 5},
  {19,  3, 6},
  {20,  4, 7},
  {21,  5, 8}, 
  {22,  6, 9},
  {23,  1, 3},
  {24,  4, 6},
  {25,  5, 7}, 
  {26,  6, 8},
  {27,  7, 9},
  {28,  8,10},
  {29,  1, 6},
  {30,  2, 7},
  {31,  3, 8},
  {32,  4, 9}
};

#define N_SV (sizeof(space_vehicles)/sizeof(struct Space_vehicle))
#define MAX_SV 32

unsigned bitmap_size_u32;
unsigned search_bands;
unsigned band_bandwidth;
unsigned *sample_history;
unsigned *work_buffer;

#define ATAN2_SIZE 128
uint_8 atan2_lookup[ATAN2_SIZE][ATAN2_SIZE];

struct Location {
  double x;
  double y;
  double z;
  double time;
};

struct Snapshot_entry {
    uint_32 nav_week_no;
    uint_32 nav_subframe_of_week;
    uint_32 nav_subframe_in_frame;
    uint_32 lock_code_nco;
    uint_32 lock_ms_of_frame;
    uint_8  lock_ms_of_bit;
    uint_8  nav_valid_bits;
    uint_8  state;
    uint_8  id;
};
#define SNAPSHOT_STATE_ORBIT_VALID  (0x01)
#define SNAPSHOT_STATE_TIME_VALID   (0x02)
#define SNAPSHOT_STATE_ACQUIRE      (0x80)
#define SNAPSHOT_STATE_TRACK        (0x40)
#define SNAPSHOT_STATE_LOCKED       (0x20)

struct Snapshot {
    uint_32 sample_count_l;
    uint_32 sample_count_h;
    struct Snapshot_entry entries[MAX_SV];
};

/*********************************************************************************************************/
/*********************************************************************************************************/
/************************ THIS CODE IS TAKEN FROM ANOTHER SOURCE *****************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
//////////////////////////////////////////////////////////////////////////
// Homemade GPS Receiver
// Copyright (C) 2013 Andrew Holme
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// http://www.aholme.co.uk/GPS/Main.htm
//////////////////////////////////////////////////////////////////////////

#include <memory.h>

#define MAX_ITER 20

#define WGS84_A     (6378137.0)
#define WGS84_F_INV (298.257223563)
#define WGS84_B     (6356752.31424518)
#define WGS84_E2    (0.00669437999014132)
#define OMEGA_E     (7.2921151467e-5)  /* Earth's rotation rate */

///////////////////////////////////////////////////////////////////////////////////////////////

#define NUM_CHANS 10

int A_solve(int chans, struct Location *sv_l, struct Location *p) {
    int i, j, r, c;

    double t_tx[NUM_CHANS]; // Clock replicas in seconds since start of week

    double x_sv[NUM_CHANS],
           y_sv[NUM_CHANS],
           z_sv[NUM_CHANS];
  
    double t_pc;  // Uncorrected system time when clock replica snapshots taken
    double t_rx;    // Corrected GPS time

    double dPR[NUM_CHANS]; // Pseudo range error

    double jac[NUM_CHANS][4], ma[4][4], mb[4][4], mc[4][NUM_CHANS], md[4];

    double weight[NUM_CHANS];

    p->x = p->y = p->z = p->time = t_pc = 0.0;

    for (i=0; i<chans && i < NUM_CHANS; i++) {
        weight[i] = 1.0;
        x_sv[i]   = sv_l[i].x;
        y_sv[i]   = sv_l[i].y;
        z_sv[i]   = sv_l[i].z;
        t_tx[i]   = sv_l[i].time;
        t_pc     += sv_l[i].time;
    }

    // Approximate starting value for receiver clock
    t_pc = t_pc/chans + 75e-3;

    // Iterate to user xyzt solution using Taylor Series expansion:
  double err_mag;
    for(j=0; j<MAX_ITER; j++) {
        t_rx = t_pc - p->time;
        for (i=0; i<chans; i++) {
            // Convert SV position to ECI coords (20.3.3.4.3.3.2)
            double theta = (t_tx[i] - t_rx) * OMEGA_E;

            double x_sv_eci = x_sv[i]*cos(theta) - y_sv[i]*sin(theta);
            double y_sv_eci = x_sv[i]*sin(theta) + y_sv[i]*cos(theta);
            double z_sv_eci = z_sv[i];

            // Geometric range (20.3.3.4.3.4)
            double gr = sqrt(pow(p->x - x_sv_eci, 2) +
                             pow(p->y - y_sv_eci, 2) +
                             pow(p->z - z_sv_eci, 2));

            dPR[i] = SPEED_OF_LIGHT*(t_rx - t_tx[i]) - gr;

            jac[i][0] = (p->x - x_sv_eci) / gr;
            jac[i][1] = (p->y - y_sv_eci) / gr;
            jac[i][2] = (p->z - z_sv_eci) / gr;
            jac[i][3] = SPEED_OF_LIGHT;
        }

        // ma = transpose(H) * W * H
        for (r=0; r<4; r++)
            for (c=0; c<4; c++) {
            ma[r][c] = 0;
            for (i=0; i<chans; i++) ma[r][c] += jac[i][r]*weight[i]*jac[i][c];
        }

        double determinant =
            ma[0][3]*ma[1][2]*ma[2][1]*ma[3][0] - ma[0][2]*ma[1][3]*ma[2][1]*ma[3][0] - ma[0][3]*ma[1][1]*ma[2][2]*ma[3][0] + ma[0][1]*ma[1][3]*ma[2][2]*ma[3][0]+
            ma[0][2]*ma[1][1]*ma[2][3]*ma[3][0] - ma[0][1]*ma[1][2]*ma[2][3]*ma[3][0] - ma[0][3]*ma[1][2]*ma[2][0]*ma[3][1] + ma[0][2]*ma[1][3]*ma[2][0]*ma[3][1]+
            ma[0][3]*ma[1][0]*ma[2][2]*ma[3][1] - ma[0][0]*ma[1][3]*ma[2][2]*ma[3][1] - ma[0][2]*ma[1][0]*ma[2][3]*ma[3][1] + ma[0][0]*ma[1][2]*ma[2][3]*ma[3][1]+
            ma[0][3]*ma[1][1]*ma[2][0]*ma[3][2] - ma[0][1]*ma[1][3]*ma[2][0]*ma[3][2] - ma[0][3]*ma[1][0]*ma[2][1]*ma[3][2] + ma[0][0]*ma[1][3]*ma[2][1]*ma[3][2]+
            ma[0][1]*ma[1][0]*ma[2][3]*ma[3][2] - ma[0][0]*ma[1][1]*ma[2][3]*ma[3][2] - ma[0][2]*ma[1][1]*ma[2][0]*ma[3][3] + ma[0][1]*ma[1][2]*ma[2][0]*ma[3][3]+
            ma[0][2]*ma[1][0]*ma[2][1]*ma[3][3] - ma[0][0]*ma[1][2]*ma[2][1]*ma[3][3] - ma[0][1]*ma[1][0]*ma[2][2]*ma[3][3] + ma[0][0]*ma[1][1]*ma[2][2]*ma[3][3];

        // mb = inverse(ma) = inverse(transpose(H)*W*H)
        mb[0][0] = (ma[1][2]*ma[2][3]*ma[3][1] - ma[1][3]*ma[2][2]*ma[3][1] + ma[1][3]*ma[2][1]*ma[3][2] - ma[1][1]*ma[2][3]*ma[3][2] - ma[1][2]*ma[2][1]*ma[3][3] + ma[1][1]*ma[2][2]*ma[3][3]) / determinant;
        mb[0][1] = (ma[0][3]*ma[2][2]*ma[3][1] - ma[0][2]*ma[2][3]*ma[3][1] - ma[0][3]*ma[2][1]*ma[3][2] + ma[0][1]*ma[2][3]*ma[3][2] + ma[0][2]*ma[2][1]*ma[3][3] - ma[0][1]*ma[2][2]*ma[3][3]) / determinant;
        mb[0][2] = (ma[0][2]*ma[1][3]*ma[3][1] - ma[0][3]*ma[1][2]*ma[3][1] + ma[0][3]*ma[1][1]*ma[3][2] - ma[0][1]*ma[1][3]*ma[3][2] - ma[0][2]*ma[1][1]*ma[3][3] + ma[0][1]*ma[1][2]*ma[3][3]) / determinant;
        mb[0][3] = (ma[0][3]*ma[1][2]*ma[2][1] - ma[0][2]*ma[1][3]*ma[2][1] - ma[0][3]*ma[1][1]*ma[2][2] + ma[0][1]*ma[1][3]*ma[2][2] + ma[0][2]*ma[1][1]*ma[2][3] - ma[0][1]*ma[1][2]*ma[2][3]) / determinant;
        mb[1][0] = (ma[1][3]*ma[2][2]*ma[3][0] - ma[1][2]*ma[2][3]*ma[3][0] - ma[1][3]*ma[2][0]*ma[3][2] + ma[1][0]*ma[2][3]*ma[3][2] + ma[1][2]*ma[2][0]*ma[3][3] - ma[1][0]*ma[2][2]*ma[3][3]) / determinant;
        mb[1][1] = (ma[0][2]*ma[2][3]*ma[3][0] - ma[0][3]*ma[2][2]*ma[3][0] + ma[0][3]*ma[2][0]*ma[3][2] - ma[0][0]*ma[2][3]*ma[3][2] - ma[0][2]*ma[2][0]*ma[3][3] + ma[0][0]*ma[2][2]*ma[3][3]) / determinant;
        mb[1][2] = (ma[0][3]*ma[1][2]*ma[3][0] - ma[0][2]*ma[1][3]*ma[3][0] - ma[0][3]*ma[1][0]*ma[3][2] + ma[0][0]*ma[1][3]*ma[3][2] + ma[0][2]*ma[1][0]*ma[3][3] - ma[0][0]*ma[1][2]*ma[3][3]) / determinant;
        mb[1][3] = (ma[0][2]*ma[1][3]*ma[2][0] - ma[0][3]*ma[1][2]*ma[2][0] + ma[0][3]*ma[1][0]*ma[2][2] - ma[0][0]*ma[1][3]*ma[2][2] - ma[0][2]*ma[1][0]*ma[2][3] + ma[0][0]*ma[1][2]*ma[2][3]) / determinant;
        mb[2][0] = (ma[1][1]*ma[2][3]*ma[3][0] - ma[1][3]*ma[2][1]*ma[3][0] + ma[1][3]*ma[2][0]*ma[3][1] - ma[1][0]*ma[2][3]*ma[3][1] - ma[1][1]*ma[2][0]*ma[3][3] + ma[1][0]*ma[2][1]*ma[3][3]) / determinant;
        mb[2][1] = (ma[0][3]*ma[2][1]*ma[3][0] - ma[0][1]*ma[2][3]*ma[3][0] - ma[0][3]*ma[2][0]*ma[3][1] + ma[0][0]*ma[2][3]*ma[3][1] + ma[0][1]*ma[2][0]*ma[3][3] - ma[0][0]*ma[2][1]*ma[3][3]) / determinant;
        mb[2][2] = (ma[0][1]*ma[1][3]*ma[3][0] - ma[0][3]*ma[1][1]*ma[3][0] + ma[0][3]*ma[1][0]*ma[3][1] - ma[0][0]*ma[1][3]*ma[3][1] - ma[0][1]*ma[1][0]*ma[3][3] + ma[0][0]*ma[1][1]*ma[3][3]) / determinant;
        mb[2][3] = (ma[0][3]*ma[1][1]*ma[2][0] - ma[0][1]*ma[1][3]*ma[2][0] - ma[0][3]*ma[1][0]*ma[2][1] + ma[0][0]*ma[1][3]*ma[2][1] + ma[0][1]*ma[1][0]*ma[2][3] - ma[0][0]*ma[1][1]*ma[2][3]) / determinant;
        mb[3][0] = (ma[1][2]*ma[2][1]*ma[3][0] - ma[1][1]*ma[2][2]*ma[3][0] - ma[1][2]*ma[2][0]*ma[3][1] + ma[1][0]*ma[2][2]*ma[3][1] + ma[1][1]*ma[2][0]*ma[3][2] - ma[1][0]*ma[2][1]*ma[3][2]) / determinant;
        mb[3][1] = (ma[0][1]*ma[2][2]*ma[3][0] - ma[0][2]*ma[2][1]*ma[3][0] + ma[0][2]*ma[2][0]*ma[3][1] - ma[0][0]*ma[2][2]*ma[3][1] - ma[0][1]*ma[2][0]*ma[3][2] + ma[0][0]*ma[2][1]*ma[3][2]) / determinant;
        mb[3][2] = (ma[0][2]*ma[1][1]*ma[3][0] - ma[0][1]*ma[1][2]*ma[3][0] - ma[0][2]*ma[1][0]*ma[3][1] + ma[0][0]*ma[1][2]*ma[3][1] + ma[0][1]*ma[1][0]*ma[3][2] - ma[0][0]*ma[1][1]*ma[3][2]) / determinant;
        mb[3][3] = (ma[0][1]*ma[1][2]*ma[2][0] - ma[0][2]*ma[1][1]*ma[2][0] + ma[0][2]*ma[1][0]*ma[2][1] - ma[0][0]*ma[1][2]*ma[2][1] - ma[0][1]*ma[1][0]*ma[2][2] + ma[0][0]*ma[1][1]*ma[2][2]) / determinant;

        // mc = inverse(transpose(H)*W*H) * transpose(H)
        for (r=0; r<4; r++)
            for (c=0; c<chans; c++) {
            mc[r][c] = 0;
            for (i=0; i<4; i++) mc[r][c] += mb[r][i]*jac[c][i];
        }

        // md = inverse(transpose(H)*W*H) * transpose(H) * W * dPR
        for (r=0; r<4; r++) {
            md[r] = 0;
            for (i=0; i<chans; i++) md[r] += mc[r][i]*weight[i]*dPR[i];
        }

        double dx = md[0];
        double dy = md[1];
        double dz = md[2];
        double dt = md[3];

        err_mag = sqrt(dx*dx + dy*dy + dz*dz);


        if (err_mag<1.0) break;

        p->x    += dx;
        p->y    += dy;
        p->z    += dz;
        p->time += dt;
    }
    return j;
}

///////////////////////////////////////////////////////////////////////////////////////////////

static void LatLonAlt(
    double x_n, double y_n, double z_n,
    double *lat, double *lon, double *alt) {
    int iterations = 100;
    const double a  = WGS84_A;
    const double e2 = WGS84_E2;

    const double p = sqrt(x_n*x_n + y_n*y_n);

    *lon = 2.0 * atan2(y_n, x_n + p);
    *lat = atan(z_n / (p * (1.0 - e2)));
    *alt = 0.0;

    while(iterations > 0) {
        double tmp = *alt;
        double N = a / sqrt(1.0 - e2*pow(sin(*lat),2));
        *alt = p/cos(*lat) - N;
        *lat = atan(z_n / (p * (1.0 - e2*N/(N + *alt))));
        if(fabs(*alt-tmp)<1e-3) 
            break;
        iterations--;
    }
}

/*********************************************************************************************************/
/*********************************************************************************************************/
/************************ END OF CODE TAKEN FROM ANOTHER SOURCE ******************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
static double  orbit_ecc_anom(struct Space_vehicle *sv, double t) {
    int    iterations  = 200;
    double delta       = pow(10,-10);
    double estimate, correction, semi_major_axis,     computed_mean_motion;
    double time_from_ephemeris,  correct_mean_motion, mean_anomaly;

    semi_major_axis      = sv->nav_orbit.sqrt_A * sv->nav_orbit.sqrt_A;
    computed_mean_motion = sqrt(mu / pow(semi_major_axis,3.0));

    time_from_ephemeris  = t - sv->nav_orbit.time_of_ephemeris;
    if(time_from_ephemeris >  302400) time_from_ephemeris  -= 604800;
    if(time_from_ephemeris < -302400) time_from_ephemeris  += 604800;
    correct_mean_motion  = computed_mean_motion + sv->nav_orbit.delta_n;

    /* Add on how much we have moved through the orbit since ephemeris */
    mean_anomaly         = sv->nav_orbit.mean_motion_at_ephemeris + correct_mean_motion * time_from_ephemeris;

    /* First estimate */
    estimate   = (sv->nav_orbit.e<0.8) ? mean_anomaly :  PI;
    correction = estimate - (mean_anomaly + sv->nav_orbit.e*sin(mean_anomaly));

    /* Solve iteratively */
    while ((fabs(correction)>delta) && iterations > 0) {
        double last = estimate;
        estimate  = mean_anomaly  + sv->nav_orbit.e * sin(estimate);
        correction = estimate - last;
        iterations--;
    }

    if(iterations == 0) {
        printf("Error calculating Eccentric Anomaly\n");
    }
    return estimate;
}

/*************************************************************************/
static void sv_calc_corrected_time(int i) {
    double delta_t, delta_tr, ek, time_correction;
    struct Space_vehicle *sv;    
    sv = space_vehicles+i;
    sv->pos_t_valid = 0;

    /* Calulate the time for the adjustment */
    delta_t = sv->time_raw - sv->nav_time.reference_time;
    if(delta_t >  302400)  delta_t -= 604800;
    if(delta_t < -302400)  delta_t += 604800;

    /* Relativistic term */
    ek = orbit_ecc_anom(sv, sv->time_raw);

    delta_tr = -4.442807633e-10 * sv->nav_orbit.e * sv->nav_orbit.sqrt_A * sin(ek);

    time_correction = sv->nav_time.correction_f0 
                    + (sv->nav_time.correction_f1 * delta_t) 
                    + (sv->nav_time.correction_f2 * delta_t * delta_t) 
                    + delta_tr
                    - sv->nav_time.group_delay;
    sv->pos_t = sv->time_raw - time_correction;
    sv->pos_t_valid = 1;
    return;

}

/**************************************************************************
* Calculate where the Space Vehicle will be at time "pos_t"
**************************************************************************/
static int orbit_calc_position(struct Space_vehicle *sv, struct Location *l)
{
  double time_from_ephemeris,   semi_major_axis;
  double ek, true_anomaly,      corrected_argument_of_latitude;
  double argument_of_latitude,  argument_of_latitude_correction;
  double radius_correction,     corrected_radius;
  double correction_of_inclination;
  double pos_in_orbial_plane_x, pos_in_orbial_plane_y;
  double corrected_inclination, corrected_angle_of_ascending_node;
  
  /***********************
  * Calculate orbit
  ***********************/
  time_from_ephemeris  = sv->pos_t - sv->nav_orbit.time_of_ephemeris;
  if(time_from_ephemeris >  302400) time_from_ephemeris  -= 604800;
  if(time_from_ephemeris < -302400) time_from_ephemeris  += 604800;

  semi_major_axis      = sv->nav_orbit.sqrt_A * sv->nav_orbit.sqrt_A;
  ek = orbit_ecc_anom(sv, sv->pos_t);

  /* Now calculate the first approximation of the latitude */
  true_anomaly = atan2( sqrt(1-sv->nav_orbit.e * sv->nav_orbit.e) * sin(ek), cos(ek) - sv->nav_orbit.e);
  argument_of_latitude = true_anomaly + sv->nav_orbit.w;

  /*****************************************
  * Second Harmonic Perbturbations 
  *****************************************/
  argument_of_latitude_correction = sv->nav_orbit.Cus * sin(2*argument_of_latitude) 
                                  + sv->nav_orbit.Cuc * cos(2*argument_of_latitude);

  radius_correction               = sv->nav_orbit.Crc * cos(2*argument_of_latitude) 
                                  + sv->nav_orbit.Crs * sin(2*argument_of_latitude);
  
  correction_of_inclination       = sv->nav_orbit.Cic * cos(2*argument_of_latitude) 
                                  + sv->nav_orbit.Cis * sin(2*argument_of_latitude);

  corrected_argument_of_latitude  = argument_of_latitude + argument_of_latitude_correction;
  corrected_radius                = semi_major_axis * (1- sv->nav_orbit.e * cos(ek)) + radius_correction;
  corrected_inclination           = sv->nav_orbit.inclination_at_ephemeris + correction_of_inclination 
                                  + sv->nav_orbit.idot*time_from_ephemeris;

  pos_in_orbial_plane_x = corrected_radius * cos(corrected_argument_of_latitude);
  pos_in_orbial_plane_y = corrected_radius * sin(corrected_argument_of_latitude);
  

  corrected_angle_of_ascending_node = sv->nav_orbit.omega_0
                                    + (sv->nav_orbit.omega_dot - omegaDot_e)*time_from_ephemeris 
                                    - omegaDot_e * sv->nav_orbit.time_of_ephemeris;

  /******************************************************
  * Project into Earth Centered, Earth Fixed coordinates
  ******************************************************/
  l->x = pos_in_orbial_plane_x * cos(corrected_angle_of_ascending_node)
       - pos_in_orbial_plane_y * cos(corrected_inclination) * sin(corrected_angle_of_ascending_node);
  l->y = pos_in_orbial_plane_x * sin(corrected_angle_of_ascending_node) 
       + pos_in_orbial_plane_y * cos(corrected_inclination) * cos(corrected_angle_of_ascending_node);
  l->z = pos_in_orbial_plane_y * sin(corrected_inclination);
  l->time = sv->pos_t;

  return 1;
}
/*************************************************************************/
static int sv_calc_location(int id, struct Location *l)
{  
    l->time = space_vehicles[id].pos_t;
    orbit_calc_position(space_vehicles+id, l);
    return 1;
}
/****************************************************************************/
static void attempt_solution(struct Snapshot *s) {
    int sv_count = 0;
    int sv_ids[N_SV];
    int i;
    struct Location predicted_location;
    struct Location *sv_location;
    double lat,lon,alt;
    int q, valid_locations;
  
    /* First, filter out who we can use in the solution */
    for(i = 0; i < MAX_SV; i++) {
        int id, j;
        /* Find a valid entry in the snapshot */
        if(!(s->entries[i].state & SNAPSHOT_STATE_LOCKED) )      continue;
        if(!(s->entries[i].state & SNAPSHOT_STATE_ORBIT_VALID) ) continue;
        if(!(s->entries[i].state & SNAPSHOT_STATE_TIME_VALID) )  continue;        
        id = s->entries[i].id;
        
        /* Check we can find the matching entry in the Space_vehicle table */
        for(j = 0 ; j < N_SV; j++) {
            if(space_vehicles[j].id == id)
                break;
        }
        /* If not found */
        if(j == N_SV) continue;
        
        /* Yes, we will use this ID */
        sv_ids[sv_count] = id;
        sv_count++;
    }

    /* Not enough vaild data to find the location! */
    if(sv_count < 4) {
        return;
    }
  
    /* Allocate space to hold the space vehicle locations */
    sv_location = malloc(sizeof (struct Location)*sv_count);
    if(sv_location == NULL) {
        printf("Out of memory \n");
        return;
    }
  
    /* Calculate the Space Vehicle positions at time of transmit */
    valid_locations = 0;
    for(q = 0; q < sv_count; q++) {
        int e, sv;
        
        unsigned phase_in_gold_code;  /* value is 0 to (1023*64-1) */
        /* Find the Snapshot entry for this ID */
        for(e = 0; e < MAX_SV; e++) {
            if(s->entries[e].id == sv_ids[q])
                break;
        }
        if(e == MAX_SV) {
            printf("UNEXPECTED! Unable to find snapshot entry for id %02i!\n",sv_ids[q]);
            continue;
        }

        /* Find the Space Vehicle entry for this ID */
        for(sv = 0; sv < N_SV; sv++) {
            if(space_vehicles[sv].id == sv_ids[q])
                break;
        }
        if(sv == N_SV) {
            printf("UNEXPECTED! Unable to find Space Vehicle entry!\n");
            continue;
        }
        
        phase_in_gold_code    = (s->entries[e].lock_code_nco >> 1) & 0x7FFFFFFF;
    
        /* calc transmit time milliseconds  */
        space_vehicles[sv].time_raw   = s->entries[e].nav_subframe_of_week * 6000 + s->entries[e].lock_ms_of_frame
                                      + ((double)phase_in_gold_code)/(1023*(1<<21));
        /* Convert from milliseconds to seconds */
        space_vehicles[sv].time_raw  /= 1000.0;                                    
        
        /* Correct the time using calibration factors */ 
        space_vehicles[sv].pos_t_valid  = 0;
        sv_calc_corrected_time(sv);
        /* calculate the location of the space vehicle */
        sv_calc_location(sv, sv_location+valid_locations);
        
        /* Verify that the location is valid */
        if(sv_location[valid_locations].x < 40000000 && sv_location[valid_locations].x > -40000000) 
            if(sv_location[valid_locations].y < 40000000 && sv_location[valid_locations].y > -40000000)
                if(sv_location[valid_locations].z < 40000000 && sv_location[valid_locations].z > -40000000)
                    valid_locations++;
    }

#if 1
    for(q = 0; q < valid_locations; q++) {
        printf("Location is (%16.5f, %16.5f, %16.5f) @ %15.8f\n", sv_location[q].x, sv_location[q].y, sv_location[q].z, sv_location[q].time);
    }
#endif
  
    if(valid_locations < 4) {
        free(sv_location);
        return;
    }

    A_solve(valid_locations, sv_location, &predicted_location);
#if 1
    printf("\nSolved is  (%20f, %20f, %20f) @ %20f (alt %20f)\n", 
        predicted_location.x, predicted_location.y, predicted_location.z, predicted_location.time,
        sqrt(predicted_location.x*predicted_location.x
           + predicted_location.y*predicted_location.y
           + predicted_location.z*predicted_location.z));
#endif        
    LatLonAlt(predicted_location.x, predicted_location.y, predicted_location.z, &lat, &lon, &alt);
    printf("Lat/Lon/Alt : %20.6f, %20.6f, %20.1f\n", lat*180/PI, lon*180/PI, alt);

    if(position_file != NULL) {
        fprintf(position_file,"%16.5f, %16.5f, %16.5f, %15.8f,  %20.6f, %20.6f, %20.1f\n", 
                              predicted_location.x, predicted_location.y, predicted_location.z, predicted_location.time,
                              lat*180/PI, lon*180/PI, alt);
    }

    free(sv_location);
    return;
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
static void debug_print_orbit(struct Space_vehicle *sv) {
  printf("\nOrbit parameters for SV %i:\n",   sv->id);
  printf("iode      %02X\n",                  sv->nav_orbit.iode);
  printf("double M0        = %2.30g;\n", sv->nav_orbit.mean_motion_at_ephemeris);
  printf("double delta_n   = %2.30g;\n", sv->nav_orbit.delta_n);
  printf("double e         = %2.30g;\n", sv->nav_orbit.e);
  printf("double sqrt_A    = %2.30g;\n", sv->nav_orbit.sqrt_A);
  printf("double omega_0   = %2.30g;\n", sv->nav_orbit.omega_0);
  printf("double i_0       = %2.30g;\n", sv->nav_orbit.inclination_at_ephemeris);
  printf("double w         = %2.30g;\n", sv->nav_orbit.w);
  printf("double omega_dot = %2.30g;\n", sv->nav_orbit.omega_dot);
  printf("double idot      = %2.30g;\n", sv->nav_orbit.idot);
  printf("double Cuc       = %2.30g;\n", sv->nav_orbit.Cuc);
  printf("double Cus       = %2.30g;\n", sv->nav_orbit.Cus);
  printf("double Crc       = %2.30g;\n", sv->nav_orbit.Crc);
  printf("double Crs       = %2.30g;\n", sv->nav_orbit.Crs);
  printf("double Cic       = %2.30g;\n", sv->nav_orbit.Cic);
  printf("double Cis       = %2.30g;\n", sv->nav_orbit.Cis);
  printf("unsigned Toe     = %u;\n", sv->nav_orbit.time_of_ephemeris);
  printf("Fit       %01x\n",                  sv->nav_orbit.fit_flag);
  printf("aodo      %02X\n",                  sv->nav_orbit.aodo);    
  printf("\n");
}

void debug_print_time(struct Space_vehicle *sv) {
  struct tm  ts;
  char       buf[80];
  time_t     timestamp;

  printf("\nTime parameters for SV %i:\n",   sv->id);
  printf("Week No    %i\n", sv->nav_time.week_no);
  printf("Accuracy   %i\n", sv->nav_time.user_range_accuracy);
  printf("Health     %i\n", sv->nav_time.health);
  printf("IDOC       %i\n", sv->nav_time.issue_of_data);
  printf("double T_gd       = %2.30g;\n", sv->nav_time.group_delay);
  printf("double T_oc       = %2.30g;\n", sv->nav_time.reference_time);
  printf("double a_f2       = %2.30g;\n", sv->nav_time.correction_f2);
  printf("double a_f1       = %2.30g;\n", sv->nav_time.correction_f1);
  printf("double a_f0       = %2.30g;\n", sv->nav_time.correction_f0);
  printf("\n");
  
  timestamp = TIME_EPOCH + sv->nav_time.week_no * 604800;
  ts = *localtime(&timestamp);
  strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &ts);
  printf("Epoch is %u (%s)\n", (unsigned)timestamp, buf);
}

/******************************************************************************
* This is called everytime we have a frame of 300 bits, that passes validation
******************************************************************************/
static void nav_save_frame(struct Space_vehicle *sv) {
  unsigned int unflipped[10];
  unsigned int handover_word;
  int i;
  int frame_type = 0;
  
  /* Key the initial inversion for subframe 0 off the preamble. 
     If the LSB is 0, then it is flipped */
  if(sv->navdata.new_subframe[0] & 1)
    unflipped[0] = sv->navdata.new_subframe[0];
  else
    unflipped[0] = ~sv->navdata.new_subframe[0];

  /* The next 9 depend on the value of bit 0 of the previous subframe */
  for(i = 1; i < 10; i++) {
    if(sv->navdata.new_subframe[i-1] & 1)
      unflipped[i] = ~sv->navdata.new_subframe[i];
    else
      unflipped[i] = sv->navdata.new_subframe[i];   
  }

  /* Handover word is in subframe 1 of every frame. It includes 
     the time of start for the NEXT frame. It gets held in 
     next_time_of_week till the end of frame occurs */
  handover_word = unflipped[1];
  sv->navdata.subframe_of_week = (handover_word >> 13) & 0x1FFFF; 
  sv->lock.ms_of_frame = 0;

  char buf[100];
  time_t timestamp;
  struct tm  ts;

  timestamp = TIME_EPOCH + sv->nav_time.week_no * 604800 + sv->navdata.subframe_of_week * 6;
  ts = *localtime(&timestamp);
  strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &ts);

  
  printf("Time of next frame %i %s :",sv->navdata.subframe_of_week, buf);
  /* Handover word also includes the subframe type */
  frame_type = (handover_word >>  8) & 0x7;

  /* Now Save the required frames for later */
  printf("Frame type is %i: ",frame_type);
  for(i=0; i < 10; i++)
    printf(" %08X", unflipped[i]);
  printf("\n");

  if(frame_type > 0 && frame_type < 6) {
      if(sv->nav_file == NULL) {
          char name[100];
          sprintf(name, "NAV_%02i.dat",sv->id);
          sv->nav_file = fopen(name,"r+b");
          if(sv->nav_file == NULL) {
             sv->nav_file = fopen(name,"wb");
          } 
          if(sv->nav_file == NULL) {
              printf("Unable to open NAV file '%s'\n",name);
          }
      }
      if(sv->nav_file != NULL) {
          fseek(sv->nav_file, sizeof(sv->navdata.new_subframe)*(frame_type-1), SEEK_SET);
          fwrite(sv->navdata.new_subframe,sizeof(sv->navdata.new_subframe),1,sv->nav_file);
          fflush(sv->nav_file);
      }
  }
      
  if(frame_type == 1) {
    sv->navdata.valid_subframe[1] = 1;
      for(i = 0; i < 10; i++)
         sv->navdata.subframes[1][i] = unflipped[i];
  
      sv->nav_time.week_no              = join_bits_u(0,           0, 0, sv->navdata.subframes[1][2], 20,10);
      /* Week 524+1024 is sometime in late 2010. This will work for about 20 years after that */
      if(sv->nav_time.week_no < 524) {
        sv->nav_time.week_no += 1024*2;
      } else {
        sv->nav_time.week_no += 1024;
      }
      sv->nav_time.user_range_accuracy  = join_bits_u(0,           0, 0, sv->navdata.subframes[1][2], 14, 4);
      sv->nav_time.health               = join_bits_u(0,           0, 0, sv->navdata.subframes[1][2],  8, 6);
      sv->nav_time.group_delay     = join_bits_s(0,           0, 0, sv->navdata.subframes[1][6],  6, 8) * pow(2.0, -31.0);
      sv->nav_time.issue_of_data  = join_bits_u(sv->navdata.subframes[1][2],6, 2, sv->navdata.subframes[1][7], 22, 8);
      sv->nav_time.reference_time = join_bits_u(0,           0, 0, sv->navdata.subframes[1][7],  6,16) * pow(2.0, 4.0);
      sv->nav_time.correction_f2  = join_bits_s(0,           0, 0, sv->navdata.subframes[1][8], 22, 8) * pow(2.0, -55.0);
      sv->nav_time.correction_f1  = join_bits_s(0,           0, 0, sv->navdata.subframes[1][8],  6,16) * pow(2.0, -43.0);
      sv->nav_time.correction_f0  = join_bits_s(0,           0, 0, sv->navdata.subframes[1][9],  8,22) * pow(2.0, -31.0);
      debug_print_time(sv);
      sv->nav_time.time_good = 1;
  } else if(frame_type == 2 || frame_type == 3) {  
    unsigned iode2, iode3; 
    /* Frame 2 and 3 share common processsing */
    if(frame_type == 2) {
      sv->navdata.valid_subframe[2] = 1;
      for(i = 0; i < 10; i++)
        sv->navdata.subframes[2][i] = unflipped[i];
    } else {
      sv->navdata.valid_subframe[3] = 1;
      for(i = 0; i < 10; i++)
        sv->navdata.subframes[3][i] = unflipped[i];
    }

    /**************************************************
    * First, check that bith frames have the same Issue
    * Of Data Ephemeris values, i.e. they are havles of
    * the same data set, and that we have not already 
    * extracted the orbit parameters
    **************************************************/
    iode2               = join_bits_u(0,            0, 0, sv->navdata.subframes[2][2], 22, 8);
    iode3               = join_bits_u(0,            0, 0, sv->navdata.subframes[3][9], 22, 8);
    if(iode2 == iode3 && iode2 != sv->nav_orbit.iode) {
      /* Great! We can start extracting the fields out of the frames */
      /****************************************************
      * At most, fields are split over two subframes - so
      * we can extract them all the same, consistent way
      ****************************************************/
      /******** From FRAME 2 *******/
      sv->nav_orbit.iode        = iode2;
      sv->nav_orbit.Crs                      = join_bits_s(0,            0, 0, sv->navdata.subframes[2][2],  6,16) * pow(2.0,-5.0);
      sv->nav_orbit.delta_n                  = join_bits_s(0,            0, 0, sv->navdata.subframes[2][3], 14,16) * pow(2.0,-43.0) * PI;
      sv->nav_orbit.mean_motion_at_ephemeris = join_bits_s(sv->navdata.subframes[2][3], 6, 8, sv->navdata.subframes[2][4],  6,24) * pow(2.0,-31.0) * PI;
      sv->nav_orbit.Cuc                      = join_bits_s(0,            0, 0, sv->navdata.subframes[2][5], 14,16) * pow(2.0,-29.0);
      sv->nav_orbit.e                        = join_bits_u(sv->navdata.subframes[2][5], 6, 8, sv->navdata.subframes[2][6],  6,24) * pow(2.0,-33.0);
      sv->nav_orbit.Cus                      = join_bits_s(0,            0, 0, sv->navdata.subframes[2][7], 14,16) * pow(2.0,-29.0);
      sv->nav_orbit.sqrt_A                   = join_bits_u(sv->navdata.subframes[2][7], 6, 8, sv->navdata.subframes[2][8],  6,24) * pow(2.0,-19.0);
      sv->nav_orbit.time_of_ephemeris        = join_bits_u(0,            0, 0, sv->navdata.subframes[2][9], 14,16) * 16.0;
      sv->nav_orbit.fit_flag                 = join_bits_u(0,            0, 0, sv->navdata.subframes[2][9], 13, 1);
      sv->nav_orbit.aodo                     = join_bits_u(0,            0, 0, sv->navdata.subframes[2][9],  8, 6);

      /******** From FRAME 3 *******/
      sv->nav_orbit.Cic                      = join_bits_s(0,            0, 0, sv->navdata.subframes[3][2], 14,16) * pow(2.0,-29.0);
      sv->nav_orbit.omega_0                  = join_bits_s(sv->navdata.subframes[3][2], 6, 8, sv->navdata.subframes[3][3],  6,24) * pow(2.0,-31.0) * PI;
      sv->nav_orbit.Cis                      = join_bits_s(0,            0, 0, sv->navdata.subframes[3][4], 14,16) * pow(2.0,-29.0);
      sv->nav_orbit.inclination_at_ephemeris = join_bits_s(sv->navdata.subframes[3][4], 6, 8, sv->navdata.subframes[3][5],  6,24) * pow(2.0,-31.0) * PI;
      sv->nav_orbit.Crc                      = join_bits_s(0,            0, 0, sv->navdata.subframes[3][6], 14,16) * pow(2.0,-5.0);
      sv->nav_orbit.w                        = join_bits_s(sv->navdata.subframes[3][6], 6, 8, sv->navdata.subframes[3][7],  6,24) * pow(2.0,-31.0) * PI;
      sv->nav_orbit.omega_dot                = join_bits_s(0,            0, 0, sv->navdata.subframes[3][8],  6,24) * pow(2.0,-43.0) * PI;
      sv->nav_orbit.idot                     = join_bits_s(0,            0, 0, sv->navdata.subframes[3][9],  8,14) * pow(2.0,-43.0) * PI;
      sv->nav_orbit.orbit_valid = 1;
      debug_print_orbit(sv);
    }
  } else if(frame_type == 4) {
      sv->navdata.valid_subframe[4] = 1;
    /* Frame not used - holds almanac data */
  } else if(frame_type == 5) {
      sv->navdata.valid_subframe[5] = 1;
    /* Frame not used - holds almanac data */
  } else {
    printf("Invalid subframe type\n");
  }
}

/******************************************************************************
* Read in any cached NAV data for one Space Vehicles. This will allow results 
* quicker as we don't have to wait for all the orbit info to be received
******************************************************************************/
static int nav_read_in_cached_data(struct Space_vehicle *sv) {
    FILE *f;
    char name[22];

    sprintf(name, "NAV_%02i.dat",sv->id);
    f = fopen(name,"r+");
    if(f == NULL) {
        printf("Unable to open NAV file '%s'\n",name);
        return 0;
    }

    while(fread(sv->navdata.new_subframe,40,1,f) == 1) {
        printf("Read in subframe\n");
        nav_save_frame(sv);
    }
    fclose(f);
    /* Reset the time_good flag, as the frame_of_week will be wrong */
    sv->nav_time.time_good = 0;
    return 1;
}

/******************************************************************************
* Read in any cached NAV data for all Space Vehicles. This will allow results
* quicker as we don't have to wait for all the orbit info to be received
******************************************************************************/
static void nav_read_in_all_cached_data(void) {
    int i;
    for(i = 0; i < N_SV; i++) {
        nav_read_in_cached_data(space_vehicles+i);
    }
}

/******************************************************************************
* Determine if the last 32 bits holds a valid preamble value (tests not only
* bits 29 to 24 of the current frame, but also bit 30, which is the last bit
* of the previous frame, indicating if the following subframe is flipped)
******************************************************************************/
static int nav_test_telemetry(struct Space_vehicle *sv) {
    /* TOD - mask should be 0x7FC00000A, and the first test should be against 0x5D000000 */
    
    unsigned int temp = sv->navdata.new_word & 0x3FC00000;

    /* Test the first 8 bits for for the preamble, bur also check the 
    * last bit of the previous frame to see if this one is inverted  */
    if(temp == 0x1D000000)
        return 1;
    if(temp == 0x22C00000)
        return 1;
    return 0;
}
    
/************************************************************
* Does new_word hold a valid NAV subframe? 
***********************************************************/
static int nav_valid_subframes(struct Space_vehicle *sv) {
    if(!nav_test_parity(sv->navdata.new_word)) {
#if SHOW_NAV_FRAMING
        printf("%02i:%5i: Parity fail\n", sv->id, sv->navdata.seq);
#endif
        return 0;
    }

    if(!sv->navdata.synced) {
        if(!nav_test_telemetry(sv)) {
#if SHOW_NAV_FRAMING
            printf("%02i:%5i: Telemetry fail 0x%08X\n", sv->id, sv->navdata.seq, sv->navdata.new_word );
#endif
            return 0;
        }
#if SHOW_NAV_FRAMING
        printf("%02i:%5i: Valid telemetry 0x%08X - synced\n", sv->id, sv->navdata.seq, sv->navdata.new_word );
#endif
        sv->navdata.new_subframe[sv->navdata.subframe_in_frame] = sv->navdata.new_word;
        sv->navdata.synced      = 1;
        sv->navdata.subframe_in_frame = 1;
        sv->lock.ms_of_frame      = 600;
    } else {
        if(sv->navdata.subframe_in_frame == 0) {
            if(!nav_test_telemetry(sv)) {
#if SHOW_NAV_FRAMING
                printf("%02i:%5i: Telemetry fail 0x%08X\n", sv->id, sv->navdata.seq, sv->navdata.new_word );
#endif
                sv->navdata.synced = 0;
                return 0;
            }
#if SHOW_NAV_FRAMING
            printf("%02i:%5i: Valid telemetry\n", sv->id, sv->navdata.seq);
#endif
            sv->navdata.new_subframe[sv->navdata.subframe_in_frame] = sv->navdata.new_word;
            sv->navdata.subframe_in_frame = 1;
        } else {
#if SHOW_NAV_FRAMING
            printf("%02i:%5i:    Subframe %i\n", sv->navdata.seq, sv->id, sv->navdata.subframe_in_frame);
#endif
            sv->navdata.new_subframe[sv->navdata.subframe_in_frame] = sv->navdata.new_word;
            if(sv->navdata.subframe_in_frame  == 9) {
                nav_save_frame(sv);
                sv->navdata.subframe_in_frame = 0;
            } else {
                sv->navdata.subframe_in_frame++;
            }
        }
    }
    return 1;
}
/************************************************************************
* Receive a new bit of NAV data
************************************************************************/
static void nav_new_bit(struct Space_vehicle *sv, uint_8 s) {
    /* Shift in the next bit */
    sv->navdata.new_word <<= 1;
    if(s) sv->navdata.new_word |= 1;

    sv->navdata.valid_bits++;
    if(sv->navdata.valid_bits == 32) {
        if(nav_valid_subframes(sv)) {
            sv->navdata.valid_bits = 2;
        } else {
            sv->navdata.valid_bits = 31;
        }
    }
    sv->navdata.seq++;
}

/*************************************************************************
* Restart the reception of NAV data. This happens if we have an
* unexpected BPSK phase flip
*************************************************************************/
static void nav_abandon(struct Space_vehicle *sv) {
    printf("%2i: Abandon - %5i errors\n", sv->id, sv->navdata.bit_errors);
    sv->navdata.valid_bits = 0;
    sv->navdata.synced = 0;
    /* Force drop if we havn't got a good bit after 'n' transistions */
    if(sv->navdata.bit_errors > MAX_BAD_BIT_TRANSITIONS)
        sv->state = state_acquiring;
    sv->navdata.bit_errors++;
}

/*************************************************************************
* We have a new MS of signal. Work towards decoding a bit of BPSH data
*************************************************************************/
static void nav_process(struct Space_vehicle *sv, uint_8 s) {
    if(sv->navdata.part_in_bit == BIT_LENGTH-1) {
        if(sv->navdata.bit_errors>0)
            sv->navdata.bit_errors--;
        sv->navdata.part_in_bit = 0;
    } else if(s != (sv->navdata.new_word&1)) {
        nav_abandon(sv);
        sv->navdata.part_in_bit = 0;
    } else
        sv->navdata.part_in_bit++;
    
    if(sv->navdata.part_in_bit == 0) {
        nav_new_bit(sv,s);
    }
}

/*********************************************************************
* Print out bits
*********************************************************************/
#if 0
static int print_bitmap(uint_32 *a, int i) {
  int j;
  for(j = 0; j < i; j++) 
     putchar(a[j/32] & (1<<j%32) ? '1' : '0');
  printf("\n");
}
#endif

/*********************************************************************
* Count how many bits are set in 'i' bits
*********************************************************************/
static int count_one_bits(uint_32 *a, int i) {
  int count = 0;
  int j;
  uint_8 *b = (uint_8 *)a;

  static int one_bits[256];
  static int setup = 1;
  
  if(setup) {
      for(j = 0; j < 256; j++) {
          one_bits[j] = 0;
          if(j&0x01) one_bits[j]++;
          if(j&0x02) one_bits[j]++;
          if(j&0x04) one_bits[j]++;
          if(j&0x08) one_bits[j]++;
          if(j&0x10) one_bits[j]++;
          if(j&0x20) one_bits[j]++;
          if(j&0x40) one_bits[j]++;
          if(j&0x80) one_bits[j]++;
      }
      setup = 0;
  }

  count = 0;
  while(i >= 8) {
     count += one_bits[b[0]&0xFF];
     i -= 8;
     b++;
  }

  for(j = 0; j < i; j++) {
     if(b[0] & (1<<j))
         count++;
  }
  return count;
}

/*********************************************************************
* XOR two buffers to gether to a new destination
*********************************************************************/
static void xor_to_work(uint_32 *dest, uint_32 *a, uint_32 *b) {
  int i;
  for(i = 0; i < acquire_bitmap_size_u32; i++) {
     dest[i] = a[i] ^ b[i];
  }
}

/*********************************************************************
* Start tracking - flip from acquire to tracking
*********************************************************************/
static void start_locked(struct Space_vehicle *sv, int offset_from_peak) {
    int lower_delta=0, upper_delta=0;
    int start_freq, fine_adjustment;
    long long step;

    start_freq = (int)band_bandwidth * ((int)sv->track.band-(search_bands/2));
    lower_delta = (sv->track.power[1][1] - sv->track.power[1][0])/1024;
    upper_delta = (sv->track.power[1][1] - sv->track.power[1][2])/1024;

    /* Clamp to prevent divide by zero */
    if(lower_delta < 1) lower_delta = 1;
    if(upper_delta < 1) upper_delta = 1;

    if(lower_delta > upper_delta) {
      fine_adjustment = (int)band_bandwidth * (lower_delta-upper_delta) / lower_delta;
    } else if(lower_delta < upper_delta) {
      fine_adjustment = - (int)band_bandwidth * (lower_delta-upper_delta) / upper_delta;
    } else {
      fine_adjustment = 0;
    }
    printf("Adjust %4i   \n",fine_adjustment);
    start_freq -= fine_adjustment;


    sv->state = state_locked;
    sv->nav_time.time_good = 0;
    
    /* Prime the IIR filter so we don't unlock immediately */
    sv->lock.filtered_power = 1000000;

    /* Set up the NCO parameters */

    step = ((1<<30) / (samples_per_ms * 1000/4) * start_freq)  + (unsigned)((long long)(1<<30)*4*if_cycles_per_ms/samples_per_ms);
    sv->lock.phase_nco_sine       = 0;
    sv->lock.phase_nco_cosine     = 0x40000000;
    sv->lock.phase_nco_step       = step;

    sv->lock.code_nco_step          = (unsigned)((long long)(1<<22)*1023/samples_per_ms);
    sv->lock.code_nco_filter      = 0;
    sv->lock.code_nco = offset_from_peak * sv->lock.code_nco_step;
    sv->lock.code_nco_trend = start_freq*272/100;
#if 0
    int s,c;
    /*** This doesn't work *****/
    /* Set up the last angle, preventing any initial angle from
       upsetting the ability lock */
    s = sv->lock.prompt_sine_total;
    c = sv->lock.prompt_cosine_total;    
    while(c > 15 || c < -15 || s > 15 || s < -15) {
        c /= 2;
        s /= 2;
    }
    s &= ATAN2_SIZE-1;  c &= ATAN2_SIZE-1;
    sv->lock.last_angle = atan2_lookup[c][s];
#endif
    sv->lock.last_angle = 0;

    /* Reset the running totals */
    sv->lock.early_sine_count_last    = sv->lock.early_sine_count; 
    sv->lock.early_cosine_count_last  = sv->lock.early_cosine_count;
    sv->lock.prompt_sine_count_last   = sv->lock.prompt_sine_count; 
    sv->lock.prompt_cosine_count_last = sv->lock.prompt_cosine_count;
    sv->lock.late_sine_count_last     = sv->lock.late_sine_count; 
    sv->lock.late_cosine_count_last   = sv->lock.late_cosine_count;

    sv->lock.early_power           = 0;
    sv->lock.early_power_not_reset = 0;
    sv->lock.prompt_power          = 2 * lock_lost_power;
    sv->lock.late_power            = 0;
    sv->lock.late_power_not_reset  = 0;
    sv->lock.delta_filtered        = 0;
    sv->lock.angle_filtered        = 0;
    sv->lock.ms_of_bit             = 0;
    sv->navdata.bit_errors         = 0;
    
#if LOCK_SHOW_INITIAL
    printf("%2i: ", sv->id);
    printf("Lower band %7i upper band %7i ", lower_delta, upper_delta);
    printf("Adjust %4i   ",fine_adjustment);
    printf("Freq guess %i\n",start_freq);
    printf("\nLock band %i, Lock offset %i, step %x,  Code NCO %8x\n", sv->track.band, sv->track.offset, (unsigned) step, sv->lock.code_nco>>22);
    printf("lock_phase_nco_step  %8x\n",sv->lock.phase_nco_step);
    printf("lock code_nco & step    %8x   %8x %i\n",sv->lock.code_nco, sv->lock.code_nco_step, sv->lock.code_nco_trend);
#endif

#if WRITE_BITS
    char name[100];
    sprintf(name,"bits.%i",sv->id);
    sv->bits_file = fopen(name,"wb");
    if(sv->bits_file == NULL) {
        printf("WARNING : UNABLE TO OPEN '%s'\n",name);
    }
#else    
    sv->bits_file = NULL;
#endif
}

/*********************************************************************
* Start tracking - flip from acquire to tracking
*********************************************************************/
static void start_tracking(struct Space_vehicle *sv) {
    sv->state          = state_tracking;
    sv->track.percent_locked = 50;

    /* Where we are hunting around */
    sv->track.band       = sv->acquire.max_band;
    sv->track.offset     = sv->acquire.max_offset;

    sv->track.max_power  = 0;           
}

/*********************************************************************
* Start acquire - start the hunt for the signal 
*********************************************************************/
static void start_acquire(struct Space_vehicle *sv) {
            sv->state = state_acquiring;
            sv->acquire.max_power = 0;
}

/*********************************************************************
* When tracking, this is how to find the peak power 
*********************************************************************/
void track_update(struct Space_vehicle *sv) {
    /**********************************************************
    * We have enough power to see if we can work toward locking
    **********************************************************/
    if(sv->track.max_power < track_unlock_power) {
        sv->track.percent_locked--;
        if(sv->track.percent_locked==0) {
            start_acquire(sv);
#if SHOW_LOCK_UNLOCK         
            printf("Unlocked %i\n", sv->id);
#endif
            sv->state = state_acquiring;
            sv->acquire.max_power = 0;
        }
    } else {
        /***************************************
        * If this is  is the local maximum, then
        * se can moved towards being locked 
        ***************************************/
        if(sv->track.power[1][1] >= sv->track.power[1][0] &&
           sv->track.power[1][1] >= sv->track.power[1][2] &&
           sv->track.power[1][1] >= sv->track.power[0][1] &&
           sv->track.power[1][1] >= sv->track.power[2][1]) {
           sv->track.percent_locked+=5;
        }

        if(sv->track.percent_locked > 99) {
            sv->track.percent_locked = 100;  
            start_locked(sv, 2);  // Offset of two from peak power
        } else {
            sv->track.max_power = 0;
            sv->track.band   = sv->track.max_band;
            sv->track.offset = sv->track.max_offset;
        }
    }
}
/*********************************************************************
* How we track, to find the maximum signal level, allowing us to 
* flipover to locked when we are sure that we have the best initial 
* starting condition to ensure a successful lock
*********************************************************************/
void track(struct Space_vehicle *sv) {
    int line,i;
    if((sv->track.offset+1)%samples_per_ms == code_offset_in_ms) {
        line =0;
    } else if(sv->track.offset == code_offset_in_ms) {
        line = 1;
    } else if((sv->track.offset+samples_per_ms-1)%samples_per_ms == code_offset_in_ms) {
        line = 2;
    } else if((sv->track.offset+samples_per_ms-2)%samples_per_ms == code_offset_in_ms) {
        track_update(sv);
        return;
    } else {
        return;
    }

#if PRINT_TRACK_POWER
    if(line == 0) printf("\n");
    printf("%2i %2i%% %5i: ", sv->id, sv->track.percent_locked, code_offset_in_ms);
    for(i = 0; i < sv->track.band-1;i++)
            printf("      ");      
#endif
    for(i = 0; i <3; i++) {
        int band;
        int sin_power, cos_power, power;

        band = sv->track.band-1+i;
        if(band < 0) band = 0;
        if(band >= search_bands) band = search_bands-1;

        xor_to_work(work_buffer, sample_history, sv->acquire.seek_in_phase[band]);
        sin_power = count_one_bits(work_buffer, samples_for_acquire) - samples_for_acquire/2;

        xor_to_work(work_buffer, sample_history, sv->acquire.seek_quadrature[band]);
        cos_power = count_one_bits(work_buffer, samples_for_acquire) - samples_for_acquire/2;

        power = sin_power*sin_power + cos_power*cos_power;
        sv->track.power[line][i] = power;
#if PRINT_TRACK_POWER
        if(power > PRINT_TRACK_SQUETCH) 
            printf("%5i ", sv->track.power[line][i]/1000);
        else
            printf("      ");      
#endif
        if(power > sv->track.max_power) {
            sv->track.max_power  = power;
            sv->track.max_band   = band;
            sv->track.max_offset = code_offset_in_ms;
        }
    }
#if PRINT_TRACK_POWER
    printf("\n");      
#endif

    /************************************************************
    * If we aren't on the last of the three lines, then return
    ************************************************************/
    if(line != 2) 
        return;
    
}

/*********************************************************************
* acquire - attempt to see if a SV has transmitting with this phase
*********************************************************************/
static void acquire(struct Space_vehicle *sv) {
   int band;

   for(band = 0; band < search_bands; band++) {
      int sin_power, cos_power, power;
      xor_to_work(work_buffer, sample_history, sv->acquire.seek_in_phase[band]);
      sin_power = count_one_bits(work_buffer, samples_for_acquire) - samples_for_acquire/2;

      xor_to_work(work_buffer, sample_history, sv->acquire.seek_quadrature[band]);
      cos_power = count_one_bits(work_buffer, samples_for_acquire) - samples_for_acquire/2;

      power = sin_power*sin_power + cos_power*cos_power;
#if PRINT_ACQUIRE_POWERS
      if(power > PRINT_ACQUIRE_SQUETCH) 
         printf("%5i ", power/1000);
      else
         printf("      ");
      
#endif
      if(sv->acquire.max_power < power) {
        sv->acquire.max_power  = power;
        sv->acquire.max_band   = band;
        sv->acquire.max_offset = code_offset_in_ms;
      }
   }
#if PRINT_ACQUIRE_POWERS
   printf("\n");
#endif
   if(code_offset_in_ms  == 0) {
       if(sv->acquire.max_power > acquire_min_power) {
           start_tracking(sv);
       } else {
          sv->acquire.max_power = 0;           
       }
       printf("%02i: Max power %7i at band %2i, offset %5i %s\n", 
              sv->id,
            sv->acquire.max_power,
            sv->acquire.max_band,
            sv->acquire.max_offset,
            sv->state == state_tracking ? "TRACKING" : "");       
   }
}

/*********************************************************************
* Move all the values in sample_history along by one bit.
*********************************************************************/
static void add_to_bitmap(uint_32 *bitmap, int s) {
  int i;
  for(i = samples_for_acquire/32; i > 0; i--) {
     bitmap[i] <<= 1;
     if(bitmap[i-1]&0x80000000)
        bitmap[i]++;
  }
  /*************************
  * And now the last sample 
  *************************/
  bitmap[i] <<= 1;
  if(s)
     bitmap[i]++;
}

/*********************************************************************
* Set a bit in the bitmap 
*********************************************************************/
static void bitmap_set_bit(uint_32 *bitmap, int o, int s) {
 if(s) {
    bitmap[o/32] |= 1<<(o%32);
 } else {
    bitmap[o/32] &= ~(1<<(o%32));      
 }
}

/*********************************************************************
* Fine-tune the the code phase tracking, by looking at the relative
* powers of the early and late signal
*********************************************************************/
static void adjust_early_late(struct Space_vehicle *sv) {
    int adjust;
    /* Use the relative power levels of the late and early codes 
     * to adjust the code NCO phasing */
    adjust =  sv->lock.code_nco_trend;

    if(sv->lock.early_power/5 > sv->lock.late_power/4) {
        sv->lock.late_power = (sv->lock.early_power*7+sv->lock.late_power)/8;
        adjust += samples_per_ms*1;
        sv->lock.code_nco_trend+=4;
    } else if(sv->lock.late_power/5 > sv->lock.early_power/4) {
        sv->lock.early_power = (sv->lock.late_power*7+sv->lock.early_power)/8;
        adjust  -= samples_per_ms*1;
        sv->lock.code_nco_trend-=4;
    }

    if(adjust < 0) {
        if( sv->lock.code_nco  < -adjust) 
            sv->lock.code_nco += 1023<<22;
    }
    sv->lock.code_nco += adjust;

    if(sv->lock.code_nco+adjust >= 1023<<22) 
        sv->lock.code_nco -= 1023<<22;
        
#if LOCK_SHOW_EARLY_LATE_TREND
    printf("%2i: %6i, %6i, %6i, %5i\n", sv->id,
    sv->lock.early_power_not_reset/LATE_EARLY_IIR_FACTOR,
    sv->lock.prompt_power,
    sv->lock.late_power_not_reset/LATE_EARLY_IIR_FACTOR,
    sv->lock.code_nco_trend);
#endif
}

/*********************************************************************
* Accumulate!
********************************************************************/
static void accumulate(struct Space_vehicle *sv, int sample) {
    int early_code_index,  late_code_index;
    uint_8 sine_positive, cosine_positive;
    int prompt_code_index;

    prompt_code_index            = sv->lock.code_nco>>22;

    early_code_index  = (prompt_code_index == CHIPS_PER_MS-1) ? 0: prompt_code_index+1;
    late_code_index   = (prompt_code_index == 0) ? CHIPS_PER_MS-1: prompt_code_index-1;

    /***********************************************************
    * What is our current sein()/cosine() (I/Q) value?
    ***********************************************************/
    sine_positive   = (sv->lock.phase_nco_sine   & 0x80000000) ? 0 : 1;
    cosine_positive = (sv->lock.phase_nco_cosine & 0x80000000) ? 0 : 1;

    if(sv->gold_code[early_code_index] ^ sample) {
        sv->lock.early_sine_count    += sine_positive;
        sv->lock.early_cosine_count  += cosine_positive;
    }

    if(sv->gold_code[prompt_code_index] ^ sample) {
        sv->lock.prompt_sine_count   += sine_positive;
        sv->lock.prompt_cosine_count += cosine_positive;
    }

    if(sv->gold_code[late_code_index] ^ sample) {
        sv->lock.late_sine_count     += sine_positive;
        sv->lock.late_cosine_count   += cosine_positive;
    }
}

/*********************************************************************
* Update the "code received early" power levels
*********************************************************************/
static void update_early(struct Space_vehicle *sv, int prompt_code_index, int prompt_code_index_next_cycle) {

#if DOUBLECHECK_PROMPT_CODE_INDEX
    int early_code_index;
    early_code_index  = (prompt_code_index == CHIPS_PER_MS-1) ? 0: prompt_code_index+1;

    /* This should never be true, but just in case... */
    if(prompt_code_index == prompt_code_index_next_cycle) {
        printf("Bad prompt_code_index in update_early()\n");
        return;
    }
    

    /***********************************************************
    * Ensure that this is the last data bitt for this repeat of 
    * the prompt C/A code?
    ***********************************************************/
    if(early_code_index != CHIPS_PER_MS-1) {
       printf("Bad prompt_code_index in update_early()\n");
       return;
    }
#endif

    /* Work out the changes over the last repeat of the Gold code */
    sv->lock.early_sine   = sv->lock.early_sine_count   - sv->lock.early_sine_count_last   - samples_per_ms/4;
    sv->lock.early_cosine = sv->lock.early_cosine_count - sv->lock.early_cosine_count_last - samples_per_ms/4;
    sv->lock.early_sine_count_last   = sv->lock.early_sine_count;
    sv->lock.early_cosine_count_last = sv->lock.early_cosine_count;

    sv->lock.early_power -= sv->lock.early_power/LATE_EARLY_IIR_FACTOR;
    sv->lock.early_power += sv->lock.early_sine   * sv->lock.early_sine
                          + sv->lock.early_cosine * sv->lock.early_cosine;
                             
    sv->lock.early_power_not_reset -= sv->lock.early_power_not_reset/LATE_EARLY_IIR_FACTOR;
    sv->lock.early_power_not_reset += sv->lock.early_sine   * sv->lock.early_sine
                                    + sv->lock.early_cosine * sv->lock.early_cosine;
}

/*********************************************************************
* Update the "code received late" power levels
*********************************************************************/
static void update_late(struct Space_vehicle *sv, int prompt_code_index, int prompt_code_index_next_cycle) {

#if DOUBLECHECK_PROMPT_CODE_INDEX
    int late_code_index;
    late_code_index   = (prompt_code_index == 0) ? CHIPS_PER_MS-1: prompt_code_index-1;

    /* This should never be true, but just in case... */
    if(prompt_code_index == prompt_code_index_next_cycle) {
        printf("Bad prompt_code_index in update_late()\n");
        return;
    }
    

    /***********************************************************
    * Is this the last data bit for this repeat of the prompt C/A code?
    ***********************************************************/
    if(late_code_index != CHIPS_PER_MS-1) {
        printf("Bad prompt_code_index in update_late()\n");
        return;
    }
#endif

    sv->lock.late_sine   = sv->lock.late_sine_count   - sv->lock.late_sine_count_last   - samples_per_ms/4;
    sv->lock.late_cosine = sv->lock.late_cosine_count - sv->lock.late_cosine_count_last - samples_per_ms/4;
    sv->lock.late_sine_count_last   = sv->lock.late_sine_count;
    sv->lock.late_cosine_count_last = sv->lock.late_cosine_count;

    sv->lock.late_power -= sv->lock.late_power/LATE_EARLY_IIR_FACTOR;
        
    sv->lock.late_power += sv->lock.late_sine   * sv->lock.late_sine
                         + sv->lock.late_cosine * sv->lock.late_cosine;
                            
    sv->lock.late_power_not_reset -= sv->lock.late_power_not_reset/LATE_EARLY_IIR_FACTOR;        
    sv->lock.late_power_not_reset += sv->lock.late_sine   * sv->lock.late_sine
                                   + sv->lock.late_cosine * sv->lock.late_cosine;
                            
#if LOCK_SHOW_PER_MS_POWER
    printf("%2i: %6i, %6i, %6i\n", sv->id,
    sv->lock.early_power_not_reset/LATE_EARLY_IIR_FACTOR,
    sv->lock.prompt_power,
    sv->lock.late_power_not_reset/LATE_EARLY_IIR_FACTOR);
#endif
}

/*********************************************************************
* Track the phase of the carrier using the vector of the prompt 
* signal to tune the NCO
*********************************************************************/
static void adjust_prompt(struct Space_vehicle *sv) {
    int s, c;
    int_8 delta;
    int adjust = 0;
    uint_8 angle;
    uint_8 this_bit;

    s = sv->lock.prompt_sine;
    c = sv->lock.prompt_cosine;
    while(c > 15 || c < -15 || s > 15 || s < -15) {
        c /= 2;
        s /= 2;
    }

    s &= ATAN2_SIZE-1;  c &= ATAN2_SIZE-1;
    angle = atan2_lookup[c][s];
    delta = angle - sv->lock.last_angle;
    sv->lock.last_angle = angle;
 
   sv->lock.delta_filtered -= sv->lock.delta_filtered / LOCK_DELTA_IIR_FACTOR;
    sv->lock.delta_filtered += delta;

    adjust = angle;
    sv->lock.angle_filtered -= sv->lock.angle_filtered / LOCK_ANGLE_IIR_FACTOR;
    sv->lock.angle_filtered += angle;

    if(angle >=128)
        sv->lock.angle_filtered -= 256;

    adjust = sv->lock.angle_filtered/8;
    
    adjust  += (1<<24) / 32 / LOCK_DELTA_IIR_FACTOR / samples_per_ms * sv->lock.delta_filtered;
    sv->lock.phase_nco_step  -= adjust;
#if LOCK_SHOW_ANGLES
    printf("%6i, %6i, %3i,%4i,%6i, %6i, %6i\n",sv->lock.prompt_sine_power, sv->lock.prompt_cosine_power, angle,delta, sv->lock.delta_filtered, adjust, sv->lock.phase_nco_step);
#endif    
  
    if(sv->lock.prompt_cosine < 0)
        this_bit = 0;
    else
        this_bit = 1;
    sv->lock.ms_of_bit++;
    if(sv->lock.ms_of_bit == MS_PER_BIT || sv->lock.last_bit != this_bit)
        sv->lock.ms_of_bit = 0;
    sv->lock.last_bit = this_bit;

    if(sv->lock.ms_of_frame == MS_PER_BIT*BITS_PER_FRAME-1) {
        sv->lock.ms_of_frame = 0;
        if(sv->navdata.subframe_of_week == 7*24*60*60/6-1) {
            sv->navdata.subframe_of_week=0;
        } else {
            sv->navdata.subframe_of_week++;
        }
    } else {
        sv->lock.ms_of_frame++;
    }
  
    nav_process(sv,this_bit);

#if LOCK_SHOW_BITS
    if(sv->lock_ms_of_bit == 0)
        putchar('\n');
    putchar('0'+this_bit);
#endif

#if LOCK_SHOW_BPSK_PHASE_PER_MS
    if(sv->bits_file) {
        if(sv->lock_ms_of_bit == 0)
            putc('\n',sv->bits_file);
        putc('0'+this_bit,sv->bits_file);
    }
#endif
 
}

/*********************************************************************
* Update the state of a locked channel with the new sample.
* Slow path for a single sample, used when the code phase NCO will 
* roll over and adjustments are needed
*********************************************************************/
static void update_prompt(struct Space_vehicle *sv, int prompt_code_index, int prompt_code_index_next_cycle) {
#if DOUBLECHECK_PROMPT_CODE_INDEX
    /*************************************************
    * This should never be true, but just in case... 
    *************************************************/
    if(prompt_code_index == prompt_code_index_next_cycle) {
        printf("Bad prompt_code_index in update_prompt()\n");
        return;
    }
    
    /***********************************************************
    * Make sure that this is the last data bit for this 
    * repeat of the prompt C/A code?
    ***********************************************************/
    /* This too should never be true, but just in case... */
    if(prompt_code_index != CHIPS_PER_MS-1) {
        printf("Bad prompt_code_index in update_prompt()\n");
        return;
    }
#endif
    /***********************
    * Yes - so do the update 
    ***********************/
    sv->lock.prompt_sine   = sv->lock.prompt_sine_count   - sv->lock.prompt_sine_count_last    - samples_per_ms/4;
    sv->lock.prompt_cosine = sv->lock.prompt_cosine_count - sv->lock.prompt_cosine_count_last  - samples_per_ms/4;
    sv->lock.prompt_sine_count_last = sv->lock.prompt_sine_count;
    sv->lock.prompt_cosine_count_last = sv->lock.prompt_cosine_count;
    
    sv->lock.prompt_power -= sv->lock.prompt_power/LATE_EARLY_IIR_FACTOR;
    sv->lock.prompt_power += sv->lock.prompt_sine   * sv->lock.prompt_sine
                           + sv->lock.prompt_cosine * sv->lock.prompt_cosine;

#if LOCK_SHOW_PER_MS_IQ
    printf("%2i-%4i:  (%6i, %6i)   %8i\n", sv->id, sv->lock.code_nco>>22, 
    sv->lock.prompt_sine, sv->lock.prompt_cosine, sv->lock.prompt_power);
#endif        

    if(sv->lock.prompt_power/LATE_EARLY_IIR_FACTOR < lock_lost_power) { 
        printf("Lock lost at power %u", sv->lock.prompt_power/LATE_EARLY_IIR_FACTOR);
        sv->state = state_acquiring;
        return;
    }
    adjust_prompt(sv);
}

/*********************************************************************
* Update all the NCOs for this Space Vehicle
*********************************************************************/
static void update_ncos(struct Space_vehicle *sv) {    
    /***************************
    * Update the phase NCO 
    ***************************/
    sv->lock.phase_nco_sine   += sv->lock.phase_nco_step;
    sv->lock.phase_nco_cosine += sv->lock.phase_nco_step;
    /***********************************
    * Update the current C/A code offset
    ***********************************/
    sv->lock.code_nco += sv->lock.code_nco_step;
    if(sv->lock.code_nco >= CHIPS_PER_MS<<22)
        sv->lock.code_nco -= CHIPS_PER_MS<<22;
}

/*********************************************************************
* Take a snapshot of all the importing timing information, either for 
* generating a position, or for printing to the screen. This can also
* be written to a file for later analysis
*********************************************************************/
static void snapshot_timing(struct Snapshot *s) {
    int i;
    static int heading = 0;
    
    
    /* Clear out the snapshot structure */
    memset(s,0,sizeof(struct Snapshot));
    s->sample_count_h = sample_count>>32;
    s->sample_count_l = sample_count & 0xFFFFFFFF;
    
    
    if(heading == 0) {
       printf("Id,  State, Orbt, Time, WeekNo, FrameOfWeek, msOfFrame, ChipOfCode, fracOfChip\n");
    }
    heading++;
    if(heading == 49)
        heading = 0;
    if(snapshot_file != NULL) {
        for(i = 0; i < N_SV; i++) {
            struct Space_vehicle *sv;
            sv = space_vehicles+i;
            s->entries[i].id = sv->id;
            switch(sv->state) {
                case state_acquiring:
                    s->entries[i].state |= SNAPSHOT_STATE_ACQUIRE;
                    break;
                case state_tracking:
                    s->entries[i].state |= SNAPSHOT_STATE_TRACK;
                    break;
                case state_locked:
                    s->entries[i].state |= SNAPSHOT_STATE_LOCKED;
                    break;
                default:
                    break;
            }
            if(sv->nav_orbit.orbit_valid) 
                s->entries[i].state |= SNAPSHOT_STATE_ORBIT_VALID;

            if(sv->nav_time.time_good) 
                s->entries[i].state |= SNAPSHOT_STATE_TIME_VALID;

            s->entries[i].lock_ms_of_frame      = sv->lock.ms_of_frame;
            s->entries[i].lock_ms_of_bit        = sv->lock.ms_of_bit;
            s->entries[i].lock_code_nco         = sv->lock.code_nco;
            s->entries[i].nav_week_no           = sv->nav_time.week_no;
            s->entries[i].nav_subframe_of_week  = sv->navdata.subframe_of_week;
            s->entries[i].nav_subframe_in_frame = sv->navdata.subframe_in_frame;
            s->entries[i].nav_valid_bits        = sv->navdata.valid_bits;
        }
        fwrite(s, sizeof(struct Snapshot), 1, snapshot_file);
    }
#if SHOW_TIMING_SNAPSHOT_DETAILS            
    for(i = 0; i < N_SV; i++) {
        struct Space_vehicle *sv;
        int id;
        sv = space_vehicles+i;
        id = sv->id;
        printf("%02i,",id);
        switch(sv->state) {
            case state_acquiring:
                printf("ACQUIRE, ");
                break;
            case state_tracking:
                printf("  TRACK, ");
                break;
            case state_locked:
                printf(" LOCKED, ");
                break;
            default:
                printf(" ??????, ");
                break;
        }
        if(sv->nav_orbit.orbit_valid) 
            printf("YES,");
        else
            printf(" NO,");

        if(sv->nav_time.time_good) 
            printf("YES,");
        else
            printf(" NO,");

        printf(" %6i,      %6i,      %4i,       %4i,         %2i %1i  %2i %2i %c%c%c%c%c\n",
            sv->nav_time.week_no,
            sv->navdata.subframe_of_week,
            sv->lock.ms_of_frame,
            (sv->lock.code_nco>>22),
            (sv->lock.code_nco>>16)&0x3f,
            sv->navdata.subframe_in_frame,
            sv->navdata.valid_bits-2,
            sv->lock.ms_of_bit,
            (sv->navdata.valid_subframe[1] ? '1' : '.'),
            (sv->navdata.valid_subframe[2] ? '2' : '.'),
            (sv->navdata.valid_subframe[3] ? '3' : '.'),
            (sv->navdata.valid_subframe[4] ? '4' : '.'),
            (sv->navdata.valid_subframe[5] ? '5' : '.')
            );
    }
    printf("\n");
#endif
}

/*********************************************************************
* Process samples as they come in. Each space vehicle can be in one of
* three states:
*
* Acquring - Looking for the SV's signature at different alignments
*            and frequencies - Very resounce heavy.
*
* Tracking - Finding out exactly which alignment and frequncy is the
*            the best, allowing a good guess of frequency and code
*            alignment. 
*
* Locked   - Following the carrier and extracting the BSPK data.
*********************************************************************/
static void gps_process_sample(int s) {
  int sv;
  static int prime = 0;
  static int processed = 0;
  add_to_bitmap(sample_history,s);
  if(prime < samples_for_acquire) {
     if(prime == 0) {
       printf("Starting to prime sample history\n");
     }
     prime++;
     if(prime == samples_for_acquire) {
       printf("History primed with %i samples (%i milliseconds of data)\n", samples_for_acquire, ms_for_acquire);
     }
     return;
  }

  for(sv = 0; sv < N_SV; sv++) {
    int prompt_code_index, prompt_code_index_next_cycle;
    switch(space_vehicles[sv].state) {
       case state_acquiring:
            if(processed < samples_per_ms+100)
                acquire(space_vehicles+sv);
            break;
       case state_tracking:
            track(space_vehicles+sv);
            break;
       case state_locked:
            prompt_code_index            = space_vehicles[sv].lock.code_nco>>22;
            prompt_code_index_next_cycle = (space_vehicles[sv].lock.code_nco + space_vehicles[sv].lock.code_nco_step)>>22;

            accumulate(space_vehicles+sv, s);

            /* Are we about to flip over to a new code chip? */
            if(prompt_code_index != prompt_code_index_next_cycle) {

              if(prompt_code_index == CHIPS_PER_MS-2) 
                 update_early(space_vehicles+sv, prompt_code_index, prompt_code_index_next_cycle);

              if(prompt_code_index == CHIPS_PER_MS-1) 
                update_prompt(space_vehicles+sv,  prompt_code_index, prompt_code_index_next_cycle);

              if(prompt_code_index == 0) 
                 update_late(space_vehicles+sv, prompt_code_index, prompt_code_index_next_cycle);

              if(prompt_code_index == 1)
                adjust_early_late(space_vehicles+sv);
            }
            update_ncos(space_vehicles+sv);
            break;
       default:
            space_vehicles[sv].state = state_acquiring;
            break;
    }
  }
  if(code_offset_in_ms>0)
     code_offset_in_ms--;
  else
     code_offset_in_ms = samples_per_ms-1;
  processed++;

#if PRINT_SAMPLE_NUMBERS
  if(processed % PRINT_SAMPLE_FREQUENCY == 0) 
     {
        int i, locked = 0;
        for(i = 0; i < N_SV; i++)
            if(space_vehicles[i].state == state_locked)
                locked++;
        printf("Processing sample %i,   %i locked\n",processed);
     }
 #endif
#if PRINT_LOCKED_NCO_VALUES

  if(processed % samples_per_ms == 0) 
    {
    int i;
    for(i = 0; i < N_SV; i++)
        printf("%4i.%06i ",space_vehicles[i].lock_code_nco>>22, (space_vehicles[i].lock_code_nco>>16 & 0x3f)* 15625);
    putchar('\n');
    }
#endif
 
 if(processed % (samples_per_ms*SHOW_SOLUTION_MS) == 0) {
     struct Snapshot s;
     snapshot_timing(&s);
     attempt_solution(&s);
 } else if(processed % (samples_per_ms*SHOW_TIMING_SNAPSHOT_FREQ_MS) == 0) {
     struct Snapshot s;
     snapshot_timing(&s);
 }
}
/**********************************************************************
* Generate the G1 LFSR bit stream
**********************************************************************/
static void g1_lfsr(unsigned char *out) {
  int state = 0x3FF,i;
  for(i = 0; i < 1023; i++) {
    int new_bit;
    out[i]   = (state >>9) & 0x1;
    /* Update the G1 LFSR */
    new_bit = ((state >>9) ^ (state >>2))&1;
    state   = ((state << 1) | new_bit) & 0x3FF;
  }
}

/**********************************************************************
* Generate the G2 LFSR bit stream. Different satellites have different
* taps, which effectively alters the relative phase of G1 vs G2 codes
**********************************************************************/
static void g2_lfsr(unsigned char tap0, unsigned char tap1, unsigned char *out) {
  int state = 0x3FF,i;
  /* Adjust tap number from 1-10 to 0-9 */
  tap0--;
  tap1--;
  for(i = 0; i < 1023; i++) {
    int new_bit;

    out[i] = ((state >> tap0) ^ (state >> tap1)) & 0x1;

    /* Update the G2 LFSR  */
    new_bit = ((state >>9) ^ (state >>8) ^
               (state >>7) ^ (state >>5) ^
               (state >>2) ^ (state >>1))&1;
    state = ((state << 1) | new_bit) & 0x3FF;
  }
}

/**********************************************************************
* Combine the G1 and G2 codes to make each satellites code
**********************************************************************/
static void combine_g1_and_g2(unsigned char *g1, unsigned char *g2, unsigned char *out)
{
  int i;
  for(i = 0; i < 1023; i++ ) {
    out[i] = g1[i] ^ g2[i];
  }
}

/*********************************************************************
* Build the Gold codes for each Satellite from the G1 and G2 streams
*********************************************************************/
void generateGoldCodes(void)
{
  static unsigned char g1[1023];
  static unsigned char g2[32][1023];
  int sv;
  g1_lfsr(g1);
  
  for(sv = 0; sv < N_SV; sv++) {
    int i;

    printf("Calculating Gold Code for SV %i\n",space_vehicles[sv].id);
    g2_lfsr(space_vehicles[sv].tap1, space_vehicles[sv].tap2, g2[sv]);
    combine_g1_and_g2(g1, g2[sv], space_vehicles[sv].gold_code);
    for(i = 0; i < 1023; i++) {
      if(space_vehicles[sv].gold_code[i])
        printf("1, ");
      else
        printf("0, ");
      if(i%31 == 30)
       printf("\n");
    } 
  }
}

/********************************************
* Stretch the C/A code out to the required 
* of bits to cover the acquision test 
********************************************/
static void stretchGoldCodes(void) {
  int i;
  for(i = 0; i < N_SV; i++) {
    int j;
    for(j = 0; j < samples_for_acquire; j++) {
       int index = (j*1023/samples_per_ms)%1023;
       bitmap_set_bit(space_vehicles[i].acquire.gold_code_stretched, 
                      samples_for_acquire-1 - j,
                      space_vehicles[i].gold_code[index]);
    }
  }
}

/********************************************
* Mix the G/A codes with the different LO
* freqencies, to give the final bitmap used
* to humt for the space vehicles
*******************************************/
static void mixLocalOscAndGoldCodes(unsigned sample_freq, unsigned if_freq) {
  int band;
  sample_freq /= 1000;
  if_freq     /= 1000;
  for(band = 0; band < search_bands; band++) {        
    int i, p, s;
    /************************************
    * Step for phase accumulator for this 
    * band
    ************************************/
    s = samples_for_acquire*if_freq/sample_freq + band - search_bands/2;
    /**********************************
    * Calculate IF inphase bitmap using 
    * a phase accumulator
    **********************************/
    p = 0;
    for(i = 0; i < samples_for_acquire; i++) {
       if(p < samples_for_acquire/2)
         bitmap_set_bit(work_buffer, samples_for_acquire-1 - i, 1); 
       else
         bitmap_set_bit(work_buffer, samples_for_acquire-1 - i, 0); 
       p += s;       
       if(p >= samples_for_acquire) p-= samples_for_acquire;
    }

    for(i = 0; i < N_SV; i++) {
       struct Space_vehicle *sv;
         sv = space_vehicles+i;
       xor_to_work(sv->acquire.seek_in_phase[band], work_buffer, sv->acquire.gold_code_stretched);
#if 0
           printf("\n\n");
           print_bitmap(sv->seek_in_phase[band], samples_for_acquire);
           printf("^^^^^^^^^^^^^^^^^^^\n");
           print_bitmap(sv->gold_code_stretched, samples_for_acquire);
           printf("===================\n");
           print_bitmap(sv->seek_in_phase[band], samples_for_acquire);
#endif
    }

    /**********************************
    * Calculate IF quadrature bitmap 
    * using a phase accumulator
    **********************************/
    p = samples_for_acquire/4;
    for(i = 0; i < samples_for_acquire; i++) {
       if(p < samples_for_acquire/2)
         bitmap_set_bit(work_buffer, samples_for_acquire-1 - i, 1); 
       else
         bitmap_set_bit(work_buffer, samples_for_acquire-1 - i, 0); 
     
       p += s;
       if(p >= samples_for_acquire) p-= samples_for_acquire;
    }
    for(i = 0; i < N_SV; i++) {
       struct Space_vehicle *sv;
       sv = space_vehicles+i;
       xor_to_work(sv->acquire.seek_quadrature[band], work_buffer, sv->acquire.gold_code_stretched);
    }
  }
}

/*********************************************************************
* gps_setup() - Allocate and initialise the data structures
*********************************************************************/
int gps_setup(int sample_rate, int if_freq) {
  int i, band;
  /****************************************
  * Calculate some 'almost' constants
  ****************************************/
  if(sample_rate % 1000 != 0) { 
     printf("Sample rate must be a multiple of 1000\n");
     return 0;
  }
  band_bandwidth          = 1000/ms_for_acquire;  // Hz
  search_bands            = 5000/band_bandwidth*2+1;  // For +/- 5kHz
  samples_per_ms          = sample_rate / 1000;
  code_offset_in_ms       = samples_per_ms-1;
  samples_for_acquire     = samples_per_ms * ms_for_acquire;
  acquire_min_power       = ms_for_acquire * ms_for_acquire * samples_per_ms * 2; /*  needs tuning */
  track_unlock_power      = ms_for_acquire * ms_for_acquire * samples_per_ms;     /*  needs tuning */
  lock_lost_power         = samples_per_ms*samples_per_ms/250000; /* This is over 1 ms  - needs tuning */  
  acquire_bitmap_size_u32 = (samples_for_acquire+31)/32;
  if_cycles_per_ms        = if_freq/1000;
  printf("bitmaps are %i 32-bit words in size\n",  acquire_bitmap_size_u32);
  printf("Seaching %i bands of %i Hz wide\n",search_bands, band_bandwidth);
  printf("Acquire min power %u\n",acquire_min_power);
  printf("Track lost power  %u\n",track_unlock_power);
  printf("Lock lost power   %u\n",lock_lost_power);
  /*********************************
  * Allocate memory that is used 
  * during the acquire phase
  **********************************/
  printf("Allocating memory\n");
  sample_history = malloc(acquire_bitmap_size_u32*4);
  if(sample_history == NULL) {
    printf("Out of memory for history\n");
    return 0;
  }

  work_buffer = malloc(acquire_bitmap_size_u32*4);
  if(work_buffer == NULL) {
    printf("Out of memory for history\n");
    return 0;
  }

  /*************************************************
  * Allocate per-space vehicle memory.
  *
  * As well as a stretched copy of the gold cold we
  * need space for the code mixed with the different
  * possible intermendiate frequency bitmmaps
  *************************************************/
  printf("Allocating memory for %i Space Vehicles\n", (int)N_SV);
  for(i = 0; i < N_SV; i++) {
    struct Space_vehicle *sv;
    sv = space_vehicles+i;

    sv->acquire.gold_code_stretched = malloc(acquire_bitmap_size_u32*4);
    if(sv->acquire.gold_code_stretched == NULL) {
        printf("Out of memory for gold_code_stretched\n");
        return 0;
    }

        sv->acquire.seek_in_phase       = malloc(sizeof(uint_32 *)*search_bands);
    if(sv->acquire.seek_in_phase == NULL) {
        printf("Out of memory for seek_in_phase\n");
        return 0;
    }

    sv->acquire.seek_quadrature     = malloc(sizeof(uint_32 *)*search_bands);
    if(sv->acquire.seek_quadrature == NULL) {
        printf("Out of memory for seek_quadrature\n");
        return 0;
    }

    for(band = 0; band < search_bands; band++) {        
        sv->acquire.seek_in_phase[band]            = malloc(acquire_bitmap_size_u32*4);
        if(sv->acquire.seek_in_phase[band]==NULL) {
            printf("Out of memory for sv->seek_in_phase[]\n");
            return 0;            
        }

        sv->acquire.seek_quadrature[band] = malloc(acquire_bitmap_size_u32*4);
        if(sv->acquire.seek_quadrature[band]==NULL) {
            printf("Out of memory for sv->seek_quadrature[]\n");
            return 0;            
        }
    }
  }
  
  /*************************************************
  * Now calcluate the Gold code and and populate 
  *************************************************/
  printf("Calculating Gold Codes\n");
  generateGoldCodes();
  printf("Stretching Gold Codes\n");
  stretchGoldCodes();
  printf("Mixing Gold Codes\n");
  mixLocalOscAndGoldCodes(sample_rate, if_freq);
  printf("Setup completed\n");
  return 1;
}
/*********************************************************************
* Generate the atan2 table
*********************************************************************/
void generate_atan2_table(void) {
    int x,y;
    printf("Generating ATAN2 table\n");
    for(x = 0; x < ATAN2_SIZE; x++) {
        for(y = 0; y < ATAN2_SIZE; y++) {
            atan2_lookup[y][x] = 0;
        }
    }
    
    for(x = -(ATAN2_SIZE-1)/2; x <=ATAN2_SIZE/2-1; x++) {
        for(y = -(ATAN2_SIZE-1)/2; y <= ATAN2_SIZE/2-1; y++) {
            double a = atan2(x,y);
            a *= 256.0 / 3.141592;
            a += 0.5;
            if(a < 0) 
                a +=256;
            atan2_lookup[y&(ATAN2_SIZE-1)][x&(ATAN2_SIZE-1)] = a;
        }
    }
#if PRINT_ATAN2_TABLE
    for(x = 0; x < ATAN2_SIZE; x++) {
        for(y = 0; y < ATAN2_SIZE; y++) {
            printf("%5i,", atan2_lookup[y][x]);
        }
        putchar('\n');
    }
#endif    
}

/*********************************************************************
* Usage message
*********************************************************************/
static void usage(char *message) {
  printf("%s\n", message);
  printf("Usage: gps -s sample_rate -i if_freq -o offset filename\n");
  exit(1);
}

/*********************************************************************
* Main program - check parameters, open file and process samples
*********************************************************************/
int main(int argc, char *argv[]) {
  int upto = 1;
  int sample_rate = 16368000, if_freq = 4092000, offset = 0;
  FILE *f;
  char *filename = NULL;
  int c;

  if(argc == 1) {
    usage("Must supply args");
    return 0;
  }

  while(upto < argc) {
     if(argv[upto][0] == '-') {
       if(strlen(argv[upto]) != 2) {
         usage("Only single character switches are allowed\n");
         return 0;
       }    

       if(upto == argc-1) {
         usage("Missing argument for switch");
         return 0;
       }

       switch(argv[upto][1]) {
         case 's':  sample_rate = atoi(argv[upto+1]);
                    break;
         case 'i':  if_freq     = atoi(argv[upto+1]);
                    break;
         case 'o':  offset      = atoi(argv[upto+1]);
                    break;
         default:   usage("Unknown switch");
                    return 0;
       }
       upto += 2;
     } else {
       if(filename != NULL) {
         usage("No file name supplied");
         return 0;
       }
       filename = argv[upto];
       upto++;
     }  
  }
  printf("Sample rate:            %i\n", sample_rate);
  printf("Intermediate Frequency: %i\n", if_freq);
  printf("Offset:                 %i\n", offset);
  printf("Filename:               %s\n", filename);
  if(sample_rate%1000 != 0) {
     usage("Sample rate must be divisible by 1000");
     return 0;
  } 

  f = fopen(filename, "rb");
  if(f == NULL) {
    printf("Unable to open file '%s'\n",filename);
  }

#if LOG_TIMING_SNAPSHOT_TO_FILE
  {
    char snapshot_filename[40];
    sprintf(snapshot_filename,"snapshots_%u.dat",(unsigned)time(NULL));
    snapshot_file = fopen(snapshot_filename, "wb");
    if(snapshot_file == NULL) {
        snapshot_file = fopen(snapshot_filename, "w+b");
    }
    if(snapshot_file == NULL) {
        printf("Unable to open file '%s'\n",snapshot_filename);
    }
  }
#endif

#if LOG_POSITION_FIX_TO_FILE
  char position_filename[40];
  sprintf(position_filename,"position_%u.dat",(unsigned)time(NULL));
  position_file = fopen(position_filename, "wb");
  if(position_file == NULL) {
        position_file = fopen(position_filename, "w+b");
  }
  if(position_file == NULL) {
        printf("Unable to open file '%s'\n",position_filename);
  }
#endif
  /***********************************
  * Setup the internal data structures
  ***********************************/
  gps_setup(sample_rate,if_freq);
  generate_atan2_table();
  nav_read_in_all_cached_data();
  /***********************************
  * Process all the bits - LSB first 
  ***********************************/
  printf("Processing samples\n");
  c = getc(f);
  while(c != EOF) {
    int i;
    for(i =0; i < 8; i++) {
      if(c&1)
        gps_process_sample(1);
      else
        gps_process_sample(0);   
      sample_count++;
      c >>= 1;
    }
    c = getc(f);
  }
  if(snapshot_file)
    fclose(snapshot_file);
  if(position_file)
    fclose(position_file);
  fclose(f);
  printf("%lli samples processed\n",sample_count);
  return 0;
}
/*********************************************************************
* END OF FILE
*********************************************************************/
