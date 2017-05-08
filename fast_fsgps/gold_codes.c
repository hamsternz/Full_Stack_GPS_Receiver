/****************************************************************************
* gold_codes.c - Generating the GPS Gold codes
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

struct Space_vehicle {
   uint_8 sv_id;
   uint_8 tap1;
   uint_8 tap2;
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

uint_32 gold_codes_32_cycles[MAX_SV_ID+1][1023*16];
uint_8 gold_codes[MAX_SV_ID+1][1023];

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
static void generate_gold_codes(void) {
  int sv;
  static unsigned char g1[1023];
  static unsigned char g2[1023];
  g1_lfsr(g1);
  for(sv = 0; sv < sizeof(space_vehicles)/sizeof(struct Space_vehicle); sv++) {
    g2_lfsr(space_vehicles[sv].tap1, space_vehicles[sv].tap2, g2);
    combine_g1_and_g2(g1, g2, gold_codes[ space_vehicles[sv].sv_id]);
  }
}

/*********************************************************************
* Build the Gold code early/prompt/late table
*********************************************************************/
static void generate_gold_codes_32_cycles(void) {
  int sv;
  for(sv = 1; sv <= MAX_SV_ID; sv++) {
    int i;
    for(i = 0; i < 1023*16; i++) {
      int j;
      uint_32 t = 0;
      for(j = 0; j < 32; j++) { 
        t = t << 1;
        if(gold_codes[sv][((i+j)>>4)%1023])
          t |= 1;
      }
      gold_codes_32_cycles[sv][i] = t;
    }
  }
}

/************************************************
*
************************************************/
void gold_code_startup(void) {
   generate_gold_codes();
   generate_gold_codes_32_cycles();
}
/******************************************************
* END OF FILE
******************************************************/
