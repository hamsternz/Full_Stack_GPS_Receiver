#include <stdio.h>
#include <memory.h>
#include <math.h>
#include <assert.h>

typedef unsigned char uint_8;
typedef unsigned int  uint_32;
typedef int           int_32;
typedef char          int_8;

#define MS_PER_BIT  20
#define LATE_EARLY_IIR_FACTOR      8
#define LOCK_DELTA_IIR_FACTOR       8
#define LOCK_ANGLE_IIR_FACTOR       8



#define ATAN2_SIZE 128
uint_8 atan2_lookup[ATAN2_SIZE][ATAN2_SIZE];

#if 0
//////////////////////////////////////////////////////////////////////////
// 26: Lower band     679 upper band     657 Adjust   16   Freq guess 3984
// 
// Lock band 18, Lock offset 389, step 400fed60,  Code NCO        0
// lock_phase_nco_step  400fed60
// lock code_nco & step       80000      40000 10836
//////////////////////////////////////////////////////////////////////////
uint_8 gold_code[1023] = {
1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1,
0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1,
1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0,
0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0,
1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1,
0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0,
0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1,
1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0,
1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1,
1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1,
0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1,
1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1,
1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1,
1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1,
1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1,
1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1,
1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1,
0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1,
1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0,
1, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1,
1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0,
0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1,
0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0,
0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1,
1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1,
0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0,
1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1,
0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0,
0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0,
0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0,
0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0
};
#endif
//////////////////////////////////////////////////////////////////////////
// 32: Lower band     491 upper band     513 Adjust   21   Freq guess -1521
// 
// Lock band 7, Lock offset 11438, step 3ff9eb5a,  Code NCO        0
// lock_phase_nco_step  3ff9eb5a
// lock code_nco & step       80000      40000 -4137
//////////////////////////////////////////////////////////////////////////

uint_8 gold_code[1023] = {
1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0,
1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0,
0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0,
0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1,
0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1,
0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0,
0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1,
1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1,
0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0,
0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1,
1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1,
1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1,
0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0,
0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0,
0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0,
1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0,
1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1,
0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1,
1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1,
0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1,
0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1,
0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1,
0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1,
0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1,
1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1,
0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1,
1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0,
1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1,
0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0
};

/*********************************************************************
* Generate the atan2 table
*********************************************************************/
void generate_atan2_table(void) {
    int x,y;
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
/************************************************
*
************************************************/
int count_ones(uint_32 a) {
  int rtn;
  static int setup = 1; 
  static unsigned char ones_lookup[256];
  if(setup) {
    int i;
    for(i = 0; i < 256; i++) {
      int j;
      for(j = 0; j < 32; j++) {
        if(i&(1<<j))
          ones_lookup[i]++;
      }
    } 
    setup = 0;
  }
  rtn  = ones_lookup[(a)     & 0xFF];
  rtn += ones_lookup[(a>>8)  & 0xFF];
  rtn += ones_lookup[(a>>16) & 0xFF];
  rtn += ones_lookup[(a>>24) & 0xFF];
  return rtn;
}
/************************************************
*
************************************************/
static uint_32 early_code,  early_mask;
static uint_32 prompt_code, prompt_mask;
static uint_32 late_code,   late_mask;
static int early_end_of_repeat;
static int prompt_end_of_repeat;
static int late_end_of_repeat;
void fast_code_nco(uint_8 *gc, uint_32 nco, uint_32 step) {
  int i,n0,n1,n2;
  int wrap0=0, wrap1=0, wrap2=0, wrap3=0;
  uint_8 codeSub1, code0, code1, code2, code3;
  n0 = 1;
  i = nco>>22;

  if(i == 0) {
    codeSub1 = gc[1022];
    wrap0    = 1;
  } else
    codeSub1 = gc[i - 1];


  code0 = gc[i];
  i++;
  if(i > 1022) { i-=1023; wrap1 = 1; }
  code1 = gc[i];
  i++;
  if(i > 1022) { i-=1023; wrap2 = 1; }
  code2 = gc[i];
  i++;
  if(i > 1022) { i-=1023; wrap3 = 1; }
  code3 = gc[i];
  
  
  /* Work out how many bits of each code chip */
  n0 = n1 = n2 = 0;  
  
  nco &= ((1<<22)-1);
  while(nco < (1<<22)) {
	  nco += step;
	  n0++;
  }

  while((nco < (2<<22)) && n0+n1 < 32) {
	  nco += step;
	  n1++;
  }
  n2 = 32 - (n0 + n1);
  late_code = prompt_code = early_code = 0;

  /***************************************************
  * First code bit in results 
  ***************************************************/ 
  if(codeSub1) late_code   = 0xFFFFFFFF;
  if(code0)    prompt_code = 0xFFFFFFFF;
  if(code1)    early_code  = 0xFFFFFFFF;


  /***************************************************
  * Second code bit in results 
  ***************************************************/ 
  late_code   <<= n1;
  prompt_code <<= n1;
  early_code  <<= n1;
  if(code0) late_code   |= (1<<n1)-1;
  if(code1) prompt_code |= (1<<n1)-1;
  if(code2) early_code  |= (1<<n1)-1;

  /***************************************************
  * Third (and not always present) code bit in results 
  ***************************************************/ 
  if(n2 > 0) {
	late_code   <<= n2;
	prompt_code <<= n2;
	early_code  <<= n2;
#if 1
	if(code1) late_code   |= (1<<n2)-1;
	if(code2) prompt_code |= (1<<n2)-1;
	if(code3) early_code  |= (1<<n2)-1;
#endif
  }

  early_end_of_repeat  = 0;
  prompt_end_of_repeat = 0;
  late_end_of_repeat   = 0;
  early_mask  = 0;
  prompt_mask = 0;
  late_mask   = 0;

  /****************************************************
  * Now to generate the masks for when we have wrapped
  ****************************************************/
  if(wrap0) {
	  late_mask = 0xFFFFFFFF;
	  late_mask >>= n0;
          late_end_of_repeat = 1;
  } else if(wrap1 && (nco >= (2<<22))) {
	  late_mask = 0xFFFFFFFF;
	  late_mask >>= n0;
	  late_mask >>= n1;
          late_end_of_repeat = 1;
  }

  if(wrap1) {
	  prompt_mask = 0xFFFFFFFF;
	  prompt_mask >>= n0;
          prompt_end_of_repeat = 1;
//          printf("A %i %08X\n",n0,prompt_mask);
  } else if(wrap2 && (nco >= (2<<22))) {
	  prompt_mask = 0xFFFFFFFF;
	  prompt_mask >>= n0;
	  prompt_mask >>= n1;
//          printf("A %i %08X\n",n0,prompt_mask);
          prompt_end_of_repeat = 1;
  }

  if(wrap2) {
	  early_mask = 0xFFFFFFFF;
	  early_mask >>= n0;
          early_end_of_repeat = 1;
  } else if(wrap3 && (nco >= (2<<22))) {
	  early_mask = 0xFFFFFFFF;
	  early_mask >>= n0;
	  early_mask >>= n1;
          early_end_of_repeat = 1;
  }
}

/************************************************
*
************************************************/
void fast_IF_nco_mask_8(uint_32 nco, uint_32 step, uint_32 *s, uint_32 *c) {
  int j;
  uint_32 nco_in_8;
  nco_in_8 = nco + step*8;
  if( ((nco^nco_in_8) & 3<<30) == 0) {
    switch(nco>>30) {
      case 0: *s = (*s>>8) | 0x33000000;
              *c = (*c>>8) | 0x99000000;
              break;
      case 1: *s = (*s>>8) | 0x99000000;
              *c = (*c>>8) | 0xCC000000;
              break;
      case 2: *s = (*s>>8) | 0xCC000000;
              *c = (*c>>8) | 0x66000000;
              break;
      case 3: *s = (*s>>8) | 0x66000000;
              *c = (*c>>8) | 0x33000000;
              break;
    }
    return;
  }
  for(j = 0; j < 8; j++) {
     switch(nco>>30) {
       case 0: *s = (*s >> 1) | (1<<31);
               *c = (*c >> 1) | (1<<31);
               break;
       case 1: *s = (*s >> 1) | (1<<31);
               *c = (*c >> 1) | (0<<31);
               break;
       case 2: *s = (*s >> 1) | (0<<31);
               *c = (*c >> 1) | (0<<31);
               break;
       case 3: *s = (*s >> 1) | (0<<31);
               *c = (*c >> 1) | (1<<31);
               break;
     }
     nco += step;
  }
}

void fast_IF_nco_mask_16(uint_32 nco, uint_32 step, uint_32 *s, uint_32 *c) {
  uint_32 nco_in_16;

  nco_in_16 = nco + step*16;
  if(((nco^nco_in_16) & 3<<30) == 0) {
    switch(nco>>30) {
      case 0: *s = (*s>>16) | 0x33330000;
              *c = (*c>>16) | 0x99990000;
              break;
      case 1: *s = (*s>>16) | 0x99990000;
              *c = (*c>>16) | 0xCCCC0000;
              break;
      case 2: *s = (*s>>16) | 0xCCCC0000;
              *c = (*c>>16) | 0x66660000;
              break;
      case 3: *s = (*s>>16) | 0x66660000;
              *c = (*c>>16) | 0x33330000;
              break;
    }
    return;
  } 
    fast_IF_nco_mask_8(nco, step, s, c);
    nco += step * 8;
    fast_IF_nco_mask_8(nco, step, s, c);
}

void fast_IF_nco_mask(uint_32 nco, uint_32 step, uint_32 *s, uint_32 *c) {
  uint_32 nco_in_32;

  nco_in_32 = nco+step*32;
  if( ((nco^nco_in_32) & 3<<30) == 0) {
    switch(nco>>30) {
      case 0: *s = 0x33333333;
              *c = 0x99999999;
              break;
      case 1: *s = 0x99999999;
              *c = 0xCCCCCCCC;
              break;
      case 2: *s = 0xCCCCCCCC;
              *c = 0x66666666;
              break;
      case 3: *s = 0x66666666;
              *c = 0x33333333;
              break;
    }
    return;
  }
  fast_IF_nco_mask_16(nco, step, s, c);
  nco += step*16;
  fast_IF_nco_mask_16(nco, step, s, c);
}

struct Channel {
   uint_32 nco_if;
   uint_32 step_if;

   uint_32 nco_code;
   uint_32 step_code;
   uint_32 code_tune;

   int_32 early_sine_count,  early_cosine_count,  early_sample_count;
   int_32 prompt_sine_count, prompt_cosine_count, prompt_sample_count;
   int_32 late_sine_count,   late_cosine_count,   late_sample_count;

   uint_32 early_power_filtered; 
   uint_32 prompt_power_filtered; 
   uint_32 late_power_filtered; 

   uint_32 early_power_filtered_not_reset; 
   uint_32 prompt_power_filtered_not_reset; 
   uint_32 late_power_filtered_not_reset; 

   int prompt_sc_temp;
   int prompt_cc_temp;

   uint_8 last_angle;
   int_32 delta_filtered;
   int_32 angle_filtered;
   uint_8 ms_of_bit;
   uint_8 last_bit;
   uint_8 no_adjust;
} channel;

static void adjust_prompt(struct Channel *ch) {
    int s, c;
    int_8 delta;
    int adjust = 0;
    uint_8 angle;
    uint_8 this_bit;

    s = ch->prompt_sine_count;
    c = ch->prompt_cosine_count;
    while(c > 15 || c < -15 || s > 15 || s < -15) {
        c /= 2;
        s /= 2;
    }

    s &= ATAN2_SIZE-1;  c &= ATAN2_SIZE-1;
    angle = atan2_lookup[c][s];
    delta = angle -ch->last_angle;
    ch->last_angle = angle;

    ch->delta_filtered -= ch->delta_filtered / LOCK_DELTA_IIR_FACTOR;
    ch->delta_filtered += delta;

    adjust = angle;
    ch->angle_filtered -= ch->angle_filtered / LOCK_ANGLE_IIR_FACTOR;
    ch->angle_filtered += angle;

    if(angle >=128)
        ch->angle_filtered -= 256;

    adjust  = ch->angle_filtered/8;
    adjust  += (1<<24) / 32 / LOCK_DELTA_IIR_FACTOR / 16368 * ch->delta_filtered;
    ch->step_if  -= adjust;
#if LOCK_SHOW_ANGLES
    printf("%6i, %6i, %3i,%4i,%6i, %6i, %6i\n",sv->lock.prompt_sine_power, sv->lock.prompt_cosine_power, angle,delta, sv->lock.delta_filtered, adjust, sv->lock.phase_nco_step);
#endif

    if(ch->prompt_cosine_count < 0)
        this_bit = 0;
    else
        this_bit = 1;
    ch->ms_of_bit++;
    if(ch->ms_of_bit == MS_PER_BIT || ch->last_bit != this_bit)
        ch->ms_of_bit = 0;
    ch->last_bit = this_bit;

#if 0
    /* WIll need to be moved outside of channel */
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
#endif
//    nav_process(sv,this_bit);

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
/************************************************
*
************************************************/
int main(int argc, char *argv[]) {
   FILE *f;
   struct Channel *c;
   int q = 0;

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
   generate_atan2_table();

   /* Set the start values */
   memset(c,0,sizeof(struct Channel));
   c->nco_if    = 0;
   c->step_if   = 0x40104800;
   c->nco_code  = ((382)<<18);
   c->step_code = 0x00040000;
   c->code_tune = 10880;

   c->nco_if    = 0;
   c->step_if   = 0x3ff9eb5a;
   c->nco_code  = ((11438)<<18);
   c->step_code = 0x00040000;
   c->code_tune = -4137;

//////////////////////////////////////////////////////////////////////////
// 32: Lower band     491 upper band     513 Adjust   21   Freq guess -1521
// 
// Lock band 7, Lock offset 11438, step 3ff9eb5a,  Code NCO        0
// lock_phase_nco_step  3ff9eb5a
// lock code_nco & step       80000      40000 -4137
//////////////////////////////////////////////////////////////////////////

   for(q = 0; q < 16368*1000/32*20; q++) {
     uint_32 data;
     uint_32 mixed_sine, mixed_cosine;
     uint_32 s_intermediate_freq, c_intermediate_freq;
     int ch;

     /* Read the data */
     ch  = getc(f);
     if(ch == EOF) break;
     data  = ch<<24;
     ch  = getc(f);
     if(ch == EOF) break;
     data |= ch<<16;
     ch = getc(f);
     if(ch == EOF) break;
     data |= ch<<8;
     ch  = getc(f);
     if(ch == EOF) break;
     data |= ch;

     /* Generate the intermediate frequency bitmaps
        and advance the NCO     */ 
     fast_IF_nco_mask(c->nco_if, c->step_if, &s_intermediate_freq, &c_intermediate_freq);
     c->nco_if   += c->step_if   * 32;

     /* Generate the gold codes for this set of samples
        and advance the NCO     */ 
     fast_code_nco(gold_code, c->nco_code, c->step_code);
     {
     int i;
     for(i = 0; i < 32; i++) {
         c->nco_code += c->step_code;
         if(c->nco_code >= (1023<<22))
           c->nco_code -= 1023<<22;
       } 
     }

     /*****************************
     * Process the early codes
     *****************************/ 
     /* Mix the signals */
     mixed_sine   = data ^ early_code ^ s_intermediate_freq; 
     mixed_cosine = data ^ early_code ^ c_intermediate_freq; 

     /* Count how many bits are set */
     c->early_sine_count   += count_ones(mixed_sine);
     c->early_cosine_count += count_ones(mixed_cosine);
     c->early_sample_count += 32;

     if(early_end_of_repeat) {
         int next_sine, next_cosine, next_sample_count;
         next_sine         = count_ones(mixed_sine   & early_mask);
         next_cosine       = count_ones(mixed_cosine & early_mask);
         next_sample_count = count_ones(early_mask);
         c->early_sample_count -= next_sample_count;
         c->early_sine_count   -= next_sine   + c->early_sample_count/2;
         c->early_cosine_count -= next_cosine + c->early_sample_count/2;
         c->early_power_filtered -= c->early_power_filtered/LATE_EARLY_IIR_FACTOR;
         c->early_power_filtered += c->early_sine_count*c->early_sine_count + c->early_cosine_count*c->early_cosine_count;
         c->early_power_filtered_not_reset -= c->early_power_filtered_not_reset/LATE_EARLY_IIR_FACTOR;
         c->early_power_filtered_not_reset += c->early_sine_count*c->early_sine_count + c->early_cosine_count*c->early_cosine_count;
         printf("%6i, %7i, ", q, c->early_power_filtered_not_reset);
         c->early_sine_count   = next_sine;
         c->early_cosine_count = next_cosine;
         c->early_sample_count = next_sample_count;
     }

     /*****************************
     * Process the prompt codes
     *****************************/ 
     /* Mix the signals */
     mixed_sine   = data ^ prompt_code ^ s_intermediate_freq; 
     mixed_cosine = data ^ prompt_code ^ c_intermediate_freq; 

     /* Count how many bits are set */
     c->prompt_sine_count   += count_ones(mixed_sine);
     c->prompt_cosine_count += count_ones(mixed_cosine);
     c->prompt_sample_count += 32;

     if(prompt_end_of_repeat) {
         int next_sine, next_cosine, next_sample_count;
         next_sine         = count_ones(mixed_sine   & prompt_mask);
         next_cosine       = count_ones(mixed_cosine & prompt_mask);
         next_sample_count = count_ones(prompt_mask);
         c->prompt_sample_count -= next_sample_count;
         c->prompt_sine_count   -= next_sine   + 16368/2; // c->prompt_sample_count/2;
         c->prompt_cosine_count -= next_cosine + 16368/2; // c->prompt_sample_count/2;
         c->prompt_power_filtered -= c->prompt_power_filtered/8;
         c->prompt_power_filtered += c->prompt_sine_count*c->prompt_sine_count + c->prompt_cosine_count*c->prompt_cosine_count;
         printf(" %7i, ", c->prompt_power_filtered);
         adjust_prompt(c);
         c->prompt_sc_temp = c->prompt_sine_count;
         c->prompt_cc_temp = c->prompt_cosine_count;
         c->prompt_sine_count   = next_sine;
         c->prompt_cosine_count = next_cosine;
         c->prompt_sample_count = next_sample_count;
     }

     /*****************************
     * Process the late codes
     *****************************/ 
     /* Mix the signals */
     mixed_sine   = data ^ late_code ^ s_intermediate_freq; 
     mixed_cosine = data ^ late_code ^ c_intermediate_freq; 

     /* Count how many bits are set */
     c->late_sine_count   += count_ones(mixed_sine);
     c->late_cosine_count += count_ones(mixed_cosine);
     c->late_sample_count += 32;
     if(late_end_of_repeat && c->no_adjust == 0) {
         int next_sine, next_cosine, next_sample_count;
         int adjust;

         next_sine         = count_ones(mixed_sine   & late_mask);
         next_cosine       = count_ones(mixed_cosine & late_mask);
         next_sample_count = count_ones(late_mask);
         c->late_sample_count -= next_sample_count;
         c->late_sine_count   -= next_sine   + c->late_sample_count/2;
         c->late_cosine_count -= next_cosine + c->late_sample_count/2;
         c->late_power_filtered -= c->late_power_filtered/LATE_EARLY_IIR_FACTOR;
         c->late_power_filtered += c->late_sine_count*c->late_sine_count + c->late_cosine_count*c->late_cosine_count;
         c->late_power_filtered_not_reset -= c->late_power_filtered_not_reset/LATE_EARLY_IIR_FACTOR;
         c->late_power_filtered_not_reset += c->late_sine_count*c->late_sine_count + c->late_cosine_count*c->late_cosine_count;
         printf(" %7i\n", c->late_power_filtered_not_reset);
         c->late_sine_count   = next_sine;
         c->late_cosine_count = next_cosine;
         c->late_sample_count = next_sample_count;

         /* Trim the NCO for the Gold Code */
         /* Use the relative power levels of the late and early codes
         * to adjust the code NCO phasing */
         adjust =  c->code_tune;
         if(c->early_power_filtered/5 > c->late_power_filtered/4) {
           c->early_power_filtered = (c->early_power_filtered*7+c->late_power_filtered)/8;
           adjust += 16368*1;
           c->code_tune+=2;
         } else if(c->late_power_filtered/5 > c->early_power_filtered/4) {
           c->late_power_filtered = (c->late_power_filtered*7+c->early_power_filtered)/8;
           adjust  -= 16368*1;
           c->code_tune-=2;
         }
         c->nco_code += adjust;

         c->no_adjust = 1;
     }
     else c->no_adjust = 0;
   }
   return q;
}
