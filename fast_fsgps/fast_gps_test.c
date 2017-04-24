#include <stdio.h>

typedef unsigned char uint_8;
typedef unsigned int  uint_32;

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

/************************************************
*
************************************************/
int count_ones(uint_32 a) {
  int i, rtn=0;
  for(i = 0; i < 32; i++) {
    if(a&1)
      rtn++;
    a >>= 1; 
  }
  return rtn;
}
/************************************************
*
************************************************/
int fast_code_nco(uint_8 *gc, uint_32 nco, uint_32 step, uint_32 *code, uint_32 *mask) {
  int i,n0,n1,n2,wrap1=0,wrap2=0;
  uint_8 code0, code1, code2;
  n0 = 1;
  i = nco>>22;
  code0 = gc[i];
  i++;
  if(i > 1022) { i-=1023; wrap1 = 1; }
  code1 = gc[i];
  i++;
  if(i > 1022) { i-=1023; wrap2 = 1; }
  code2 = gc[i];
  
  /* Work out how many bits of each code chip */
  n0 = n1 = n2 = 0;  
  
  nco &= ((1<<22)-1);
  while(nco < 1<<22) {
	  nco += step;
	  n0++;
  }

  while((nco < 2<<22) && n0+n1 < 32) {
	  nco += step;
	  n1++;
  }
  n2 = 32 - (n0 + n1);
 
  if(code0)
    *code = 0xFFFFFFFF << n1;
  else
    *code = 0;

  if(code1)
	*code |= (1<<n1)-1;

  if(n2 > 0) {
	*code <<= n2;
	if(code2)
		*code |= (1<<n2)-1;
  }

  if(wrap1) {
	  *mask = 0xFFFFFFFF;
	  *mask >>= n0;
	  return 1;
  }
  
  if(wrap2 && (nco >= (2<<22))) {
	  *mask = 0xFFFFFFFF;
          /* *mask >>= (n0+n1) doesn't work! */
	  *mask >>= n0;
	  *mask >>= n1;
	  return 1;
  }
  /* No wrapping of the gold code */
  *mask = 0;
  return 0;
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
  } else {
    fast_IF_nco_mask_8(nco, step, s, c);
    nco += step * 8;
    fast_IF_nco_mask_8(nco, step, s, c);
  }
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
  } else {
    fast_IF_nco_mask_16(nco, step, s, c);
    nco += step*16;
    fast_IF_nco_mask_16(nco, step, s, c);
  }
}

/************************************************
*
************************************************/
int main(int argc, char *argv[]) {
   FILE *f;
   uint_32 nco_if    = 0;
   uint_32 step_if   = 0x40104900;

   uint_32 nco_code  = (389)<<18, new_nco_code;
   uint_32 step_code = 0x00040000;

   int q = 0;
   int sine_count, cosine_count, sample_count;

   if(argc != 2) {
     printf("Please supply file name\n");
     return 0;
   }
   f = fopen(argv[1],"rb");
   if(f == NULL) {
     printf("Unable to open file\n");
     return 0;
   }

   /* Set the start values */
   sine_count = cosine_count = sample_count = 0;

   for(q = 0; q < 16368*1000/32*20; q++) {
     int end_of_repeat;
     uint_32 data;
     uint_32 fast_code, code_mask;
     uint_32 mixed_sine, mixed_cosine;
     uint_32 s_intermediate_freq, c_intermediate_freq;

     /* Read the data */
     data  = getc(f)<<24;
     data |= getc(f)<<16;
     data |= getc(f)<<8;
     data |= getc(f);
     if(data == EOF)
        break;

     /* Generate the intermediate frequency bitmaps     */ 
     fast_IF_nco_mask(nco_if, step_if, &s_intermediate_freq, &c_intermediate_freq);
     /* Generate the gold codes for this set of samples */
     end_of_repeat = fast_code_nco(gold_code, nco_code, step_code, &fast_code, &code_mask);

     /* Advance the NCOs */
     nco_if   += step_if   * 32;
     new_nco_code = nco_code + step_code * 32;
     if(new_nco_code >= 1023<<22 || new_nco_code < nco_code)
         new_nco_code -= 1023<<22;
     nco_code = new_nco_code;
    
     /* Mix the signals */
     mixed_sine   = data ^ fast_code ^ s_intermediate_freq; 
     mixed_cosine = data ^ fast_code ^ c_intermediate_freq; 

     /* Count how many bits are set */
     sine_count   += count_ones(mixed_sine);
     cosine_count += count_ones(mixed_cosine);
     sample_count += 32;

     if(end_of_repeat) {
         int next_sine, next_cosine, next_sample_count;

         next_sine         = count_ones(mixed_sine   & code_mask);
         next_cosine       = count_ones(mixed_cosine & code_mask);
         next_sample_count = count_ones(code_mask);
         sample_count -= next_sample_count;
         sine_count   -= next_sine   + sample_count/2;
         cosine_count -= next_cosine + sample_count/2;
         printf("%5i, %5i\n", sine_count, cosine_count);
         sine_count   = next_sine;
         cosine_count = next_cosine;
         sample_count = next_sample_count;
     }
   }
   return q;
}
