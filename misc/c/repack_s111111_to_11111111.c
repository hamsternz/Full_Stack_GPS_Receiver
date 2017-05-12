/*****************************************************
* Convert a data file that has a 2-bit sequence number
* in bits 7 and 6, and six 1-bit samples into a stream 
* of 8 1-bit samples, while verifing that the sequence
* numbers are correct 
*
* Should detect 3 out of 4 gaps in the data stream
*****************************************************/
#include <stdio.h>

int processBytes(FILE *fin, FILE *fout) {
  static int bytes=0;
  static unsigned char mask0, mask1, mask2, mask3;
  int i,n;
  unsigned char input[1024];
  unsigned char output[768];
  n = fread(input, 4, sizeof(input)/4, fin);
  if(n < 1) {
    printf("%i bytes processed\n",bytes);
    return 0;
  }
  if(bytes == 0) {
     mask0 = input[0] & 0xC0;
     mask1 = input[1] & 0xC0;
     mask2 = input[2] & 0xC0;
     mask3 = input[3] & 0xC0;
     if(mask1 != ((mask0 + 0x40) & 0xC0)) {
	printf("Bad mask1\n");
	return 0;              
     }
     if(mask2 != ((mask1 + 0x40) & 0xC0)) {
	printf("Bad mask2\n");
	return 0;              
     }
     if(mask3 != ((mask2 + 0x40) & 0xC0)) {
	printf("Bad mask3\n");
	return 0;              
     }
  }
  for(i = 0; i < n; i++) {
    if((input[i*4+0]&0xC0) != mask0) {
	printf("mismatch0 %i bytes processed\n",bytes);
	return 0;       
    }
    if((input[i*4+1]&0xC0) != mask1) {
	printf("mismatch1 %i bytes processed\n",bytes);
	return 0;       
    }
    if((input[i*4+2]&0xC0) != mask2) {
	printf("mismatch2 %i bytes processed\n",bytes);
	return 0;       
    }
    if((input[i*4+3]&0xC0) != mask3) {
	printf("mismatch3 %i bytes processed\n",bytes);
	return 0;       
    }
  }
   
  for(i = 0; i < n; i++) {
    output[i*3+0] = ((input[i*4+1] & 0x3f)<<6) | ((input[i*4+0] & 0x3f)>>0);
    output[i*3+1] = ((input[i*4+2] & 0x3f)<<4) | ((input[i*4+1] & 0x3f)>>2);
    output[i*3+2] = ((input[i*4+3] & 0x3f)<<2) | ((input[i*4+2] & 0x3f)>>4);
  }

  if(fwrite(output, 3, n, fout) != n) {
    fprintf(stderr,"Unable to write\n");
    return 0;
  }
  bytes += n*4;
  return 1;
}

int main(int argc, char *argv[])
{
  FILE *fin, *fout;
  if(argc != 3) {
    fprintf(stderr,"No file names supplied\n"); 
    return 0;
  }
  fin = fopen(argv[1],"rb");
  if(fin == NULL) {
    fprintf(stderr,"Unable to open input file\n"); 
    return 0;
  }

  fout = fopen(argv[2],"wb");
  if(fout == NULL) {
    fprintf(stderr,"Unable to open output file\n");
    fclose(fin);
    return 0;
  }

  while(processBytes(fin, fout))
    ;
  fclose(fin);
  fclose(fout);
  return 0;
}
