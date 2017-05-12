/******************************************************
* Use the Digilent EPP parallel interface to grab a 
* gigabyte of data from an FPGA board.
*
* Requires that the Digilent Adept SDK libraries are
* installed
******************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <windows.h>
#define INT32 int

#include "dpcdecl.h"
#include "depp.h"
#include "dmgr.h"

static HIF hif = hifInvalid;

#define BLOCKSIZE (1024*128)
static int blocks = 0;
BYTE    buffer[1024*1024*1024];
static int DoGetRegRepeat(FILE *f) {
    static BYTE l = 0xFF;
    if(blocks > sizeof(buffer)/BLOCKSIZE)
      return 0;

    if(!DeppGetRegRepeat(hif, 0, buffer+blocks*BLOCKSIZE, BLOCKSIZE, fFalse)){
      printf("DepGetRegRepeat failed.\n");
      return 0;
    }
    if((buffer[blocks*BLOCKSIZE] & 0xC0) != l) {
       l = buffer[blocks*BLOCKSIZE] & 0xC0;
       printf(" Keyed to %02x at %i\n",l,blocks*BLOCKSIZE);
    }
    blocks++;
    return 1;
}

int main(int cszArg, char * rgszArg[]) {
    FILE *f;
    int start, end;
    f = fopen("gps_data.bin","wb");
    if(f == NULL) {
       fprintf(stderr,"Unable to open file\n");
       return 0;
    }
    if(!DmgrOpen(&hif, "Nexys2")) {  // Change to Basys2 for the other board.
        printf("DmgrOpen failed (check the device name you provided)\n");
        return 0;
    }

    if(!DeppEnable(hif)) {
        printf("DeppEnable failed\n");
        return 0;
    }

    if( hif == hifInvalid )
       printf("Invalid\n");

    if(!DeppGetRegRepeat(hif, 0, buffer+blocks*BLOCKSIZE, BLOCKSIZE, fFalse)){
      printf("DepGetRegRepeat failed.\n");
      return 0;
    }
    if(!DeppGetRegRepeat(hif, 0, buffer+blocks*BLOCKSIZE, BLOCKSIZE, fFalse)){
      printf("DepGetRegRepeat failed.\n");
      return 0;
    }

    start = time(NULL);
    while(DoGetRegRepeat(f))
        ;
    end = time(NULL);

    printf("Stream from register completed %i bytes!\n",BLOCKSIZE*blocks);
    if( hif != hifInvalid ) {
        printf("CLeanup1\n");
        DeppDisable(hif);
        printf("CLeanup2\n");
        DmgrClose(hif);
    }
    fwrite(buffer,1,BLOCKSIZE*blocks,f);
    fclose(f);
    printf("%i seconds - %i samples per millisecond\n",end-start, BLOCKSIZE*blocks/1000*3/(end-start));
    return 0;
}
