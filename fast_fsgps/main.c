/****************************************************************************
* main.c - Main program for my software GPS receiver
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
#include "channel.h"
#include "nav.h"
#include "solve.h"
#include "acquire.h"
#include "status.h"
#include "schedule.h"

/************************************************
*
************************************************/
int main(int argc, char *argv[]) {
   static unsigned swap_bits[256];
   int q;
   FILE *f;

   for(q = 0; q < 256; q++) {
     swap_bits[q] = 0;
     if(q&0x01) swap_bits[q] |= 0x80;
     if(q&0x02) swap_bits[q] |= 0x40;
     if(q&0x04) swap_bits[q] |= 0x20;
     if(q&0x08) swap_bits[q] |= 0x10;
     if(q&0x10) swap_bits[q] |= 0x08;
     if(q&0x20) swap_bits[q] |= 0x04;
     if(q&0x40) swap_bits[q] |= 0x02;
     if(q&0x80) swap_bits[q] |= 0x01;
   }

   if(argc != 2) {
     printf("Please supply file name\n");
     return 0;
   }
   f = fopen(argv[1],"rb");
   if(f == NULL) {
     printf("Unable to open file\n");
     return 0;
   }
   gold_code_startup();
   nav_startup();
   acquire_startup();
   channel_startup(nav_add_bit);
   schedule_startup();

   while(1) {
     uint_32 data;
     int ch;
     static int processed = 0;
     if(processed % ((16368000/32)/100) == 0) {
       show_status(processed*32.0/16368000);
     }
     processed++;

     /* Read the data */
     ch  = getc(f);
     if(ch == EOF) break;
     data  = swap_bits[ch]<<24;
     ch  = getc(f);
     if(ch == EOF) break;
     data |= swap_bits[ch]<<16;
     ch = getc(f);
     if(ch == EOF) break;
     data |= swap_bits[ch]<<8;
     ch  = getc(f);
     if(ch == EOF) break;
     data |= swap_bits[ch]<<0;
     acquire_update(data);  /* This has to go first ! */
     channel_update(data);
   }
   schedule_shutdown();

   return 0;
}
