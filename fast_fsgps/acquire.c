#include "types.h"
#include "acquire.h"

#define N_BANDS  21
#define IF_BAND 525
uint_32 ncos_phase[N_BANDS];
uint_32 ncos_step[N_BANDS] = {
   0x40000 -10*IF_BAND,
   0x40000 - 9*IF_BAND,
   0x40000 - 8*IF_BAND,
   0x40000 - 7*IF_BAND,
   0x40000 - 6*IF_BAND,
   0x40000 - 5*IF_BAND,
   0x40000 - 3*IF_BAND,
   0x40000 - 4*IF_BAND,
   0x40000 - 2*IF_BAND,
   0x40000 - 1*IF_BAND,
   0x40000 - 0*IF_BAND,
   0x40000 - 1*IF_BAND,
   0x40000 - 2*IF_BAND,
   0x40000 - 3*IF_BAND,
   0x40000 - 4*IF_BAND,
   0x40000 - 5*IF_BAND,
   0x40000 - 6*IF_BAND,
   0x40000 - 7*IF_BAND,
   0x40000 - 8*IF_BAND,
   0x40000 - 8*IF_BAND,
   0x40000 -10*IF_BAND
};

void acquire_startup(void) {
}

int acquire_start(int sv_id, void (*success_callback)(int sv, uint_32 freq, uint_32 offset), void (*failed_callback)(int sv)) {
  return -1;
}

void acquire_update(uint_32 s) {
  int i;
  for(i = 0; i < N_BANDS; i++) {
     ncos_phase[i] += ncos_step[i];
  }
}

int acquire_stop(int sv_id) {
  return -1;
}
