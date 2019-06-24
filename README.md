# Full_Stack_GPS_Receiver

A Software GPS decoder, going from raw 1-bit ADC samples to 
position fix.  This isn't designed to be super-fast, it is 
designed to let you experiment and improve the methods used, 
and test out new ideas.


It can also serve as a known-to-work implementation for you 
to compare your own implmentation against.  For me, it was
designed for testing algorthms before moving to an FPGA 
implementation.

If you want to test it out:
- Build the fsgps program 
- Download the data file from http://www.jks.com/gps/gps.html
- Process it with the following command:

    fsgps -s 5456000 -i 4092000 gps.samples.1bit.I.fs5456.if4092.bin

After a while you should start seeing location fixes

This is the last fix printed for that datafile:

    SV, STATE,Orbit,Time,  Week,       Frame,  millisec,       chip,   subchip, NAV status info
    -------------------------------------------------------------------------------------------
    01, LOCKED, YES,YES,   1609,       77797,      4688,        269,         50 7  24  8 12345
    02,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    03,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    04,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    05, LOCKED,  NO, NO,      0,       53294,      4676,        921,         39 2   0  2 ....5
    06,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    07,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    08,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    09,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    10,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    11,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    12,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    13,ACQUIRE,  NO, NO,      0,           5,      5404,          0,          8 0  -1  0 .....
    14,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    15,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    16, LOCKED,  NO,YES,   1609,       77797,      4683,        436,         18 2  29  3 12.4.
    17,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    18,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    19,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    20,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    21, LOCKED, YES,YES,   1609,       77797,      4683,         47,         27 7  24  3 12345
    22,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    23,ACQUIRE,  NO, NO,      0,           0,         0,          0,         11 0  -2  0 .....
    24,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    25, LOCKED, YES,YES,   1609,       77797,      4682,        242,          3 7  24  2 12345
    26,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    27,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    28,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....
    29, LOCKED, YES,YES,   1609,       77797,      4688,        946,          7 7  24  8 12345
    30, LOCKED, YES,YES,   1609,       77797,      4690,        741,         46 7  24 10 12345
    31, LOCKED, YES,YES,   1609,       77797,      4687,        430,         62 7  24  7 12345
    32,ACQUIRE,  NO, NO,      0,           0,         0,          0,          0 0  -2  0 .....

    Location is (  10796441.72686,  -11059550.95575,   21468769.87626) @ 466786.68844515
    Location is (  25624920.03297,    5493913.05874,    6376813.14970) @ 466786.68314817
    Location is (  17562608.84385,   17600783.06751,    9306656.31975) @ 466786.68228779
    Location is (  12436678.35329,    8851464.38357,   21813028.15234) @ 466786.68876178
    Location is (  18466438.20510,    6615544.19174,   17512994.82941) @ 466786.69042944
    Location is (  20893698.98803,   -8968194.45015,   14090804.00502) @ 466786.68741330
  
    Solved is  (      3851785.003462,        -78309.097013,       5066312.772752) @             0.003504 (alt       6364739.204313)
    Lat/Lon/Alt :            52.934773,            -1.164697,                176.8

Signals from eight space vehicles have been obtained, but three have had marginal reception.

    SV 01 has a lock, and all NAV data retrieved 
    SV 05 has a lock, and only a minimal amount of data has been received.
    SV 13 was attempted to be locked, but the lock was dropped as good BPSK data was not being received.
    SV 16 has a lock with some data, so must have lost sync
    SV 25 has a lock, and all NAV data retrieved 
    SV 29 has a lock, and all NAV data retrieved 
    SV 30 has a lock, and all NAV data retrieved 
    SV 31 has a lock, and all NAV data retrieved 


The calcuated locations of the space vehicles are printed, as well as the ECEF solution for position:
    3851785.003462,        -78309.097013,       5066312.772752

With the last Lat/Lon/Alt fix of:

   Lat/Lon/Alt :            52.934773,            -1.164697,                176.8

