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
