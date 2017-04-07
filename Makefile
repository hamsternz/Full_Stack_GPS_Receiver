fsgps : fsgps.c
	gcc -o fsgps fsgps.c -Wall -pedantic -O2 -lm -Wno-long-long --std=gnu99
