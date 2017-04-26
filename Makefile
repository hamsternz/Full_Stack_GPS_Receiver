fsgps : fsgps.c
	gcc -o fsgps fsgps.c -Wall -pedantic -O4 -lm -Wno-long-long --std=gnu99 -pg
