/****************************************************************************
* channel.h - Headers for channel.c
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
void channel_startup(int (*phase_callback)(int,int));
int  channel_add(int_8 sv_id, uint_32 step_if, uint_32 nco_code);
int channel_get_count(void);
int channel_get_power_by_sv_id(int sv_id, uint_32 *power);
int channel_get_power(int handle, uint_32 *early_power, uint_32 *prompt_power, uint_32 *late_power);
uint_32 channel_get_nco_limit(void);
int channel_disable_track(int handle);
int channel_enable_track(int handle);
uint_32 channel_get_sv_id(int handle);
uint_32 channel_get_code_tune(int handle);
uint_32 channel_tracking_by_sv_id(int sv_id);
uint_32 channel_get_nco_phase(int handle);
int  channel_remove(int handle);
void channel_update(uint_32 data);

