/****************************************************************************
* nav.h - User interface for the NAV data processing
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
int nav_startup(char *datafile_name_buf);
int nav_bit_sync(int sv);
int nav_week_num(int sv);
int nav_subframe_of_week(int sv);
int nav_ms_of_frame(int sv);
void nav_remove(int sv);
int nav_known_frames(int sv);
int nav_get_bit_errors_count(int sv);
int nav_clear_bit_errors_count(int sv);
int nav_add_bit(int sv, int power);
int nav_calc_corrected_time(int sv_id, double raw_t, double *corrected_t);
int nav_calc_position(int sv_id, double t, double *x, double *y, double *z);
