int nav_startup(void);
int nav_week_num(int sv);
int nav_subframe_of_week(int sv);
int nav_ms_of_frame(int sv);
void nav_add_bit(int sv, int power);
int nav_calc_corrected_time(int sv_id, double raw_t, double *corrected_t);
int nav_calc_position(int sv_id, double t, double *x, double *y, double *z);
