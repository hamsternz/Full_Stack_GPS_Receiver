void channel_startup(int (*phase_callback)(int,int));
int  channel_add(int_8 sv_id, uint_32 step_if, uint_32 nco_code, int_32 code_tune);
int channel_get_count(void);
int channel_get_power_by_sv_id(int sv_id, uint_32 *power);
int channel_get_power(int handle, uint_32 *early_power, uint_32 *prompt_power, uint_32 *late_power);
uint_32 channel_get_nco_limit(void);
int channel_disable_track(int handle);
int channel_enable_track(int handle);
uint_32 channel_get_sv_id(int handle);
uint_32 channel_tracking(int sv_id);
uint_32 channel_get_nco_phase(int handle);
int  channel_remove(int handle);
void channel_update(uint_32 data);

