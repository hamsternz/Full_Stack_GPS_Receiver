struct Channel {
   uint_32 nco_if;
   uint_32 step_if;

   uint_32 nco_code;
   uint_32 step_code;
   uint_32 code_tune;

   int_32 early_sine_count,  early_cosine_count,  early_sample_count;
   int_32 prompt_sine_count, prompt_cosine_count, prompt_sample_count;
   int_32 late_sine_count,   late_cosine_count,   late_sample_count;

   uint_32 early_power_filtered; 
   uint_32 prompt_power_filtered; 
   uint_32 late_power_filtered; 

   uint_32 early_power_filtered_not_reset; 
   uint_32 prompt_power_filtered_not_reset; 
   uint_32 late_power_filtered_not_reset; 

   int prompt_sc_temp;
   int prompt_cc_temp;

   uint_8 last_angle;
   int_32 delta_filtered;
   int_32 angle_filtered;
   uint_8 ms_of_bit;
   uint_8 last_bit;
   uint_8 no_adjust;
   uint_8 channel_allocated;
   uint_8 sv_id;
};
void channel_startup(void (*phase_callback)(int,int));
int  channel_add(int_8 sv_id, uint_32 step_if, uint_32 nco_code, int_32 code_tune);
int channel_get_count(void);
uint_32 channel_get_nco_limit(void);
uint_32 channel_get_sv_id(int handle);
uint_32 channel_get_nco_phase(int handle);
int  channel_remove(int handle);
void channel_update(uint_32 data);

