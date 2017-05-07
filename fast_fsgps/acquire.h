void acquire_startup(void);
int acquire_start(int sv_id,
                  void (*callback_power)(int sv, uint_32 freq, uint_32 offset, uint_32 power), 
                  void (*callback_finished)(int sv, uint_32 power));
void acquire_update(uint_32 s);
int acquire_stop(int sv_id);
