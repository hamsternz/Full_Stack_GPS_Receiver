void acquire_startup(void);
int acquire_start(int sv_id, void (*success_callback)(int sv, uint_32 freq, uint_32 offset), void (*failed_callback)(int sv));
void acquire_update(uint_32 s);
int acquire_stop(int sv_id);
