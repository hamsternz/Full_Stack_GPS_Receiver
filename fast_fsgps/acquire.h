void acquire_setup(void);
int acquire_start(int sv_id, void (*success_callback)(int sv, uint_32 freq, uint_32 offset, void (*failed_callback)(int sv));
int acquire_stop(int sv_id);
