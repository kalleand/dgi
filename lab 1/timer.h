#ifndef EFAHLEN_PPS_TIMER
#define EFAHLEN_PPS_TIMER 1

#ifdef __cplusplus
extern "C" {
#endif

/* This function reads the current time, in seconds.
   To measure time, read this value twice and subtract. */
double read_timer();

#ifdef __cplusplus
}
#endif
#endif
