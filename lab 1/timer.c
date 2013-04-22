#include "timer.h"

#include <time.h>
#include <sys/time.h>
#include <stdbool.h>

/* This implementation is taken from the matrixSum.c example
   of the homework. */
double read_timer() {
	static bool initialized = false;
	static struct timeval start;
	struct timeval end;

	if (!initialized ) {
		gettimeofday(&start, NULL);
		initialized = true;
	}

	gettimeofday(&end, NULL);
	return (end.tv_sec - start.tv_sec) + 1.0e-6 * (end.tv_usec - start.tv_usec);
}
