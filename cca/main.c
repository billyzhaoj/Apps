#include <stdio.h>
#include <rtt_stdio.h>
#include "xtimer.h"
#include <string.h>
#include "periph/timer.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#ifndef SAMPLE_INTERVAL
#define SAMPLE_INTERVAL (100000000UL)
#endif
#define SAMPLE_JITTER   ( 200000UL)

#define TYPE_FIELD 8

int main(void) {
    while (1) {
		//Sleep
		xtimer_usleep(SAMPLE_INTERVAL);
    }

    return 0;
}
