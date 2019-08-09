#include <stdio.h>
#include <rtt_stdio.h>
#include "xtimer.h"
#include <string.h>
#include "periph/timer.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

extern void start_server(uint16_t port);

int main(void) {
    printf("START as a receiver\n");
    start_server(4747);    

    while (1) {
	xtimer_usleep(3000000UL);
    }

    return 0;
}
