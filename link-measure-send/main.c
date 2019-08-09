#include <stdio.h>
#include <rtt_stdio.h>
#include "xtimer.h"
#include <string.h>
#include "net/gnrc/udp.h"
#include "phydat.h"
#include "saul_reg.h"
#include "periph/adc.h"
#include "periph/i2c.h"
#include "periph/spi.h"
//#include "periph/dmac.h"
#include "periph/timer.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#ifndef SAMPLE_INTERVAL
#define SAMPLE_INTERVAL (50000UL)
#endif

#define TYPE_FIELD 8


void send_udp(char *addr_str, uint16_t port, uint8_t *data, uint16_t datalen);

#define AES_SKIP_START_BYTES 4

typedef struct __attribute__((packed,aligned(4))) {
  uint32_t seqNum;
  uint16_t sender;
  uint16_t serial;
  uint16_t flags; //which of the fields below exist
  int16_t  acc_x;
  int16_t  acc_y;
  int16_t  acc_z;
  int16_t  mag_x;
  int16_t  mag_y;
  int16_t  mag_z;
  int16_t  radtemp;
  int16_t  temp;
  int16_t  hum;
  int16_t  light_lux;
  uint16_t buttons;
  uint16_t occup;
  uint32_t reserved1;
  uint32_t reserved2;
  uint32_t reserved3;
} ham7c_t;

//All of the flags
#define PROVIDED_FLAGS (0x7F)

void critical_error(void) {
    DEBUG("CRITICAL ERROR, REBOOT\n");
    NVIC_SystemReset();
    return;
}

ham7c_t frontbuf;
uint8_t obuffer [sizeof(ham7c_t)];

int main(void) {
    frontbuf.sender = SENDER_ID;
    frontbuf.seqNum = 0;

    printf("START as a sender %u with payload size %u\n", frontbuf.sender, sizeof(ham7c_t));

    while (1) {
        if (frontbuf.seqNum < 0xFFFFFFFF) {
            frontbuf.seqNum++;
            printf("%lu-th packet\n", frontbuf.seqNum);
            memcpy(obuffer, ((uint8_t*)&frontbuf), sizeof(ham7c_t));
	    //Send
	    send_udp("ff02::1",4747,obuffer,sizeof(obuffer));
	}
	//Sleep
        xtimer_usleep(SAMPLE_INTERVAL);
    }

    return 0;
}
