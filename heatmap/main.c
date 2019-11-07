/*
 * Copyright (C) 2017 Baptiste CLENET
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @file
 * @brief       OpenThread test application
 * 
 * @author      Hyung-Sin Kim <hs.kim@cs.berkeley.edu>
 */

#include <stdio.h>
#include <rtt_stdio.h>
#include "ot.h"
#include <openthread/udp.h>
#include <openthread/cli.h>
#include <openthread/openthread.h>
#include "main.h"
//#include "periph/dmac.h"
#include "sched.h"
#include "periph/adc.h"
#include "periph/i2c.h"
#include "periph/spi.h"

#ifndef SAMPLE_INTERVAL
#define SAMPLE_INTERVAL (20000000UL)
#endif
#define SAMPLE_JITTER   SAMPLE_INTERVAL/2
#define PAYLOAD_SIZE (75)

#define ENABLE_DEBUG (1)

#define INTERVAL (100U * US_PER_MS)
#define CHANNEL_OFFSET 11
#define CHANNEL_MOD    16
#include "debug.h"
int main(void)
{
#if DMAC_ENABLE
    dmac_init();
    adc_set_dma_channel(DMAC_CHANNEL_ADC);
    i2c_set_dma_channel(I2C_0,DMAC_CHANNEL_I2C);
    spi_set_dma_channel(0,DMAC_CHANNEL_SPI_TX,DMAC_CHANNEL_SPI_RX);
#endif 
    DEBUG("This a test for OpenThread\n");
    xtimer_usleep(3000ul);
    xtimer_ticks32_t last_wakeup = xtimer_now(); 
    netdev_t* netdev =  openthread_get_netdev();
    uint8_t chan = CHANNEL_OFFSET; 
    netopt_t state;
    while (1) {
        xtimer_periodic_wakeup(&last_wakeup, INTERVAL);
        otError err = otLinkSetChannel(openthread_get_instance(), chan);
        DEBUG("otError: %d \n", err);
        state = NETOPT_STATE_IDLE;
        netdev->driver->set(netdev, NETOPT_STATE, &state,sizeof(state));
        netdev->driver->get(netdev, NETOPT_STATE, &state,sizeof(state));
        DEBUG("NETOPT STATE: %d\n", state);

        int res = netdev->driver->get(netdev,NETOPT_ED,NULL,0);
        DEBUG("CHANNEL: %d ED READING: %d\n", otLinkGetChannel(openthread_get_instance()), res);
        chan = (chan - 10) % CHANNEL_MOD + CHANNEL_OFFSET;
    }
    return 0;
}
