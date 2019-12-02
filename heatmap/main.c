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
#include "main.h"
#include "sched.h"
#include "periph/adc.h"
#include "periph/i2c.h"
#include "periph/spi.h"
#include "at86rf2xx.h"
#include "at86rf2xx_params.h"
#include "xtimer.h"
#ifndef SAMPLE_INTERVAL
#define SAMPLE_INTERVAL (20000UL)
#endif
#define SAMPLE_JITTER   SAMPLE_INTERVAL/2
#define PAYLOAD_SIZE (75)

#define ENABLE_DEBUG (1)

#define INTERVAL (500U * US_PER_MS)
#define CHANNEL_OFFSET 11
#define CHANNEL_MOD    16
#include "debug.h"

static void _event_cb(netdev_t *dev, netdev_event_t event) {
}

static at86rf2xx_t at86rf2xx_dev;
int main(void)
{
#if DMAC_ENABLE
    dmac_init();
    adc_set_dma_channel(DMAC_CHANNEL_ADC);
    i2c_set_dma_channel(I2C_0,DMAC_CHANNEL_I2C);
    spi_set_dma_channel(0,DMAC_CHANNEL_SPI_TX,DMAC_CHANNEL_SPI_RX);
#endif 
    xtimer_usleep(300000ul);

    at86rf2xx_setup(&at86rf2xx_dev,&at86rf2xx_params[0]);
    netdev_t *netdev = (netdev_t *) &at86rf2xx_dev;

    netdev->driver->init(netdev);
    netdev->event_callback = _event_cb;
    netopt_enable_t enable = NETOPT_ENABLE;
    netdev->driver->set(netdev, NETOPT_RX_END_IRQ, &enable, sizeof(enable));
    netdev->driver->set(netdev,NETOPT_TX_END_IRQ,&enable,sizeof(enable));
    int16_t txPower = 4;
    netdev->driver->set(netdev,NETOPT_TX_POWER, &txPower, sizeof(int16_t)); 
    xtimer_ticks32_t last_wakeup = xtimer_now(); 
    uint16_t chan = CHANNEL_OFFSET; 
    netopt_t state;
    while (1) {

        //xtimer_usleep(2000000ul);
        xtimer_periodic_wakeup(&last_wakeup, INTERVAL);
        state = NETOPT_STATE_IDLE; 
        netdev->driver->set(netdev, NETOPT_STATE, &state,sizeof(state));
        netdev->driver->get(netdev, NETOPT_STATE, &state,sizeof(state));
        netdev->driver->set(netdev,NETOPT_CHANNEL,&chan, sizeof(uint16_t));
        uint32_t now = xtimer_now_usec();
        int resChan =  netdev->driver->get(netdev,NETOPT_CHANNEL,NULL,0);
        int res = netdev->driver->get(netdev,NETOPT_ED,NULL,0);
        xtimer_usleep(20000ul);
        int phase = netdev->driver->get(netdev,NETOPT_PHASE,NULL,0);
        DEBUG("TIME: %lu CHANNEL: %d ED READING: %d PHASE: %d\n", now, resChan, res, phase);
        xtimer_usleep(20000ul);
        chan = (chan - 10) % CHANNEL_MOD + CHANNEL_OFFSET;
    }
    return 0;
}
