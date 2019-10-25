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
//#include "periph/dmac.h"
#include "sched.h"
#include "xtimer.h"
#include "periph/adc.h"
#include "periph/i2c.h"
#include "periph/spi.h"
#include "at86rf2xx.h"
#include "at86rf2xx_params.h"

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

static at86rf2xx_t at86rf2xx_dev;
static void _event_cb(netdev_t *dev, netdev_event_t event) {
    switch (event) {
        case NETDEV_EVENT_ISR:
            {
                /* radio_rx_msg.type = OPENTHREAD_NETDEV_MSG_TYPE_EVENT; */
                /* radio_rx_msg.content.ptr = dev; */
                /* radio_rx_msg.content.value = 1; */
/* #ifdef MODULE_OPENTHREAD_FTD */
                /* unsigned irq_state = irq_disable(); */
                /* ((at86rf2xx_t *)dev)->pending_irq++; */
                /* irq_restore(irq_state); */
/* #endif */
                /* if (msg_send(&radio_rx_msg, openthread_get_main_pid()) <= 0) { */
                /*     printf("ot_main: possibly lost radio interrupt.\n"); */
/* #ifdef MODULE_OPENTHREAD_FTD */
                /*     unsigned irq_state = irq_disable(); */
                /*     ((at86rf2xx_t *)dev)->pending_irq--; */
                /*     irq_restore(irq_state); */
/* #endif */
                /* } */
                break;
            }
        case NETDEV_EVENT_ISR2:
            {
                /* radio_tx_msg.type = OPENTHREAD_NETDEV_MSG_TYPE_EVENT; */
                /* radio_tx_msg.content.ptr = dev; */
                /* radio_tx_msg.content.value = 0; */
                /* if (msg_send(&radio_tx_msg, openthread_get_task_pid()) <= 0) { */
                /*     printf("ot_task: possibly lost radio interrupt.\n"); */
                /* } */
                break;
            }
        case NETDEV_EVENT_RX_COMPLETE:
            //recv_pkt(openthread_get_instance(), dev);
            break;
        case NETDEV_EVENT_TX_COMPLETE:
        case NETDEV_EVENT_TX_COMPLETE_DATA_PENDING:
        case NETDEV_EVENT_TX_NOACK:
        case NETDEV_EVENT_TX_MEDIUM_BUSY:
            //sent_pkt(openthread_get_instance(), event);
            break;
        default:
            break;
    }
}
int main(void)
{
#if DMAC_ENABLE
    dmac_init();
    adc_set_dma_channel(DMAC_CHANNEL_ADC);
    i2c_set_dma_channel(I2C_0,DMAC_CHANNEL_I2C);
    spi_set_dma_channel(0,DMAC_CHANNEL_SPI_TX,DMAC_CHANNEL_SPI_RX);
#endif 
    xtimer_usleep(300000ul);
    xtimer_ticks32_t last_wakeup = xtimer_now(); 


    at86rf2xx_setup(&at86rf2xx_dev, &at86rf2xx_params[0]);
    netdev_t *netdev = (netdev_t *) &at86rf2xx_dev; 
    netdev->driver->init(netdev);
    netdev->event_callback = _event_cb;
    netopt_enable_t enable = NETOPT_ENABLE;
    netdev->driver->set(netdev, NETOPT_RX_END_IRQ, &enable, sizeof(enable));
    netdev->driver->set(netdev, NETOPT_TX_END_IRQ, &enable, sizeof(enable));
    uint8_t chan = CHANNEL_OFFSET; 
    uint16_t panid = 0xbeef;   
    netopt_t state;
    netdev->driver->set(netdev,NETOPT_NID, &panid,sizeof(panid));
    while (1) {
        xtimer_periodic_wakeup(&last_wakeup, INTERVAL);
        //int res = netdev->driver->set(netdev,NETOPT_CHANNEL, &chan,sizeof(chan));
        
        //DEBUG("channel set ret code: %d \n", res);
        state = NETOPT_STATE_IDLE;
        netdev->driver->set(netdev, NETOPT_STATE, &state,sizeof(state));
        netdev->driver->get(netdev, NETOPT_STATE, &state,sizeof(state));
        DEBUG("NETOPT STATE: %d\n", state);
        
        int res = netdev->driver->get(netdev,NETOPT_ED,NULL,0);
        
        DEBUG("CHANNEL: %d ED READING: %d\n", chan, res);
        chan = (chan - 10) % CHANNEL_MOD + CHANNEL_OFFSET;
    }
    return 0;
}
