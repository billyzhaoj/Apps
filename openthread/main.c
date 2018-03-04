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
//#include "periph/dmac.h"
#include "sched.h"
#include "periph/adc.h"
#include "periph/i2c.h"
#include "periph/spi.h"

#ifndef SAMPLE_INTERVAL
#define SAMPLE_INTERVAL (30000000UL)
#endif
#define SAMPLE_JITTER   SAMPLE_INTERVAL/2
#define PAYLOAD_SIZE (75)

#define ENABLE_DEBUG (1)
#include "debug.h"

uint32_t interval_with_jitter(void)
{
    int32_t t = SAMPLE_INTERVAL/2;
    t += rand() % SAMPLE_INTERVAL;
    //t -= (SAMPLE_JITTER >> 1);
    return (uint32_t)t;
}

uint16_t myRloc = 0;

#ifdef CPU_DUTYCYCLE_MONITOR
volatile uint64_t cpuOnTime = 0;
volatile uint64_t cpuOffTime = 0;
volatile uint32_t contextSwitchCnt = 0;
volatile uint32_t preemptCnt = 0;
#endif
#ifdef RADIO_DUTYCYCLE_MONITOR
uint64_t radioOnTime = 0;
uint64_t radioOffTime = 0;
#endif

uint32_t packetReTxArray[11];
uint32_t packetReTxCnt = 0;
uint32_t packetSuccessCnt = 0;
uint32_t packetFailCnt = 0;
uint32_t packetBusyChannelCnt = 0;
uint32_t broadcastCnt = 0;
uint32_t queueOverflowCnt = 0;

uint16_t nextHopRloc = 0;
uint8_t borderRouterLC = 0;
uint8_t borderRouterRC = 0;
uint32_t routeChangeCnt = 0;
uint32_t borderRouteChangeCnt = 0;

uint32_t totalIpv6MsgCnt = 0;
uint32_t Ipv6TxSuccCnt = 0;
uint32_t Ipv6TxFailCnt = 0;
uint32_t Ipv6RxSuccCnt = 0;
uint32_t Ipv6RxFailCnt = 0;

uint32_t pollMsgCnt = 0;
uint32_t mleMsgCnt = 0;

uint32_t mleRouterMsgCnt = 0;
uint32_t addrMsgCnt = 0;
uint32_t netdataMsgCnt = 0;

uint32_t meshcopMsgCnt = 0;
uint32_t tmfMsgCnt = 0;

uint32_t totalSerialMsgCnt = 0;

static uint8_t source = OPENTHREAD_SOURCE;
static uint32_t AppPacketLostCnt = 0;

static char buf[PAYLOAD_SIZE];
static otUdpSocket mSocket;
static otMessageInfo messageInfo;
static otMessage *message = NULL;

static kernel_pid_t _main_pid;

/*void otUdpReceive(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo) {
    msg_t msg;
    msg_send(&msg, _main_pid);
}*/

int main(void)
{
    _main_pid = thread_getpid();

    for(int i=0; i<11; i++) {
        packetReTxArray[i] = 0;
    }

#if DMAC_ENABLE
    dmac_init();
    adc_set_dma_channel(DMAC_CHANNEL_ADC);
    i2c_set_dma_channel(I2C_0,DMAC_CHANNEL_I2C);
    spi_set_dma_channel(0,DMAC_CHANNEL_SPI_TX,DMAC_CHANNEL_SPI_RX);
#endif 
    DEBUG("This a test for OpenThread\n");    
    xtimer_usleep(200000000ul);

    DEBUG("\n[Main] Start UDP\n");
	otError error;

    DEBUG("[Main] Msg Creation\n");    
	memset(&messageInfo, 0, sizeof(messageInfo));
    // router 105
	//otIp6AddressFromString("fdde:ad00:beef:0000:1b8c:02db:410f:5405", &messageInfo.mPeerAddr);
	//otIp6AddressFromString("fdde:ad00:beef:0000:ba92:be52:ce6f:7670", &messageInfo.mPeerAddr);
    // Testbed node 10
	otIp6AddressFromString("fdde:ad00:beef:0000:8a1e:671f:0402:fdbe", &messageInfo.mPeerAddr);
    // Testbed node 17
    //otIp6AddressFromString("fdde:ad00:beef:0000:c684:4ab6:ac8f:9fe5", &messageInfo.mPeerAddr);
	messageInfo.mPeerPort = 1234;
    messageInfo.mInterfaceId = 1;

    for (int i = 0; i < PAYLOAD_SIZE; i++) {
        buf[i] = 0x0;
    }

    while (1) {
		//Sample
	    //sample(&frontbuf);

		//Sleep
        xtimer_usleep(interval_with_jitter());        

        /* Identity */
        buf[0] = source;
        buf[2] = myRloc & 0xff;
        buf[1] = (myRloc >> 8) & 0xff;

        /* Sequence Number */
        buf[4]++;
        if (buf[4] == 0) {
            buf[3]++;
        }

#ifdef CPU_DUTYCYCLE_MONITOR
        /* context switch */
        buf[8] = contextSwitchCnt & 0xff;
        buf[7] = (contextSwitchCnt >> 8) & 0xff; 
        buf[6] = (contextSwitchCnt >> 16) & 0xff; 
        buf[5] = (contextSwitchCnt >> 24) & 0xff;

        /* context switch */
        buf[12] = preemptCnt & 0xff;
        buf[11] = (preemptCnt >> 8) & 0xff; 
        buf[10] = (preemptCnt >> 16) & 0xff; 
        buf[9] = (preemptCnt >> 24) & 0xff;

        uint16_t cpuDutycycle = (uint16_t) (10000 * cpuOnTime / (cpuOnTime + cpuOffTime));
        buf[14] = cpuDutycycle & 0xff;
        buf[13] = (cpuDutycycle >> 8) & 0xff;
#endif
#ifdef RADIO_DUTYCYCLE_MONITOR
        /* Radio duty-cycle */
        uint16_t radioDutycycle = (uint16_t) (10000 * radioOnTime / (radioOnTime + radioOffTime));
        buf[16] = radioDutycycle & 0xff;
        buf[15] = (radioDutycycle >> 8) & 0xff; 
#endif
        /* Link Performance */
        buf[20] = packetReTxCnt & 0xff;
        buf[19] = (packetReTxCnt >> 8) & 0xff; 
        buf[18] = (packetReTxCnt >> 16) & 0xff; 
        buf[17] = (packetReTxCnt >> 24) & 0xff;

        buf[24] = packetSuccessCnt & 0xff;
        buf[23] = (packetSuccessCnt >> 8) & 0xff; 
        buf[22] = (packetSuccessCnt >> 16) & 0xff; 
        buf[21] = (packetSuccessCnt >> 24) & 0xff;

        buf[26] = packetBusyChannelCnt & 0xff;
        buf[25] = (packetBusyChannelCnt >> 8) & 0xff; 
        //buf[22] = (packetBusyChannelCnt >> 16) & 0xff; 
        //buf[21] = (packetBusyChannelCnt >> 24) & 0xff;

        buf[28] = packetFailCnt & 0xff;
        buf[27] = (packetFailCnt >> 8) & 0xff; 
        //buf[26] = (packetFailCnt >> 16) & 0xff; 
        //buf[25] = (packetFailCnt >> 24) & 0xff;

        buf[32] = broadcastCnt & 0xff;
        buf[31] = (broadcastCnt >> 8) & 0xff; 
        buf[30] = (broadcastCnt >> 16) & 0xff; 
        buf[29] = (broadcastCnt >> 24) & 0xff;

        /* Queue Overflow */
        buf[34] = queueOverflowCnt & 0xff;
        buf[33] = (queueOverflowCnt >> 8) & 0xff; 
        //buf[34] = (queueOverflowCnt >> 16) & 0xff; 
        //buf[33] = (queueOverflowCnt >> 24) & 0xff;

        /* Ipv6 Overhead */
        buf[38] = Ipv6TxSuccCnt & 0xff;
        buf[37] = (Ipv6TxSuccCnt >> 8) & 0xff;
        buf[36] = (Ipv6TxSuccCnt >> 16) & 0xff;
        buf[35] = (Ipv6TxSuccCnt >> 24) & 0xff;

        buf[40] = Ipv6TxFailCnt & 0xff;
        buf[39] = (Ipv6TxFailCnt >> 8) & 0xff;
        //buf[42] = (Ipv6TxFailCnt >> 16) & 0xff;
        //buf[41] = (Ipv6TxFailCnt >> 24) & 0xff;

        buf[44] = Ipv6RxSuccCnt & 0xff;
        buf[43] = (Ipv6RxSuccCnt >> 8) & 0xff;
        buf[42] = (Ipv6RxSuccCnt >> 16) & 0xff;
        buf[41] = (Ipv6RxSuccCnt >> 24) & 0xff;

        buf[46] = Ipv6RxFailCnt & 0xff;
        buf[45] = (Ipv6RxFailCnt >> 8) & 0xff;
        //buf[50] = (Ipv6RxFailCnt >> 16) & 0xff;
        //buf[49] = (Ipv6RxFailCnt >> 24) & 0xff;

        /* Route toward the BR */
        buf[48] = nextHopRloc & 0xff;
        buf[47] = (nextHopRloc >> 8) & 0xff;
        
        buf[49] = borderRouterLC;
        buf[50] = borderRouterRC;

        buf[52] = borderRouteChangeCnt & 0xff;
        buf[51] = (borderRouteChangeCnt >> 8) & 0xff; 
        //buf[58] = (borderRouteChangeCnt >> 16) & 0xff;
        //buf[57] = (borderRouteChangeCnt >> 24) & 0xff;

        buf[54] = routeChangeCnt & 0xff;
        buf[53] = (routeChangeCnt >> 8) & 0xff; 
        //buf[62] = (routeChangeCnt >> 16) & 0xff; 
        //buf[61] = (routeChangeCnt >> 24) & 0xff;
        
        /* Msg Overhead */
        buf[56] = pollMsgCnt & 0xff;
        buf[55] = (pollMsgCnt >> 8) & 0xff; 
        //buf[66] = (pollMsgCnt >> 16) & 0xff; 
        //buf[65] = (pollMsgCnt >> 24) & 0xff; 

        buf[58] = mleMsgCnt & 0xff;
        buf[57] = (mleMsgCnt >> 8) & 0xff; 
        //buf[70] = (mleMsgCnt >> 16) & 0xff; 
        //buf[69] = (mleMsgCnt >> 24) & 0xff;

        buf[62] = mleRouterMsgCnt & 0xff;
        buf[61] = (mleRouterMsgCnt >> 8) & 0xff; 
        buf[60] = (mleRouterMsgCnt >> 16) & 0xff; 
        buf[59] = (mleRouterMsgCnt >> 24) & 0xff;

        buf[64] = addrMsgCnt & 0xff;
        buf[63] = (addrMsgCnt >> 8) & 0xff; 
        //buf[78] = (addrMsgCnt >> 16) & 0xff; 
        //buf[77] = (addrMsgCnt >> 24) & 0xff;

        buf[66] = netdataMsgCnt & 0xff;
        buf[65] = (netdataMsgCnt >> 8) & 0xff; 
        //buf[82] = (netdataMsgCnt >> 16) & 0xff; 
        //buf[81] = (netdataMsgCnt >> 24) & 0xff;

        buf[70] = meshcopMsgCnt & 0xff;
        buf[69] = (meshcopMsgCnt >> 8) & 0xff; 
        buf[68] = (meshcopMsgCnt >> 16) & 0xff; 
        buf[67] = (meshcopMsgCnt >> 24) & 0xff;

        /*buf[76] = tmfMsgCnt & 0xff;
        buf[75] = (tmfMsgCnt >> 8) & 0xff; 
        buf[74] = (tmfMsgCnt >> 16) & 0xff; 
        buf[73] = (tmfMsgCnt >> 24) & 0xff;*/

        buf[74] = totalSerialMsgCnt & 0xff;
        buf[73] = (totalSerialMsgCnt >> 8) & 0xff;
        buf[72] = (totalSerialMsgCnt >> 16) & 0xff;
        buf[71] = (totalSerialMsgCnt >> 24) & 0xff;

		// Generate a packet
        mutex_lock(openthread_get_buffer_mutex());
        message = otUdpNewMessage(openthread_get_instance(), true);
        if (message == NULL) {
            DEBUG("error in new message");
        } else {
            error = otMessageSetLength(message, PAYLOAD_SIZE);
            if (error != OT_ERROR_NONE) {
                DEBUG("error in set length\n");
            } else {
                otMessageWrite(message, 0, buf, PAYLOAD_SIZE);
            }
        }
        mutex_unlock(openthread_get_buffer_mutex());

        // Send a packet
        if (message != NULL) {
            DEBUG("\n[Main] Tx UDP packet %u\n", buf[3]*256+buf[4]);
            error = otUdpSend(&mSocket, message, &messageInfo);
            if (error != OT_ERROR_NONE) {
                DEBUG("error in udp send\n");
            }
        } else {
            AppPacketLostCnt++;
            DEBUG("\n[Main] Lose UDP packet %u, %lu\n", buf[3]*256+buf[4], AppPacketLostCnt);
        }
    }
    return 0;
}
