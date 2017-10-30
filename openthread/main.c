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
 * @author      Baptiste Clenet <bapclenet@gmail.com>
 */

#include <stdio.h>

#include "ot.h"
#include <openthread/udp.h>
#include <openthread/cli.h>
#include <openthread/openthread.h>
#include <openthread/platform/platform.h>
#include <openthread/platform/logging.h>
#include <openthread/config.h>
#include <xtimer.h>
#include <openthread/diag.h>
#ifndef SAMPLE_INTERVAL
#define SAMPLE_INTERVAL (1000000U)
#endif
    

void receiveCallback(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo) {
    printf("received message\n");
}

void receive(void) {
	otUdpSocket mSocket;
    otSockAddr sockaddr;
    otError error;

    otInstance *sInstance = otInstanceInitSingle();
    if (!otInstanceIsInitialized(sInstance)) {
        printf("error in init");
    }

    // mSocket.mTransport = &sInstance->mIp6.mUdp;


    memset(&sockaddr, 0, sizeof(sockaddr));
    error = otIp6AddressFromString("::", &sockaddr.mAddress);
    if (error != OT_ERROR_NONE) {
        printf("error in ip address");
    }
    
    sockaddr.mPort = 4747;
    
    // memset(&mSocket, 0, sizeof(mSocket));
    // mSocket.mTransport = 0;
    
    
    //otUdpClose(&mSocket);

    
    error = otUdpOpen(sInstance, &mSocket, receiveCallback, NULL);
    if (error != OT_ERROR_NONE) {
        printf("error in opening");
    }
    
    error = otUdpBind(&mSocket, &sockaddr);
    if (error != OT_ERROR_NONE) {
        printf("error in binding");
    }

    // while (1) {}
    // error = otUdpBind(&mSocket, &sockaddr);
    // if (error != OT_ERROR_NONE) {
    //     printf("error in binding");
    // }
    
    // void* result = NULL;
    // char* arg= "open";
    // uint8_t res = ot_call_command("udp", (void*)&arg, &result);
    // printf("%x", res);
}

void send(void) {
    // puts("This a test for OpenThread");
    // /* Example of how to call OpenThread stack functions */
    // puts("Get PANID ");
    // uint16_t panid = 0;
    // uint8_t res = ot_call_command("panid", NULL, (void*)&panid);
    // printf("Current panid: 0x%x (res:%x)\n", panid, res);
   
    // otNetifAddress* addr = NULL;
    // uint8_t res = ot_call_command("ipaddr", NULL, (void*)&addr);
    // printf("%d", res);    
 
	otUdpSocket mSocket;
	otError error;
    otInstance *sInstance = otInstanceInitSingle();
	if (sInstance == NULL) {
        printf("error in init");
    }

    // addr = NULL;
    // addr = otIp6GetUnicastAddresses(sInstance);
    //ot_ipaddr(sInstance, NULL, (void*) &addr);
    /*otCliUartInit(sInstance);
    while (1) {
        otTaskletsProcess(sInstance);
        //PlatformProcessDrivers(sInstance);
    }*/
    otSockAddr sockaddr;
	memset(&sockaddr, 0, sizeof(sockaddr));
    error = otIp6AddressFromString("fdde:ad00:beef:0:bb1:ebd6:ad10:f33", &sockaddr.mAddress);
    if (error != OT_ERROR_NONE) {
        printf("error in ip address");
    }

    error = otUdpBind(&mSocket, &sockaddr);
    if (error != OT_ERROR_NONE) {
        printf("error in binding");
    }

    sockaddr.mPort = 4747;
	error = otUdpConnect(&mSocket, &sockaddr);
    if (error != OT_ERROR_NONE) {
        printf("error in udp connect");
    }

    
    // "f846:c0bd:f820:0010:0c1c:07b5:1d1c:161c"
	otMessageInfo messageInfo;
	otMessage *message = NULL;
	memset(&messageInfo, 0, sizeof(messageInfo));
	//otIp6AddressFromString("fe08:0000:0000:0000:dcba:93c6:d43c:c83a", &messageInfo.mPeerAddr);

	otIp6AddressFromString("fe08:0000:0000:0000:dcba:93c6:d43c:c83a", &messageInfo.mPeerAddr);
    messageInfo.mPeerPort = 4747;

	message = otUdpNewMessage(sInstance, true);
    if (message == NULL) {
        printf("error in new message");
    }
    error = otMessageAppend(message, "hi", 3);
	if (error != OT_ERROR_NONE) {
        printf("error in messag eappend");
    }

    while (1) {
		printf("\n\nsending packet\n");
        error = otUdpSend(&mSocket, message, &messageInfo);
        if (error != OT_ERROR_NONE) {
            printf("error in udp send\n");
        }
        xtimer_usleep(3*SAMPLE_INTERVAL);
	}	
}

int main(void)
{
    // otNetifAddress* addr = NULL;
    // uint8_t res = ot_call_command("ipaddr", NULL, (void*)&addr);
    //printf("%d", res);
    send();
    //receive();
}
