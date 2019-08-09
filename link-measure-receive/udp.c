/*
 * Copyright (C) 2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 *
 * @author      Martine Lenders <m.lenders@fu-berlin.de>
 * @}
 */

#include <stdio.h>
#include <inttypes.h>

#include "msg.h"
#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/pktdump.h"
#include "timex.h"
#include "xtimer.h"

#define SERVER_MSG_QUEUE_SIZE   (8U)
#define SERVER_PRIO             (THREAD_PRIORITY_MAIN - 1)
#define SERVER_STACKSIZE        (THREAD_STACKSIZE_MAIN)
#define SERVER_RESET            (0x8fae)

static gnrc_netreg_entry_t server = GNRC_NETREG_ENTRY_INIT_PID(0, KERNEL_PID_UNDEF);

static char server_stack[SERVER_STACKSIZE];
static msg_t server_queue[SERVER_MSG_QUEUE_SIZE];
static kernel_pid_t server_pid = KERNEL_PID_UNDEF;

static uint32_t rcv_count = 0;

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

static void _receive(gnrc_pktsnip_t *pkt)
{
    gnrc_pktsnip_t *app;
    ham7c_t* app_payload;

    app = gnrc_pktbuf_start_write(pkt);
    app_payload = (ham7c_t *)app->data;

    /* packet count for link measurement */
    rcv_count++;
    printf("Rx %lu Tx %lu Sender %u Receiver %u\n", rcv_count, app_payload->seqNum, app_payload->sender, HOSTNAME);

    gnrc_pktbuf_release(pkt);
}

static void *_eventloop(void *arg)
{
    (void)arg;
    msg_t msg, reply;

    /* setup the message queue */
    msg_init_queue(server_queue, SERVER_MSG_QUEUE_SIZE);

    reply.content.value = (uint32_t)(-ENOTSUP);
    reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
    printf("Start UDP\n");
    while (1) {
        msg_receive(&msg);

        switch (msg.type) {
            case GNRC_NETAPI_MSG_TYPE_RCV:
                _receive(msg.content.ptr);
                break;
            case GNRC_NETAPI_MSG_TYPE_GET:
            case GNRC_NETAPI_MSG_TYPE_SET:
                msg_reply(&msg, &reply);
                break;
            case SERVER_RESET:
                rcv_count = 0;
                break;
            default:
                break;
        }
    }

    /* never reached */
    return NULL;
}


void start_server(uint16_t port)
{
    /* check if server is already running */
    if (server.target.pid != KERNEL_PID_UNDEF) {
        printf("Error: server already running on port %" PRIu32 "\n",
               server.demux_ctx);
        return;
    }
    /* parse port */
    if (port == 0) {
        puts("Error: invalid port specified");
        return;
    }
    if (server_pid <= KERNEL_PID_UNDEF) {
        server_pid = thread_create(server_stack, sizeof(server_stack), SERVER_PRIO,
                                   THREAD_CREATE_STACKTEST, _eventloop, NULL, "UDP server");
        if (server_pid <= KERNEL_PID_UNDEF) {
            puts("Error: can not start server thread");
            return;
        }
    }
    /* start server (which means registering pktdump for the chosen port) */
    gnrc_netreg_entry_init_pid(&server, port, server_pid);
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);
    printf("Success: started UDP server on port %" PRIu16 "\n", port);
}
