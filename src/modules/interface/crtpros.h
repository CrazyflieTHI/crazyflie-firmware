/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * crtpros.h - CrazyRealtimeTransferProtocol stack
 */

#ifndef CRTPROS_H_
#define CRTPROS_H_

#include <stdint.h>
#include <stdbool.h>
#include "crtp.h"

#define CRTPROS_IS_NULL_PACKET(P) ((P.header&0xF3)==0xF3)

#define CRTP_PORT_SETPOINT_SIM 0x9

typedef void (*CrtprosCallback)(CRTPPacket *);

/**
 * Initialize the CRTPROS stack
 */
void crtprosInit(void);

bool crtprosTest(void);

/**
 * Initializes the queue and dispatch of an task.
 *
 * @param[in] taskId The id of the CRTPROS task
 */
void crtprosInitTaskQueue(CRTPPort taskId);

/**
 * Register a callback to be called for a particular port.
 *
 * @param[in] port Crtpros port for which the callback is set
 * @param[in] cb Callback that will be called when a packet is received on
 *            'port'.
 *
 * @note Only one callback can be registered per port! The last callback
 *       registered will be the one called
 */
void crtprosRegisterPortCB(int port, CrtprosCallback cb);

/**
 * Put a packet in the TX task
 *
 * If the TX stack is full, the oldest lowest priority packet is dropped
 *
 * @param[in] p CRTPPacket to send
 */
int crtprosSendPacket(CRTPPacket *p);

/**
 * Put a packet in the TX task
 *
 * If the TX stack is full, the function block until one place is free (Good for console implementation)
 */
int crtprosSendPacketBlock(CRTPPacket *p);

/**
 * Fetch a packet with a specidied task ID.
 *
 * @param[in]  taskId The id of the CRTPROS task
 * @param[out] p      The CRTPROS Packet with infomation (unchanged if nothing to fetch)
 *
 * @returns status of fetch from queue
 */
int crtprosReceivePacket(CRTPPort taskId, CRTPPacket *p);

/**
 * Fetch a packet with a specidied task ID. Wait some time befor giving up
 *
 * @param[in]  taskId The id of the CRTPROS task
 * @param[out] p      The CRTPROS Packet with infomation (unchanged if nothing to fetch)
 * @param[in] wait    Wait time in milisecond
 *
 * @returns status of fetch from queue
 */
int crtprosReceivePacketWait(CRTPPort taskId, CRTPPacket *p, int wait);

/**
 * Get the number of free tx packets in the queue
 *
 * @return Number of free packets
 */
int crtprosGetFreeTxQueuePackets(void);

/**
 * Wait for a packet to arrive for the specified taskID
 *
 * @param[in]  taskId The id of the CRTPROS task
 * @paran[out] p      The CRTPROS Packet with information
 *
 * @return status of fetch from queue
 */
int crtprosReceivePacketBlock(CRTPPort taskId, CRTPPacket *p);

/**
 * Function pointer structure to be filled by the CRTPROS link to permits CRTPROS to
 * use manu link
 */
struct crtprosLinkOperations
{
  int (*setEnable)(bool enable);
  int (*sendPacket)(CRTPPacket *pk);
  int (*receivePacket)(CRTPPacket *pk);
  bool (*isConnected)(void);
  int (*reset)(void);
};

void crtprosSetLink(struct crtprosLinkOperations * lk);

/**
 * Check if the connection timeout has been reached, otherwise
 * we will assume that we are connected.
 *
 * @return true if conencted, otherwise false
 */
bool crtprosIsConnected(void);

/**
 * Reset the CRTPROS communication by flushing all the queues that
 * contain packages.
 *
 * @return 0 for success
 */
int crtprosReset(void);

#endif /*CRTPROS_H_*/
