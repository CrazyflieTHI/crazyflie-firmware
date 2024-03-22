/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie simulation firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *               2023 Thomas Izycki, THA, Germany
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
 * simlink.c: POSIX message queue implementation of the CRTP Gazebo link
 */

#include <string.h>
#include <stdint.h>

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include <math.h>
#include "configblock_sim.h"
#include "crtpros.h"
#include "gazebolink.h"
#include "crtp.h"
#include "static_mem.h"
#include "cfassert.h"
#include "posixmq_wrapper.h"
#include "debug.h"
#include "errno.h"

#define SIMLINK_TX_QUEUE_SIZE (1)
#define SIMLINK_CRTP_QUEUE_SIZE (5)
#define SIM_ACTIVITY_TIMEOUT_MS (1000)

#define SIMLINK_P2P_QUEUE_SIZE (5)

static bool isInit;

/* Caution! The rxQueue gets the name of the gazebo txQueue
   and the txQueue gets the name of the gazebo rxQueue */ 
static char *rxIpcCrtpQueueNameBase = "/txgazebocrtpmq";
static char *txIpcCrtpQueueNameBase = "/rxgazebocrtpmq";
static char rxIpcCrtpQueueName[20];
static char txIpcCrtpQueueName[20];
static char cfIdStr[3];

static int gazebolinkSendCRTPPacket(CRTPPacket *p);
static int gazebolinkSetEnable(bool enable);
static int gazebolinkReceiveCRTPPacket(CRTPPacket *p);

static const unsigned long timeout = 100000000ul; // 100ms

// Posix message queues for ipc
static mqd_t rxSimQueue;
static mqd_t txSimQueue;

// static bool gazebolinkIsConnected(void) {
//   // return (xTaskGetTickCount() - lastPacketTick) < M2T(SIM_ACTIVITY_TIMEOUT_MS);
//   return isInit;
// }

static struct crtprosLinkOperations gazebolinkOp =
{
  .setEnable         = gazebolinkSetEnable,
  .sendPacket        = gazebolinkSendCRTPPacket,
  .receivePacket     = gazebolinkReceiveCRTPPacket,
};

void gazebolinkInit(void)
{
  int errnum;

  if (isInit)
    return;

  /* crazyflieId gets set in main_sitl */
  uint8_t crazyflieId = getCrazyflieId();

  /* Convert it to two digit hexadecimal representation */
  sprintf(cfIdStr, "%02X", crazyflieId);

  memcpy(rxIpcCrtpQueueName, rxIpcCrtpQueueNameBase, strlen(rxIpcCrtpQueueNameBase));
  memcpy(&rxIpcCrtpQueueName[strlen(rxIpcCrtpQueueName)],
         cfIdStr, 2*sizeof(char));
  printf("[gazebolink] Name of rx queue: %s\n", rxIpcCrtpQueueName);

  memcpy(txIpcCrtpQueueName, txIpcCrtpQueueNameBase, strlen(txIpcCrtpQueueNameBase));
  memcpy(&txIpcCrtpQueueName[strlen(txIpcCrtpQueueName)],
          cfIdStr, 2*sizeof(char));
  printf("[gazebolink] Name of tx queue: %s\n", txIpcCrtpQueueName);

  rxSimQueue = openMqPosixNonblock(rxIpcCrtpQueueName, 10, 32, &errnum);
  if(rxSimQueue == -1) {
    DEBUG_PRINT("ERROR %d: [gazebolink] Couldn't open POSIX mq RX.\n", errnum);
    return;
  }

  txSimQueue = openMqPosixNonblock(txIpcCrtpQueueName, 10, 32, &errnum);
  if(txSimQueue == -1) {
    DEBUG_PRINT("ERROR %d: [gazebolink] Couldn't open POSIX mq TX.\n", errnum);
    return;
  }

  // Wait for validation by the SITL instance
  DEBUG_PRINT("Waiting for connection with gazebo ... \n");
  bool commInitialized = false;
  int recvlen;
  uint8_t count;
  const uint8_t max_count = 10;
  CRTPPacket p;
  unsigned rxPrio = 31;
  GazebolinkPacket glp;

  p.header = 0xF3;  // send null header for identification process
  p.size = 0;       // No data when doing identification process
  while(!commInitialized)
  {
    count = 0;
    gazebolinkSendCRTPPacket(&p);
    while(count < max_count)
    {
      while ((recvlen = rcvMqPosixTimed(rxSimQueue, (char*)&glp, sizeof(GazebolinkPacket), &rxPrio, timeout, &errnum) == -1) || (errnum == EINTR))
        vTaskDelay(M2T(10));

      if (glp.data[1] == 0xF3)
      {
          commInitialized = true;
          break;
      }
      count++;
      vTaskDelay(M2T(10));
    }
  }

  DEBUG_PRINT("Connection established with gazebo \n");

  isInit = true;
}

static int gazebolinkReceiveCRTPPacket(CRTPPacket *p)
{
  GazebolinkPacket glp;
  unsigned rxPrio = 31;
  int errnum = 0;
  int ret = -1;
  long timeout = 10e7;

  /* https://stackoverflow.com/questions/29245596/c-gracefully-interrupting-msgrcv-system-call */
  /* Without the delay other tasks seem to starve in simulation */
  while ((ret = rcvMqPosixTimed(rxSimQueue, (char*)&glp, sizeof(GazebolinkPacket), &rxPrio, timeout, &errnum) == -1) || (errnum == EINTR))
    vTaskDelay(M2T(1));

  if(ret == -1) {
    DEBUG_PRINT("ERROR RX [gazebolink]: %d\n", errnum);
    vTaskDelay(M2T(500));
  }

  if (ret >= 0)
  {
    memcpy(p, &glp.data, sizeof(CRTPPacket));
    // DEBUG_PRINT("MESSAGE RX CRTP port: %d\n", p->port);
    // DEBUG_PRINT("MESSAGE RX CRTP channel: %d\n", p->channel);
    return 0;
  }

  return -1;
}

static int gazebolinkSendCRTPPacket(CRTPPacket *p)
{
  static GazebolinkPacket glp;
  unsigned txPrio = 31;
  int errnum;
  int ret = -1;

  // ASSERT(p->size <= CRTP_MAX_DATA_SIZE);

  memcpy(&glp.data, p, sizeof(CRTPPacket));

  while(ret == -1) {
    ret = sendMqPosixTimed(txSimQueue, (char*)&glp, sizeof(GazebolinkPacket), txPrio, timeout, &errnum);
    vTaskDelay(M2T(1));
  }

  if(ret == -1) {
    DEBUG_PRINT("ERROR TX [gazebolink]: %d\n", errnum);
    vTaskDelay(M2T(500));
    return false;
  }
  // DEBUG_PRINT("TX CRTP port: %d\n", p->port);
  // DEBUG_PRINT("TX CRTP channel: %d\n", p->channel);
  return true;
}

struct crtprosLinkOperations * gazebolinkGetLink()
{
  return &gazebolinkOp;
}

static int gazebolinkSetEnable(bool enable)
{
  return 0;
}
