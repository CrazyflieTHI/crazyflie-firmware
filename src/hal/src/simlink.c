/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie simulation firmware
 *
 * Copyright (C) 2012-2021 BitCraze AB
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
 * simlink.c: POSIX message queue implementation of the CRTP sim link
 */

#include <string.h>
#include <stdint.h>

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include <math.h>
#include "gazebolink.h"
#include "simlink.h"
#include "crtp.h"
#include "static_mem.h"
#include "cfassert.h"
#include "posixmq_wrapper.h"
#include "debug.h"
#include "errno.h"
#include "configblock_sim.h"

#define SIMLINK_TX_QUEUE_SIZE (1)
#define SIMLINK_CRTP_QUEUE_SIZE (5)
#define SIM_ACTIVITY_TIMEOUT_MS (1000)

#define SIMLINK_P2P_QUEUE_SIZE (5)

static bool isInit;

static char *rxIpcCrtpQueueNameBase = "/txsimcrtpmq";
static char *txIpcCrtpQueueNameBase = "/rxsimcrtpmq";
static char rxIpcCrtpQueueName[20];
static char txIpcCrtpQueueName[20];
static char cfIdStr[4];

static int simlinkSendCRTPPacket(CRTPPacket *p);
static int simlinkSetEnable(bool enable);
static int simlinkReceiveCRTPPacket(CRTPPacket *p);

static const unsigned long timeout = 100000000ul; // 100ms

// Posix message queues for ipc
static mqd_t rxSimQueue;
static mqd_t txSimQueue;

static bool simlinkIsConnected(void) {
  // return (xTaskGetTickCount() - lastPacketTick) < M2T(SIM_ACTIVITY_TIMEOUT_MS);
  return isInit;
}

static struct crtpLinkOperations simlinkOp =
{
  .setEnable     = simlinkSetEnable,
  .sendPacket    = simlinkSendCRTPPacket,
  .receivePacket = simlinkReceiveCRTPPacket,
  .isConnected   = simlinkIsConnected
};

void simlinkInit(void)
{
  int errnum;

  if (isInit)
    return;

  /* crazyflieId gets set in main_sitl */
  uint8_t crazyflieId = getCrazyflieId();
  int digitCount = log10(crazyflieId) + 1;
  if(digitCount == 1) {
    cfIdStr[0] = '0';
    cfIdStr[1] = (crazyflieId % 10) + '0';
  } else if(digitCount == 2) {
    for (int i = digitCount-1; i >= 0; --i, crazyflieId /= 10)
    {
        cfIdStr[i] = (crazyflieId % 10) + '0';
    }
  } else {
    printf("ERROR: Simlink Init.\n");
    return;
  }

  memcpy(rxIpcCrtpQueueName, rxIpcCrtpQueueNameBase, strlen(rxIpcCrtpQueueNameBase));
  memcpy(&rxIpcCrtpQueueName[strlen(rxIpcCrtpQueueName)],
         cfIdStr, 2*sizeof(char));
  // printf("Name of rx queue: %s\n", rxIpcCrtpQueueName);

  memcpy(txIpcCrtpQueueName, txIpcCrtpQueueNameBase, strlen(txIpcCrtpQueueNameBase));
  memcpy(&txIpcCrtpQueueName[strlen(txIpcCrtpQueueName)],
          cfIdStr, 2*sizeof(char));
  // printf("Name of tx queue: %s\n", txIpcCrtpQueueName);

  rxSimQueue = openMqPosixNonblock(rxIpcCrtpQueueName, 10, 33, &errnum);
  if(rxSimQueue == -1) {
    DEBUG_PRINT("ERROR %d: Couldn't open POSIX mq RX.\n", errnum);
    return;
  }

  txSimQueue = openMqPosixNonblock(txIpcCrtpQueueName, 10, 33, &errnum);
  if(txSimQueue == -1) {
    DEBUG_PRINT("ERROR %d: Couldn't open POSIX mq TX.\n", errnum);
    return;
  }
  isInit = true;
}

static int simlinkReceiveCRTPPacket(CRTPPacket *p)
{
  SimlinkPacket slp;
  unsigned rxPrio = 0;
  int errnum = 0;
  int ret = -1;

  /* https://stackoverflow.com/questions/29245596/c-gracefully-interrupting-msgrcv-system-call */
  /* Without the delay other tasks seem to starve in simulation */
  while ((ret = rcvMqPosixTimed(rxSimQueue, (char*)&slp, sizeof(SimlinkPacket), &rxPrio, timeout, &errnum) == -1) || (errnum == EINTR))
    vTaskDelay(M2T(2));

  if(ret == -1) {
    DEBUG_PRINT("ERROR RX Simlink: %d\n", errnum);
    vTaskDelay(M2T(500));
  }

  /* Without the delay the queue runs full in simulation */
  vTaskDelay(M2T(2));

  if (ret >= 0)
  {
    memcpy(p, &slp.data, sizeof(CRTPPacket));
    // DEBUG_PRINT("MESSAGE RX CRTP port: %d\n", p->port);
    // DEBUG_PRINT("MESSAGE RX CRTP channel: %d\n", p->channel);
    return 0;
  }

  return -1;
}

static int simlinkSendCRTPPacket(CRTPPacket *p)
{
  static SimlinkPacket slp;
  unsigned txPrio = 2;
  int errnum;
  int ret = -1;

  // ASSERT(p->size <= CRTP_MAX_DATA_SIZE);

  slp.port = (uint8_t)PORT_IPC_CRTP;
  memcpy(&slp.data, p, sizeof(CRTPPacket));

  while(ret == -1) {
    ret = sendMqPosixTimed(txSimQueue, (char*)&slp, sizeof(SimlinkPacket), txPrio, timeout, &errnum);
    vTaskDelay(M2T(1));
  }

  if(ret == -1) {
    DEBUG_PRINT("ERROR TX Simlink: %d\n", errnum);
    vTaskDelay(M2T(500));
    return false;
  }
  // DEBUG_PRINT("TX CRTP port: %d\n", p->port);
  // DEBUG_PRINT("TX CRTP channel: %d\n", p->channel);
  return true;
}

struct crtpLinkOperations * simlinkGetLink()
{
  return &simlinkOp;
}

static int simlinkSetEnable(bool enable)
{
  return 0;
}
