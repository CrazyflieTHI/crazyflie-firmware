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
 * crtpros.c - CrazyRealtimeTransferProtocol stack
 */

#include <stdbool.h>
#include <errno.h>

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "config.h"

#include "crtpros.h"
#include "crtp.h"
#include "info.h"
#include "cfassert.h"
#include "queuemonitor.h"
#include "static_mem.h"

#include "log.h"
#include "debug.h"

static bool isInit;

static int nopFunc(void);
static struct crtprosLinkOperations nopLink = {
  .setEnable         = (void*) nopFunc,
  .sendPacket        = (void*) nopFunc,
  .receivePacket     = (void*) nopFunc,
};

static struct crtprosLinkOperations *crtpros_link = &nopLink;

#define STATS_INTERVAL 500
static struct {
  uint32_t rxCount;
  uint32_t txCount;

  uint16_t rxRate;
  uint16_t txRate;

  uint32_t nextStatisticsTime;
  uint32_t previousStatisticsTime;
} stats;

static xQueueHandle  txQueue;

#define CRTPROS_NBR_OF_PORTS 16
#define CRTPROS_TX_QUEUE_SIZE 120
#define CRTPROS_RX_QUEUE_SIZE 16

static void crtprosTxTask(void *param);
static void crtprosRxTask(void *param);

static xQueueHandle queues[CRTPROS_NBR_OF_PORTS];
static volatile CrtpCallback callbacks[CRTPROS_NBR_OF_PORTS];
static void updateStats();

STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(crtprosTxTask, CRTPROS_TX_TASK_STACKSIZE);
STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(crtprosRxTask, CRTPROS_RX_TASK_STACKSIZE);

void crtprosInit(void)
{
  if(isInit)
    return;

  txQueue = xQueueCreate(CRTPROS_TX_QUEUE_SIZE, sizeof(CRTPPacket));
  DEBUG_QUEUE_MONITOR_REGISTER(txQueue);

  STATIC_MEM_TASK_CREATE(crtprosTxTask, crtprosTxTask, CRTPROS_TX_TASK_NAME, NULL, CRTPROS_TX_TASK_PRI);
  STATIC_MEM_TASK_CREATE(crtprosRxTask, crtprosRxTask, CRTPROS_RX_TASK_NAME, NULL, CRTPROS_RX_TASK_PRI);

  isInit = true;
}

bool crtprosTest(void)
{
  return isInit;
}

void crtprosInitTaskQueue(CRTPPort portId)
{
  ASSERT(queues[portId] == NULL);

  queues[portId] = xQueueCreate(CRTPROS_RX_QUEUE_SIZE, sizeof(CRTPPacket));
  DEBUG_QUEUE_MONITOR_REGISTER(queues[portId]);
}

int crtprosReceivePacket(CRTPPort portId, CRTPPacket *p)
{
  ASSERT(queues[portId]);
  ASSERT(p);

  return xQueueReceive(queues[portId], p, 0);
}

int crtprosReceivePacketBlock(CRTPPort portId, CRTPPacket *p)
{
  ASSERT(queues[portId]);
  ASSERT(p);

  return xQueueReceive(queues[portId], p, portMAX_DELAY);
}


int crtprosReceivePacketWait(CRTPPort portId, CRTPPacket *p, int wait)
{
  ASSERT(queues[portId]);
  ASSERT(p);

  return xQueueReceive(queues[portId], p, M2T(wait));
}

int crtprosGetFreeTxQueuePackets(void)
{
  return (CRTPROS_TX_QUEUE_SIZE - uxQueueMessagesWaiting(txQueue));
}

void crtprosTxTask(void *param)
{
  CRTPPacket p;

  while (true)
  {
    if (crtpros_link != &nopLink)
    {
      if (xQueueReceive(txQueue, &p, portMAX_DELAY) == pdTRUE)
      {
        // Keep testing, if the link changes to USB it will go though
        while (crtpros_link->sendPacket(&p) == false)
        {
          // Relaxation time
          vTaskDelay(M2T(10));
        }
        stats.txCount++;
        updateStats();
      }
    }
    else
    {
      vTaskDelay(M2T(10));
    }
  }
}

void crtprosRxTask(void *param)
{
  CRTPPacket p;

  while (true)
  {
    if (crtpros_link != &nopLink)
    {
      if (!crtpros_link->receivePacket(&p))
      {
        if (queues[p.port])
        {
          // Block, since we should never drop a packet
          xQueueSend(queues[p.port], &p, portMAX_DELAY);
        }

        if (callbacks[p.port])
        {
          callbacks[p.port](&p);
        }

        stats.rxCount++;
        updateStats();
      }
    }
    else
    {
      vTaskDelay(M2T(10));
    }
  }
}

void crtprosRegisterPortCB(int port, CrtpCallback cb)
{
  if (port>CRTPROS_NBR_OF_PORTS)
    return;

  callbacks[port] = cb;
}

int crtprosSendPacket(CRTPPacket *p)
{
  ASSERT(p);
  ASSERT(p->size <= CRTP_MAX_DATA_SIZE);

  return xQueueSend(txQueue, p, 0);
}

int crtprosSendPacketBlock(CRTPPacket *p)
{
  ASSERT(p);
  ASSERT(p->size <= CRTP_MAX_DATA_SIZE);

  return xQueueSend(txQueue, p, portMAX_DELAY);
}

int crtprosReset(void)
{
  xQueueReset(txQueue);
  if (crtpros_link->reset) {
    crtpros_link->reset();
  }

  return 0;
}

bool crtprosIsConnected(void)
{
  if (crtpros_link->isConnected)
    return crtpros_link->isConnected();
  return true;
}

void crtprosSetLink(struct crtprosLinkOperations * lk)
{
  if(crtpros_link)
    crtpros_link->setEnable(false);

  if (lk)
    crtpros_link = lk;
  else
    crtpros_link = &nopLink;

  crtpros_link->setEnable(true);
}

static int nopFunc(void)
{
  return ENETDOWN;
}

static void clearStats()
{
  stats.rxCount = 0;
  stats.txCount = 0;
}

static void updateStats()
{
  uint32_t now = xTaskGetTickCount();
  if (now > stats.nextStatisticsTime) {
    float interval = now - stats.previousStatisticsTime;
    stats.rxRate = (uint16_t)(1000.0f * stats.rxCount / interval);
    stats.txRate = (uint16_t)(1000.0f * stats.txCount / interval);

    clearStats();
    stats.previousStatisticsTime = now;
    stats.nextStatisticsTime = now + STATS_INTERVAL;
  }
}

LOG_GROUP_START(crtpros)
LOG_ADD(LOG_UINT16, rxRate, &stats.rxRate)
LOG_ADD(LOG_UINT16, txRate, &stats.txRate)
LOG_GROUP_STOP(tdoa)
