/**
 *                                             __
 *    __  __ ____      ____  __  __ __________/ /_  __   _____________
 *   / /_/ / ___/____ / __ `/ / / /´__  / ___/ __ \/ /  / / ___/ __  /
 *  / __  (__  )/___// /_/ / /_/ / /_/ (__  ) /_/ / /__/ / /  / /_/ /
 * /_/ /_/____/      \__,_/_____/\__  /____/\____/______/_/   \__  /
 *                              /____/                       /____/
 *
 * Crazyflie control firmware for multi-agent systems
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
 * macp.c - Multi-Agent Communication Protocol
 */

#include "FreeRTOS.h"
#include <string.h>
#include "task.h"
#include "queue.h"
#include "config.h"

#include "macp.h"
#include "static_mem.h"
#include "configblock.h"
#include "crtp.h"
#include "cfassert.h"
#include "debug.h"

#define MACP_RX_QUEUE_SIZE 25
#define MACP_TX_QUEUE_SIZE 10
#define MACP_NBR_OF_PORTS 5

static uint8_t ownAddr;

static bool isInit = false;

static void macpRxTask(void *param);

static xQueueHandle queues[MACP_NBR_OF_PORTS];
static volatile MacpCallback callbacks[MACP_NBR_OF_PORTS];

static xQueueHandle macpRxQueue;

STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(macpRxTask, MACP_RX_TASK_STACKSIZE);

void macpInit(void)
{
  if(isInit)
    return;

  crtpInitTaskQueue(MACP_CRTP_PORT);

  macpRxQueue = xQueueCreate(MACP_RX_QUEUE_SIZE, sizeof(CRTPPacket));

  STATIC_MEM_TASK_CREATE(macpRxTask, macpRxTask, MACP_RX_TASK_NAME, NULL, MACP_RX_TASK_PRI);

  isInit = true;
}

bool macpTest(void)
{
  return isInit;
}

void macpRxTaskInitQueue(MACPPort portId)
{
  ASSERT(queues[portId] == NULL);

  queues[portId] = xQueueCreate(MACP_TX_QUEUE_SIZE, sizeof(MACPPacket));
}

void macpRegisterPortCB(MACPPort portId, MacpCallback cb)
{
  if (portId>MACP_NBR_OF_PORTS)
    return;

  callbacks[portId] = cb;
}

static void macpRxTask(void *param)
{
  CRTPPacket crtpPacket;
  MACPPacket* macpPacket;

  uint64_t address = configblockGetRadioAddress();
  ownAddr = (uint8_t)((address) & 0x00000000FF);
  DEBUG_PRINT("Own MACP address: %d\n", ownAddr);

  while (true)
  {
    crtpReceivePacketBlock(MACP_CRTP_PORT, &crtpPacket);

    macpPacket = (MACPPacket*)&crtpPacket.data;
    // DEBUG_PRINT("Received macp port %d\n", macpPacket->port);
    if (queues[macpPacket->port])
    {
      // Block, since we should never drop a packet
      xQueueSend(queues[macpPacket->port], macpPacket, portMAX_DELAY);
    }

    if (callbacks[macpPacket->port])
    {
      callbacks[macpPacket->port](macpPacket);
    }
  }
}

int macpSendPacket(uint8_t destAddr, uint8_t macpPort, uint8_t macpSubPort,
                   uint8_t *payload, uint8_t payloadSize)
{
  CRTPPacket rawCRTPPacket;

  if(payloadSize > MAX_MACP_DATA_SIZE) {
      DEBUG_PRINT("MACP packet payload size too big!\n");
      return pdFALSE;
  }

  rawCRTPPacket.header = CRTP_HEADER(MACP_CRTP_PORT, MACP_CLIENT_ADDR);
  rawCRTPPacket.size = payloadSize + 3;
  /* MACP header */
  rawCRTPPacket.data[0] = MACP_HEADER_ADDR(destAddr, ownAddr);
  rawCRTPPacket.data[1] = macpPort;
  rawCRTPPacket.data[2] = macpSubPort;
  /* MACP payload */
  memcpy(&rawCRTPPacket.data[3], payload, payloadSize);

  return crtpSendPacket(&rawCRTPPacket);
}

uint8_t getOwnMacpId()
{
  return ownAddr;
}
