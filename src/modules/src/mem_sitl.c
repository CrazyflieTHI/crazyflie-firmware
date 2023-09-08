/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie simulation firmware
 *
 * Copyright (c) 2018  Eric Goubault, Sylvie Putot, Franck Djeumou
 *             Cosynux , LIX , France
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
 * mem_sitl.c: Memory module. simplified version of mem_cf2 for simulation.
 * It is useless but an implementation free from driver dep was needed to be
 * able to handle trajectory upload
 */

#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "config.h"
#include "crtp.h"
#include "mem.h"

#include "crtp_commander_high_level.h"

#include "console.h"
#include "assert.h"
#include "debug.h"

#if 0
#define MEM_DEBUG(fmt, ...) DEBUG_PRINT("D/log " fmt, ## __VA_ARGS__)
#define MEM_ERROR(fmt, ...) DEBUG_PRINT("E/log " fmt, ## __VA_ARGS__)
#else
#define MEM_DEBUG(...)
#define MEM_ERROR(...)
#endif


// Maximum log payload length
#define MEM_MAX_LEN 30

#define MEM_SETTINGS_CH     0
#define MEM_READ_CH         1
#define MEM_WRITE_CH        2

#define MEM_CMD_GET_NBR     1
#define MEM_CMD_GET_INFO    2

// The first part of the memory ids are static followed by a dynamic part
// of one wire ids that depends on the decks that are attached
#define EEPROM_ID       0x00
#define LEDMEM_ID       0x01
#define LOCO_ID         0x02
#define TRAJ_ID         0x03
#define OW_FIRST_ID     0x04

#define STATUS_OK 0

#define MEM_TYPE_EEPROM 0x00
#define MEM_TYPE_OW     0x01
#define MEM_TYPE_LED12  0x10
#define MEM_TYPE_LOCO   0x11
#define MEM_TYPE_TRAJ   0x12

#define MEM_LOCO_INFO             0x0000
#define MEM_LOCO_ANCHOR_BASE      0x1000
#define MEM_LOCO_ANCHOR_PAGE_SIZE 0x0100
#define MEM_LOCO_PAGE_LEN         (3 * sizeof(float) + 1)

#define MEM_TESTER_SIZE            0x1000

#define TRAJECTORY_MEMORY_SIZE 4096
extern uint8_t trajectories_memory[TRAJECTORY_MEMORY_SIZE];

//Private functions
static void memTask(void * prm);
static void memSettingsProcess(int command);
static void memWriteProcess(void);
static void memReadProcess(void);
static void createNbrResponse(CRTPPacket* p);
static void createInfoResponse(CRTPPacket* p, uint8_t memId);
static void createInfoResponseBody(CRTPPacket* p, uint8_t type, uint32_t memSize, const uint8_t data[8]);

static uint32_t handleMemTesterGetSize(void) { return MEM_TESTER_SIZE; }
static bool handleMemTesterRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
static bool handleMemTesterWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* startOfData);
static uint32_t memTesterWriteErrorCount = 0;
static uint8_t memTesterWriteReset = 0;
static const MemoryHandlerDef_t memTesterDef = {
  .type = MEM_TYPE_TESTER,
  .getSize = handleMemTesterGetSize,
  .read = handleMemTesterRead,
  .write = handleMemTesterWrite,
};

static bool isInit = false;
static bool registrationEnabled = true;

static uint8_t nbrOwMems = 0;

static const uint8_t noData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
static CRTPPacket p;

#define MAX_NR_HANDLERS 20
static const MemoryHandlerDef_t* handlers[MAX_NR_HANDLERS];
static uint8_t nrOfHandlers = 0;
// static const MemoryOwHandlerDef_t* owMemHandler = 0;

void memInit(void)
{
  if(isInit)
    return;

  memoryRegisterHandler(&memTesterDef);

  //Start the mem task
  xTaskCreate(memTask, MEM_TASK_NAME,
              MEM_TASK_STACKSIZE, NULL, MEM_TASK_PRI, NULL);
  
  isInit = true;
}

bool memTest(void)
{
  return isInit;
}

void memoryRegisterHandler(const MemoryHandlerDef_t* handlerDef){
  for (int i = 0; i < nrOfHandlers; i++) {
    ASSERT(handlerDef->type != handlers[i]->type);
  }
  ASSERT(nrOfHandlers < MAX_NR_HANDLERS);
  ASSERT(registrationEnabled);
  handlers[nrOfHandlers] = handlerDef;
  nrOfHandlers++;
}

void memTask(void * param)
{
	crtpInitTaskQueue(CRTP_PORT_MEM);
	
	while(1)
	{
		crtpReceivePacketBlock(CRTP_PORT_MEM, &p);

		switch (p.channel)
		{
      case MEM_SETTINGS_CH:
        memSettingsProcess(p.data[0]);
        break;
      case MEM_READ_CH:
        memReadProcess();
        break;
      case MEM_WRITE_CH:
        memWriteProcess();
        break;
      default:
        break;
		}
	}
}

void memSettingsProcess(int command)
{
  switch (command)
  {
    case MEM_CMD_GET_NBR:
      createNbrResponse(&p);
      crtpSendPacket(&p);
      break;

    case MEM_CMD_GET_INFO:
      {
        uint8_t memId = p.data[1];
        createInfoResponse(&p, memId);
        crtpSendPacket(&p);
      }
      break;
  }
}

void createNbrResponse(CRTPPacket* p)
{
  p->header = CRTP_HEADER(CRTP_PORT_MEM, MEM_SETTINGS_CH);
  p->size = 2;
  p->data[0] = MEM_CMD_GET_NBR;
  p->data[1] = nbrOwMems + OW_FIRST_ID;
}

void createInfoResponse(CRTPPacket* p, uint8_t memId)
{
  p->header = CRTP_HEADER(CRTP_PORT_MEM, MEM_SETTINGS_CH);
  p->size = 2;
  p->data[0] = MEM_CMD_GET_INFO;
  p->data[1] = memId;

  // No error code if we fail, just send an empty packet back
  switch(memId)
  {
    case TRAJ_ID:
      createInfoResponseBody(p, MEM_TYPE_TRAJ, sizeof(trajectories_memory), noData);
      break;
    default:
      break;
  }
}

void createInfoResponseBody(CRTPPacket* p, uint8_t type, uint32_t memSize, const uint8_t data[8])
{
  p->data[2] = type;
  p->size += 1;

  memcpy(&p->data[3], &memSize, 4);
  p->size += 4;

  memcpy(&p->data[7], data, 8);
  p->size += 8;
}


void memReadProcess()
{
  uint8_t memId = p.data[0];
  uint8_t readLen = p.data[5];
  uint32_t memAddr;
  uint8_t status = STATUS_OK;

  memcpy(&memAddr, &p.data[1], 4);

  MEM_DEBUG("Packet is MEM READ\n");
  p.header = CRTP_HEADER(CRTP_PORT_MEM, MEM_READ_CH);
  // Dont' touch the first 5 bytes, they will be the same.

  switch(memId)
  {
    case EEPROM_ID:
      status = EIO;
      break;

    case LEDMEM_ID:
      status = EIO;
      break;

    case LOCO_ID:
      status = EIO;
      break;

    case TRAJ_ID:
      {
        if (memAddr + readLen <= sizeof(trajectories_memory) &&
            memcpy(&p.data[6], &(trajectories_memory[memAddr]), readLen)) {
          status = STATUS_OK;
        } else {
          status = EIO;
        }
      }
      break;

    default:
      status = EIO;
      break;
  }

#if 0
  {
    int i;
    for (i = 0; i < readLen; i++)
      consolePrintf("%X ", p.data[i+6]);

    consolePrintf("\nStatus %i\n", status);
  }
#endif

  p.data[5] = status;
  if (status == STATUS_OK)
    p.size = 6 + readLen;
  else
    p.size = 6;


  crtpSendPacket(&p);
}

void memWriteProcess()
{
  uint8_t memId = p.data[0];
  uint8_t writeLen;
  uint32_t memAddr;
  uint8_t status = STATUS_OK;

  memcpy(&memAddr, &p.data[1], 4);
  writeLen = p.size - 5;

  MEM_DEBUG("Packet is MEM WRITE\n");
  p.header = CRTP_HEADER(CRTP_PORT_MEM, MEM_WRITE_CH);
  // Dont' touch the first 5 bytes, they will be the same.

  switch(memId)
  {
    case EEPROM_ID:
      status = EIO;
      break;

    case LEDMEM_ID:
      status = EIO;
      break;

    case LOCO_ID:
      // Not supported
      status = EIO;
      break;

    case TRAJ_ID:
      {
        if ((memAddr + writeLen) <= sizeof(trajectories_memory)) {
          memcpy(&(trajectories_memory[memAddr]), &p.data[5], writeLen);
          status = STATUS_OK;
        } else {
          status = EIO;
        }
      }
      break;

    default:
      status = EIO;
      break;
  }

  p.data[5] = status;
  p.size = 6;

  crtpSendPacket(&p);
}

/**
 * @brief The memory tester is used to verify the functionality of the memory sub system.
 * It supports "virtual" read and writes that are used by a test script to
 * check that a client (for instance the python lib) is working as expected.
 *
 * When reading data from the tester, it simply fills up a buffer with known data so that the
 * client can examine the data and verify that buffers have been correctly assembled.
 *
 * @param memAddr - the virtual address to read from
 * @param readLen - nr of bytes to read
 * @param startOfData - address to write result to
 * @return Always returns true
 */
static bool handleMemTesterRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* startOfData) {
  for (int i = 0; i < readLen; i++) {
    uint32_t addr = memAddr + i;
    uint8_t data = addr & 0xff;
    startOfData[i] = data;
  }

  return true;
}

/**
 * @brief When writing data to the tester, the tester verifies that the received data
 * contains the expected values.
 *
 * @param memAddr - the virtual address to write to
 * @param writeLen - nr of bytes to write
 * @param startOfData - pointer to the data in the packet that is provided by the client
 * @return Always returns true
 */
static bool handleMemTesterWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* startOfData) {
  if (memTesterWriteReset) {
    memTesterWriteReset = 0;
    memTesterWriteErrorCount = 0;
  }

  for (int i = 0; i < writeLen; i++) {
    uint32_t addr = memAddr + i;
    uint8_t expectedData = addr & 0xff;
    uint8_t actualData = startOfData[i];
    if (actualData != expectedData) {
      // Log first error
      if (memTesterWriteErrorCount == 0) {
        DEBUG_PRINT("Verification failed: expected: %d, actual: %d, addr: %u\n", expectedData, actualData, addr);
      }

      memTesterWriteErrorCount++;
      break;
    }
  }

  return true;
}
