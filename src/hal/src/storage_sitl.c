/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 - 2020 Bitcraze AB
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
 *
 * storage.c: Key/Buffer persistent storage
 *
 */

#include "storage.h"
#include "param.h"

#include "FreeRTOS.h"

#include <string.h>

#define TRACE_MEMORY_ACCESS 0

#if !TRACE_MEMORY_ACCESS
#define DEBUG_MODULE "STORAGE"
#endif
#include "debug.h"

// Memory organization

// Low level memory access

// ToDo: Shall we handle partition elsewhere?
#define KVE_PARTITION_START (1024)
#define KVE_PARTITION_LENGTH (0)

static size_t readEeprom(size_t address, void* data, size_t length)
{
  return 0;
}

static size_t writeEeprom(size_t address, const void* data, size_t length)
{
  return 0;
}

static void flushEeprom(void)
{
  // NOP for now, lets fix the EEPROM write first!
}

// Public API

static bool isInit = false;

void storageInit()
{
  isInit = true;
}

bool storageTest()
{
  return true;
}

bool storageStore(const char* key, const void* buffer, size_t length)
{
  return true;
}


bool storageForeach(const char *prefix, storageFunc_t func)
{
  return true;
}

size_t storageFetch(const char *key, void* buffer, size_t length)
{
  return true;
}

bool storageDelete(const char* key)
{
  return true;
}

void storagePrintStats()
{
  DEBUG_PRINT("Storage SITL\n");
}

static bool storageStats;

static void printStats(void)
{
  if (storageStats) {
    storagePrintStats();

    storageStats = false;
  }
}

PARAM_GROUP_START(system)

/**
 * @brief Set to nonzero to dump CPU and stack usage to console
 */
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, storageStats, &storageStats, printStats)

PARAM_GROUP_STOP(system)
