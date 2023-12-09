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
 * configblocksitl.c - Simple static implementation of the config block
 * remove driver/low level dependencies and basically return static variables
 * for every functions
 */
#define DEBUG_MODULE "CFGBLK"

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>

#include "config.h"
#include "debug.h"
#include "configblock.h"

#define RADIO_ADDRESS_SIM_BASE 0xE7E7E7E7E7UL
#define RADIO_ADDRESS_SIM_MASK 0xFFFFFFFF00UL

static uint8_t crazyflieId = 0;
static uint64_t crazyflieAddr = 0;
static bool isInit = false;

int configblockInit(void)
{
  if(isInit)
    return 0;

  isInit = true;

  return 0;
}

bool configblockTest(void)
{
  return true;
}

/* Static accessors */
int configblockGetRadioChannel(void)
{
    return RADIO_CHANNEL;
}

int configblockGetRadioSpeed(void)
{
    return 0;
}

uint64_t configblockGetRadioAddress(void)
{
    return crazyflieAddr;
}

float configblockGetCalibPitch(void)
{
    return 0;
}

float configblockGetCalibRoll(void)
{
    return 0;
}

void setCrazyflieId(uint8_t newCrazyflieId)
{
  crazyflieId = newCrazyflieId;
  crazyflieAddr = (RADIO_ADDRESS_SIM_BASE & RADIO_ADDRESS_SIM_MASK) | crazyflieId;
}

uint8_t getCrazyflieId()
{
  return crazyflieId;
}
