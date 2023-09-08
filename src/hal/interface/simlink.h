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

#ifndef __SIMLINK_H__
#define __SIMLINK_H__

#include <stdbool.h>
#include "crtp.h"

#define SIMLINK_PAYLOAD_SIZE 32

typedef struct _SimlinkPacket
{
  uint8_t port;
  char data[SIMLINK_PAYLOAD_SIZE];
} __attribute__((packed)) SimlinkPacket;

typedef enum
{
  waitForFirstStartSim,
  waitForSecondStartSim,
  waitForTypeSim,
  waitForLengthSim,
  waitForDataSim,
  waitForChksum1Sim,
  waitForChksum2Sim
} SimlinkRxState;

typedef enum
{
  PORT_IPC_CRTP,
  PORT_IPC_CONTROL
} SimlinkPort;


void simlinkInit();
bool simlinkTest();
bool isSimlinkUp();
int simlinkSendPacket(SimlinkPacket *slp);
struct crtpLinkOperations * simlinkGetLink();

#endif /* __SIMLINK_H__ */
