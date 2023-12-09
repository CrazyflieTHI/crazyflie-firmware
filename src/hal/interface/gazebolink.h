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

#ifndef __GAZEBOLINK_H__
#define __GAZEBOLINK_H__

#include <stdbool.h>
#include "crtp.h"

#define GAZEBOLINK_PAYLOAD_SIZE 32

typedef struct _GazebolinkPacket
{
  uint8_t data[GAZEBOLINK_PAYLOAD_SIZE];
} __attribute__((packed)) GazebolinkPacket;

typedef enum
{
  waitForFirstStartGazebo,
  waitForSecondStartGazebo,
  waitForTypeGazebo,
  waitForLengthGazebo,
  waitForDataGazebo,
  waitForChksum1Gazebo,
  waitForChksum2Gazebo
} GazebolinkRxState;

void gazebolinkInit();
bool gazebolinkTest();
bool isGazebolinkUp();
int gazebolinkSendPacket(GazebolinkPacket *slp);
struct crtprosLinkOperations * gazebolinkGetLink();

#endif /* __GAZEBOLINK_H__ */
