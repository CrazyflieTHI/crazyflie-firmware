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
 * comm_sitl.c - High level communication module for SITL based on comm.c
 * Some modules has been deleted in order to remove low level dependencies 
 */
#define DEBUG_MODULE "COMM"
#include <stdbool.h>

#include "config.h"
#include "comm.h"
#include "crtp.h"
#include "crtpros.h"
#include "console.h"
#include "crtpservice.h"
#include "param_task.h"
#include "log.h"
#include "gazebolink.h"
#include "simlink.h"
#include "platformservice.h"
#include "crtp_localization_service.h"
#include "debug.h"

static bool isInit;

void commInit(void)
{
  if (isInit)
    return;

  /* These functions are moved to be initialized early so
   * that DEBUG_PRINT can be used early */
  // crtpInit();
  // consoleInit();

  crtprosSetLink(gazebolinkGetLink());
  crtpSetLink(simlinkGetLink());
  DEBUG_PRINT("CRTP link set to socket link \n");

  crtpserviceInit();

  platformserviceInit();

  logInit();
  DEBUG_PRINT("LOG service init finished \n");

  paramInit();
  DEBUG_PRINT("PARAM service init finished \n");

  locSrvInit();
  DEBUG_PRINT("COMM init succeed \n");
  
  isInit = true;
}

bool commTest(void)
{
  bool pass=isInit;
  
  pass &= crtpTest();
  pass &= crtpserviceTest();
  pass &= platformserviceTest();
  pass &= consoleTest();
  pass &= paramTest();
  
  return pass;
}

