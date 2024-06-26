/*
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
 * debug.h - Debugging utility functions
 */

#pragma once

#include "config.h"
#include "console.h"
#include "autoconf.h"

#ifdef CONFIG_DEBUG_PRINT_ON_UART1
  #include "uart1.h"
  #define uartPrintf uart1Printf
#endif

#ifdef DEBUG_PRINT_ON_SEGGER_RTT
  #include "SEGGER_RTT.h"
#endif

#ifdef DEBUG_MODULE
#define DEBUG_FMT(fmt) DEBUG_MODULE ": " fmt
#endif

#ifndef DEBUG_FMT
#define DEBUG_FMT(fmt) fmt
#endif

void debugInit(void);

#if defined(CONFIG_SITL_CF2)
  #include <stdio.h>
  #define DEBUG_PRINT(fmt, ...) printf(DEBUG_FMT(fmt), ##__VA_ARGS__)
  #define DEBUG_PRINT_OS(fmt, ...) printf(DEBUG_FMT(fmt), ##_VA_ARGS__)
#elif defined(UNIT_TEST_MODE)
  #include <stdio.h>
  #define DEBUG_PRINT(fmt, ...) printf(DEBUG_FMT(fmt), ##__VA_ARGS__)
  #define DEBUG_PRINT_OS(fmt, ...) printf(DEBUG_FMT(fmt), ##__VA_ARGS__)
#elif defined(CONFIG_DEBUG_PRINT_ON_UART1)
  #define DEBUG_PRINT(fmt, ...) uartPrintf(DEBUG_FMT(fmt), ##__VA_ARGS__)
  #define DEBUG_PRINT_OS(fmt, ...) uartPrintf(DEBUG_FMT(fmt), ##__VA_ARGS__)
#elif defined(DEBUG_PRINT_ON_SWO)
  #define DEBUG_PRINT(fmt, ...) eprintf(ITM_SendChar, fmt, ## __VA_ARGS__)
  #define DEBUG_PRINT_OS(fmt, ...) eprintf(ITM_SendChar, fmt, ## __VA_ARGS__)
#elif defined(DEBUG_PRINT_ON_SEGGER_RTT)
  #define DEBUG_PRINT(fmt, ...) SEGGER_RTT_printf(0, fmt, ## __VA_ARGS__)
  #define DEBUG_PRINT_OS(fmt, ...) SEGGER_RTT_printf(0, fmt, ## __VA_ARGS__)
#else // Debug using radio or USB
  #define DEBUG_PRINT(fmt, ...) consolePrintf(DEBUG_FMT(fmt), ##__VA_ARGS__)
  #define DEBUG_PRINT_OS(fmt, ...) consolePrintf(DEBUG_FMT(fmt), ##__VA_ARGS__)
  //#define DEBUG_PRINT(fmt, ...)
#endif

#ifndef PRINT_OS_DEBUG_INFO
  #undef DEBUG_PRINT_OS
  #define DEBUG_PRINT_OS(fmt, ...)
#endif


#ifdef TEST_PRINTS
  #define TEST_AND_PRINT(e, msgOK, msgFail)\
    if(e) { DEBUG_PRINT(msgOK); } else { DEBUG_PRINT(msgFail); }
  #define FAIL_PRINT(msg) DEBUG_PRINT(msg)
#else
  #define TEST_AND_PRINT(e, msgOK, msgFail)
  #define FAIL_PRINT(msg)
#endif
