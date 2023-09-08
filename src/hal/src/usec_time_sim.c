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
 *					   Cosynux , LIX , France
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
 * usec_time.c - fake microsecond-resolution timer and timestamps on LINUX RTOS
 */

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#include "usec_time.h"

void initUsecTimer(void)
{
  return ;
}

/* usecTimestamp just to be able to used crtp_commander_high_level module */
uint64_t usecTimestamp(void)
{
  return (unsigned long) ((xTaskGetTickCount())*(1000000.0/configTICK_RATE_HZ)) ;
}
