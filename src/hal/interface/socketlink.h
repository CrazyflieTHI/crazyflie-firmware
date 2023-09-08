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
 * socketlink.c: Socket implementation of the CRTP link
 */

#ifndef __SOCKETLINK_H__
#define __SOCKETLINK_H__

#include <stdbool.h>
#include <stdint.h>
#include "crtp.h"

/* Initialize the socket link */
void socketlinkInit();

/* Return true if the init was successful */
bool socketlinkTest();

void setCrtpPort(uint16_t port);
uint16_t getCrtpPort();

void setAddressHost(char* addressHost);
void getAddressHost(char** addressHost);

void setCfId(uint8_t id);
uint8_t getCfId();

/* Get the socket link (Only link used in SITL) */
struct crtprosLinkOperations * socketlinkGetLink();

#endif
