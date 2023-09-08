/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2018 Bitcraze AB
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
 * Generic platform functionality
 *
 */

#include <string.h>
#include "platform.h"
#include "motors.h"
#include "debug.h"

static const MotorPerifDef simDummy1;
static const MotorPerifDef simDummy2;
static const MotorPerifDef simDummy3;
static const MotorPerifDef simDummy4;

static const platformConfig_t* active_config = 0;

const MotorPerifDef* simMotorMap[NBR_OF_MOTORS] = {
  &simDummy1,
  &simDummy2,
  &simDummy3,
  &simDummy4
};

static const platformConfig_t sitlConfig = {
    .deviceType = "Sim",
    .deviceTypeName = "Simulation",
    .sensorImplementation = SensorImplementation_sitl,
    .physicalLayoutAntennasAreClose = false,
    .motorMap = simMotorMap
};

int platformInit(void) {
  active_config = &sitlConfig;
  return 0;
}

// Mock implementation
static char* deviceTypeStringToReturn = "";
void platformGetDeviceTypeString(char* deviceTypeString) {
  strcpy(deviceTypeString, deviceTypeStringToReturn);
}

int platformParseDeviceTypeString(const char* deviceTypeString, char* deviceType) {
  if (deviceTypeString[0] != '0' || deviceTypeString[1] != ';') {
    return 1;
  }

  const int start = 2;
  const int last = start + PLATFORM_DEVICE_TYPE_MAX_LEN - 1;
  int end = 0;
  for (end = start; end <= last; end++) {
    if (deviceTypeString[end] == '\0' || deviceTypeString[end] == ';') {
      break;
    }
  }

  if (end > last) {
    return 1;
  }

  int length = end - start;
  memcpy(deviceType, &deviceTypeString[start], length);
  deviceType[length] = '\0';
  return 0;
}

int platformInitConfiguration(const platformConfig_t* configs, const int nrOfConfigs) {
  return 0;
}

const char* platformConfigGetDeviceType() {
  return active_config->deviceType;
}

const char* platformConfigGetDeviceTypeName() {
  return active_config->deviceTypeName;
}

SensorImplementation_t platformConfigGetSensorImplementation() {
  return active_config->sensorImplementation;
}

bool platformConfigPhysicalLayoutAntennasAreClose() {
  return active_config->physicalLayoutAntennasAreClose;
}

const MotorPerifDef** platformConfigGetMotorMapping() {
  return active_config->motorMap;
}
