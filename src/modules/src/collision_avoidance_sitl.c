/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2020 Bitcraze AB
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
 * collision_avoidance.c - Collision avoidance for multiple Crazyflies.
 *
 * Original author: James A. Preiss, University of Southern California, 2020.
 */

#include <float.h>
#include <string.h>  // for memset

#include "collision_avoidance.h"

#ifdef CRAZYFLIE_FW

#include "FreeRTOS.h"
#include "task.h"

#include "param.h"
#include "log.h"


static uint8_t collisionAvoidanceEnable = 0;

static collision_avoidance_params_t params = {
  .ellipsoidRadii = { .x = 0.3, .y = 0.3, .z = 0.9 },
  .bboxMin = { .x = -FLT_MAX, .y = -FLT_MAX, .z = -FLT_MAX },
  .bboxMax = { .x = FLT_MAX, .y = FLT_MAX, .z = FLT_MAX },
  .horizonSecs = 1.0f,
  .maxSpeed = 0.5f,  // Fairly conservative.
  .sidestepThreshold = 0.25f,
  .maxPeerLocAgeMillis = 5000,  // Probably longer than desired in most applications.
  .voronoiProjectionTolerance = 1e-5,
  .voronoiProjectionMaxIters = 100,
};

static collision_avoidance_state_t collisionState = {
  .lastFeasibleSetPosition = { .x = NAN, .y = NAN, .z = NAN },
};

void collisionAvoidanceInit()
{
}

bool collisionAvoidanceTest()
{
  return true;
}

// Latency counter for logging.
static uint32_t latency = 0;

void collisionAvoidanceUpdateSetpoint(
  setpoint_t *setpoint, sensorData_t const *sensorData, state_t const *state, uint32_t tick)
{}

LOG_GROUP_START(colAv)
  LOG_ADD(LOG_UINT32, latency, &latency)
LOG_GROUP_STOP(colAv)


/**
 * Onboard collision avoidance algorithm.
 *
 * Buffered Voronoi collision avoidance (BVCA) is a reactive multi-robot
 * collision avoidance method [1]. It is suitable for scenarios with low to
 * medium spatial contention -- i.e., when the robots generally do not
 * interfere with each other, but might occasionally cross paths.
 *
 * We obtain the positions of neighbors **on the same radio channel** from the
 * peer_localization.h module. We then compute our Voronoi cell: the set of 3D
 * points that are closer to us than to any neighbor. We shrink (buffer) the
 * cell to account for the Crazyflie's size. We are free to move within our
 * buffered Voronoi cell, but cannot leave it.
 *
 * BVCA acts by modifying the setpoints sent from the commander to the
 * controller. The new setpoint will be as close as possible to the original
 * while respecting the buffered Voronoi cell constraint. Our motion within the
 * cell also depends on a planning horizon (longer horizon will lead to more
 * conservative behavior) and a maximum speed. The commander and controller do
 * not need to know if BVCA is enabled. 
 *
 * BVCA does not attempt to smooth the modified setpoints, so the output may be
 * discontinuous or far from the current robot state. The controller must be
 * able to handle this kind of input. Currently, **only the PID controller** is
 * confirmed to work with BVCA. High-gain controllers like Mellinger may become
 * unstable.
 *
 * The volume for collision checking is a tall ellipsoid. This accounts for the
 * downwash effect: Due to the fast-moving stream of air produced by the
 * rotors, the safe distance to pass underneath another rotorcraft is much
 * further than the safe distance to pass to the side. The radii of the
 * ellipsoid can be set by using the parameters below.
 *
 * A bounding box may be specified, for example when using a motion capture
 * system. The box is applied to the Crazyflie's center point only; the
 * ellipsoid collision volume is ignored. The box can be set to +/- infinity if
 * the flight space is unbounded.
 *
 * [1] Zhou, Dingjiang, et al. "Fast, on-line collision avoidance for dynamic
 * vehicles using buffered voronoi cells." IEEE Robotics and Automation
 * Letters 2.2 (2017): 1047-1054.
 */

#endif  // CRAZYFLIE_FW
