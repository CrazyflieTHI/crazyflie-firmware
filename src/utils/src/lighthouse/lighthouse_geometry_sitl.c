/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Code addapted from ashtuchkin's Vive DIY position sensor
 * see https://github.com/ashtuchkin/vive-diy-position-sensor/
 *
 * Copyright (c) 2016 Alexander Shtuchkin
 * Copyright (C) 2018 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * lighthouseGeometry.c: lighthouse tracking system geometry functions
 */

#include "lighthouse_geometry.h"

static void vec_cross_product(const vec3d a, const vec3d b, vec3d res) {
    res[0] = a[1]*b[2] - a[2]*b[1];
    res[1] = a[2]*b[0] - a[0]*b[2];
    res[2] = a[0]*b[1] - a[1]*b[0];
}

static float vec_dot(const vec3d a, const vec3d b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void vec_add(const vec3d a, const vec3d b, vec3d r) {
  r[0] = a[0] + b[0];
  r[1] = a[1] + b[1];
  r[2] = a[2] + b[2];
}

static float vec_length(const vec3d vec) {
    return 0.0;
}

static bool intersect_lines(vec3d orig1, vec3d vec1, vec3d orig2, vec3d vec2, vec3d res, float *dist) {
    return false;
}

bool lighthouseGeometryGetPositionFromRayIntersection(const baseStationGeometry_t* geo1, const baseStationGeometry_t* geo2, float angles1[2], float angles2[2], vec3d position, float *position_delta)
{
    return false;
}

void lighthouseGeometryGetBaseStationPosition(const baseStationGeometry_t* bs, vec3d baseStationPos) {}

void lighthouseGeometryGetRay(const baseStationGeometry_t* baseStationGeometry, const float angleH, const float angleV, vec3d ray) {}

bool lighthouseGeometryIntersectionPlaneVector(const vec3d linePoint, const vec3d lineVec, const vec3d planePoint, const vec3d PlaneNormal, vec3d intersectionPoint) {
    return false;
}

void lighthouseGeometryGetSensorPosition(const vec3d cfPos, const arm_matrix_instance_f32 *R, vec3d sensorPosition, vec3d pos) {}

bool lighthouseGeometryYawDelta(const vec3d ipv, const vec3d spv, const vec3d n, float* yawDelta) {
    return false;
}
