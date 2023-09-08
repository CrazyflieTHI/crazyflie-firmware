/**
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
 * motors.c - Motor driver
 *
 * This code mainly interfacing the PWM peripheral lib of ST.
 */
#define DEBUG_MODULE "MTR-DRV"

//FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>
#include <stdbool.h>

/* ST includes */
// #include "stm32f_sitl.h"

#include "motors.h"
#include "pm.h"
#include "debug.h"

//Logging includes
#include "log.h"
#include "param.h"
#include "crtpros.h"

static uint16_t motorsBLConvBitsTo16(uint16_t bits);
static uint16_t motorsBLConv16ToBits(uint16_t bits);
static uint16_t motorsConvBitsTo16(uint16_t bits);
static uint16_t motorsConv16ToBits(uint16_t bits);

static bool motorSetEnable = false;
static uint32_t motorPower[] = {0, 0, 0, 0};    // user-requested PWM signals
static uint16_t motorPowerSet[] = {0, 0, 0, 0}; // user-requested PWM signals (overrides)
static uint32_t motor_ratios[] = {0, 0, 0, 0};  // actual PWM signals

void motorsPlayTone(uint16_t frequency, uint16_t duration_msec);
void motorsPlayMelody(uint16_t *notes);
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

// #include "motors_def.c"

const MotorPerifDef** motorMap;  /* Current map configuration */

const uint32_t MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };

const uint16_t testsound[NBR_OF_MOTORS] = {A4, A5, F5, D5 };

bool motorPowerReady[4];
static uint32_t lastSentTime = 0;
static CRTPPacket p;

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} __attribute__((packed)) motorPowerPack;

const MotorHealthTestDef brushedMotorHealthTestSettings = {
  /* onPeriodMsec = */ 50,
  /* offPeriodMsec = */ 950,
  /* varianceMeasurementStartMsec = */ 0,
  /* onPeriodPWMRatio = */ 0xFFFF,
};

const MotorHealthTestDef brushlessMotorHealthTestSettings = {
  /* onPeriodMsec = */ 2000,
  /* offPeriodMsec = */ 1000,
  /* varianceMeasurementStartMsec = */ 1000,
  /* onPeriodPWMRatio = */ 0 /* user must set health.propTestPWMRatio explicitly */
};

const MotorHealthTestDef unknownMotorHealthTestSettings = {
  /* onPeriodMsec = */ 0,
  /* offPeriodMseec = */ 0,
  /* varianceMeasurementStartMsec = */ 0,
  /* onPeriodPWMRatio = */ 0
};

static bool isInit = false;

/* Private functions */

static uint16_t motorsBLConvBitsTo16(uint16_t bits)
{
  return (0xFFFF * (bits - MOTORS_BL_PWM_CNT_FOR_HIGH) / MOTORS_BL_PWM_CNT_FOR_HIGH);
}

static uint16_t motorsBLConv16ToBits(uint16_t bits)
{
  return (MOTORS_BL_PWM_CNT_FOR_HIGH + ((bits * MOTORS_BL_PWM_CNT_FOR_HIGH) / 0xFFFF));
}

static uint16_t motorsConvBitsTo16(uint16_t bits)
{
  return ((bits) << (16 - MOTORS_PWM_BITS));
}

static uint16_t motorsConv16ToBits(uint16_t bits)
{
  return ((bits) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

// We have data that maps PWM to thrust at different supply voltage levels.
// However, it is not the PWM that drives the motors but the voltage and
// amps (= power). With the PWM it is possible to simulate different
// voltage levels. The assumption is that the voltage used will be an
// procentage of the supply voltage, we assume that 50% PWM will result in
// 50% voltage.
//
//  Thrust (g)    Supply Voltage    PWM (%)     Voltage needed
//  0.0           4.01              0           0
//  1.6           3.98              6.25        0.24875
//  4.8           3.95              12.25       0.49375
//  7.9           3.82              18.75       0.735
//  10.9          3.88              25          0.97
//  13.9          3.84              31.25       1.2
//  17.3          3.80              37.5        1.425
//  21.0          3.76              43.25       1.6262
//  24.4          3.71              50          1.855
//  28.6          3.67              56.25       2.06438
//  32.8          3.65              62.5        2.28125
//  37.3          3.62              68.75       2.48875
//  41.7          3.56              75          2.67
//  46.0          3.48              81.25       2.8275
//  51.9          3.40              87.5        2.975
//  57.9          3.30              93.75       3.09375
//
// To get Voltage needed from wanted thrust we can get the quadratic
// polyfit coefficients using GNU octave:
//
// thrust = [0.0 1.6 4.8 7.9 10.9 13.9 17.3 21.0 ...
//           24.4 28.6 32.8 37.3 41.7 46.0 51.9 57.9]
//
// volts  = [0.0 0.24875 0.49375 0.735 0.97 1.2 1.425 1.6262 1.855 ...
//           2.064375 2.28125 2.48875 2.67 2.8275 2.975 3.09375]
//
// p = polyfit(thrust, volts, 2)
//
// => p = -0.00062390   0.08835522   0.06865956
//
// We will not use the contant term, since we want zero thrust to equal
// zero PWM.
//
// And to get the PWM as a percentage we would need to divide the
// Voltage needed with the Supply voltage.
float motorsCompensateBatteryVoltage(uint32_t id, float iThrust, float supplyVoltage)
{
  #ifdef CONFIG_ENABLE_THRUST_BAT_COMPENSATED_
  ASSERT(id < NBR_OF_MOTORS);

  if (motorMap[id]->drvType == BRUSHED)
  {
    /*
    * A LiPo battery is supposed to be 4.2V charged, 3.7V mid-charge and 3V
    * discharged.
    *
    * A suitable sanity check for disabling the voltage compensation would be
    * under 2V. That would suggest a damaged battery. This protects against
    * rushing the motors on bugs and invalid voltage levels.
    */
    if (supplyVoltage < 2.0f)
    {
      return iThrust;
    }

    float thrust = (iThrust / 65536.0f) * 60;
    float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
    float ratio = volts / supplyVoltage;
    return UINT16_MAX * ratio;
  }
  #endif

  return iThrust;
}

void resetPowerReady() {
  for(int i=0; i < NBR_OF_MOTORS; i++) {
    motorPowerReady[i] = false;
  }
}

/* Return true if all motor ratios have been set */
bool checkPowerReady() {
  for(int i=0; i < NBR_OF_MOTORS; i++) {
    if(!motorPowerReady[i])
      return false;
  }
  return true;
}

/* Public functions */

//Initialization. Will set all motors ratio to 0%
void motorsInit(const MotorPerifDef** motorMapSelect)
{
  if (isInit)
  {
    // First to init will configure it
    return;
  }

  resetPowerReady();
  lastSentTime = xTaskGetTickCount();
  p.size = 4 * sizeof(uint32_t);
  p.header = CRTP_HEADER(CRTP_PORT_SETPOINT_SIM, 0);

  isInit = true;

  // Output zero power
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}

void motorsDeInit(const MotorPerifDef** motorMapSelect){}

bool motorsTest(void)
{
  return isInit;
}

void motorsStop()
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}

// Ithrust is thrust mapped for 65536 <==> 60 grams
void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
  if (isInit) {
    uint16_t ratio = ithrust;

    ASSERT(id < NBR_OF_MOTORS);

    motorPower[id] = ithrust;

    motorPowerReady[id] = true;

    bool sendMotorPower = checkPowerReady();

    /* Send the motors ratio to ROS Gazebo */
    if(sendMotorPower) {
      motorPowerPack.m1 = motorPower[0];
      motorPowerPack.m2 = motorPower[1];
      motorPowerPack.m3 = motorPower[2];
      motorPowerPack.m4 = motorPower[3];

      if (xTaskGetTickCount() - lastSentTime >= M2T(2)) {
        memcpy(p.data , &motorPowerPack , p.size);
        crtprosSendPacket(&p);
        lastSentTime = xTaskGetTickCount();

        resetPowerReady();
      }
    }
  }
}

int motorsGetRatio(uint32_t id)
{
  ASSERT(id < NBR_OF_MOTORS);
  return motorPower[id];
}

void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio){}


// Play a tone with a given frequency and a specific duration in milliseconds (ms)
void motorsPlayTone(uint16_t frequency, uint16_t duration_msec){}

// Plays a melody from a note array
void motorsPlayMelody(uint16_t *notes){}

const MotorHealthTestDef* motorsGetHealthTestSettings(uint32_t id)
{
    return &unknownMotorHealthTestSettings;
}

/**
 * Override power distribution to motors.
 */
PARAM_GROUP_START(motorPowerSet)

/**
 * @brief Nonzero to override controller with set values
 */
PARAM_ADD_CORE(PARAM_UINT8, enable, &motorSetEnable)

/**
 * @brief motor power for m1: `0 - UINT16_MAX`
 */
PARAM_ADD_CORE(PARAM_UINT16, m1, &motorPowerSet[0])

/**
 * @brief motor power for m2: `0 - UINT16_MAX`
 */
PARAM_ADD_CORE(PARAM_UINT16, m2, &motorPowerSet[1])

/**
 * @brief motor power for m3: `0 - UINT16_MAX`
 */
PARAM_ADD_CORE(PARAM_UINT16, m3, &motorPowerSet[2])

/**
 * @brief motor power for m4: `0 - UINT16_MAX`
 */
PARAM_ADD_CORE(PARAM_UINT16, m4, &motorPowerSet[3])

PARAM_GROUP_STOP(motorPowerSet)

/**
 * Motor output related log variables.
 */
LOG_GROUP_START(motor)
/**
 * @brief Requested motor power (PWM value) for M1 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT32, m1, &motorPower[0])
/**
 * @brief Requested motor power (PWM value) for M2 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT32, m2, &motorPower[1])
/**
 * @brief Requested motor power (PWM value) for M3 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT32, m3, &motorPower[2])
/**
 * @brief Requested motor power (PWM value) for M4 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT32, m4, &motorPower[3])
LOG_GROUP_STOP(motor)


/**
 * Logging variables of the motors PWM output
 */
LOG_GROUP_START(pwm)
/**
 * @brief Current motor 1 PWM output
 */ 
LOG_ADD(LOG_UINT32, m1_pwm, &motor_ratios[0])
/**
 * @brief Current motor 2 PWM output
 */ 
LOG_ADD(LOG_UINT32, m2_pwm, &motor_ratios[1])
/**
 * @brief Current motor 3 PWM output
 */ 
LOG_ADD(LOG_UINT32, m3_pwm, &motor_ratios[2])
/**
 * @brief Current motor 4 PWM output
 */ 
LOG_ADD(LOG_UINT32, m4_pwm, &motor_ratios[3])
LOG_GROUP_STOP(pwm)
