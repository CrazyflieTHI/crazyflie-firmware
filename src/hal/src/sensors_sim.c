/**
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
 * Source file for Sensors simulation using external source of measurement
 * This is based on the work made in sensors_cf2.c 
 */

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <string.h>
#include "sensors_sim.h"

#include "system.h"
#include "debug.h"

#ifndef CONFIG_SITL_CF2
/* Add a simulate driver for Zrange*/
// #include "zranger.h"
// #include "zranger2.h"
#include "ledseq.h"
#include "sound.h"
#endif

#include "configblock.h"
#include "filter.h"
#include "param.h"
#include "log.h"

#include "crtp.h"
#include "crtpros.h"

#include <math.h>

#include "estimator.h"
#include "range.h"

// We try to be as close as possible from the real sensor implementation
#define MAG_GAUSS_PER_LSB                                 666.7f
#define SENSORS_DEG_PER_LSB_CFG                           (float)((2 * 2000.0) / 65536.0)
#define SENSORS_G_PER_LSB_CFG                             (float)((2 * 16) / 65536.0)

#define SENSORS_BIAS_SAMPLES                              2000
#define SENSORS_ACC_SCALE_SAMPLES                         1000

#define SENSORS_GYRO_BIAS_CALCULATE_STDDEV

#define GYRO_NBR_OF_AXES                                  3
#define GYRO_MIN_BIAS_TIMEOUT_MS                          M2T(1*1000)

// Number of samples used in variance calculation. Changing this effects the threshold
#define SENSORS_NBR_OF_BIAS_SAMPLES                       1024

// Variance threshold to take zero bias for gyro
#define GYRO_VARIANCE_BASE                                5000
#define GYRO_VARIANCE_THRESHOLD_X                         (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y                         (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z                         (GYRO_VARIANCE_BASE)

// Sensor type (first byte of crtp packet)
enum SensorTypeSim_e {
  SENSOR_GYRO_ACC_SIM           = 0,
  SENSOR_MAG_SIM                = 1,
  SENSOR_BARO_SIM               = 2,
  SENSOR_POS_SIM                = 3,
  SENSOR_MR_SIM                 = 4,
};

typedef struct
{
  Axis3f     bias;
  bool       isBiasValueFound;
  bool       isBufferFilled;
  Axis3i16*  bufHead;
  Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

static xQueueHandle accelerometerDataQueue;
static xQueueHandle gyroDataQueue;
static xQueueHandle magnetometerDataQueue;
static xQueueHandle barometerDataQueue;
static xSemaphoreHandle dataReady;

static bool isInit = false;
static sensorData_t sensors;

measurement_t measurement;

static BiasObj gyroBiasRunning;
static Axis3f  gyroBias;
#if defined(SENSORS_GYRO_BIAS_CALCULATE_STDDEV) && defined (GYRO_BIAS_LIGHT_WEIGHT)
static Axis3f  gyroBiasStdDev;
#endif
static bool    gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;

// Low Pass filtering <---> Not sure if needed when doing simulation
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in);

static bool isBarometerPresent = false;
static bool isMagnetometerPresent = false;

static bool isMpu6500TestPassed = false;
static bool isAK8963TestPassed = false;
static bool isLPS25HTestPassed = false;

// Pre-calculated values for accelerometer alignment
float cosPitch;
float sinPitch;
float cosRoll;
float sinRoll;

static void processAccGyroMeasurements(const uint8_t *buffer);
static void processMagnetometerMeasurements(const uint8_t *buffer);
static void processBarometerMeasurements(const uint8_t *buffer);
void processMultirangerMeasurements(const uint8_t* buffer);

#ifdef GYRO_GYRO_BIAS_LIGHT_WEIGHT
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
#else
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz,  Axis3f *gyroBiasOut);
#endif
static bool processAccScale(int16_t ax, int16_t ay, int16_t az);
static void sensorsBiasObjInit(BiasObj* bias);
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut);
/*static void sensorsCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut);*/
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj* bias);
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out);

// static void sensorsDeviceInit(void);
// static void sensorsTaskInit(void);
// static void sensorsTask(void *param);

bool sensorsSimReadGyro(Axis3f *gyro)
{
  return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsSimReadAcc(Axis3f *acc)
{
  return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

bool sensorsSimReadMag(Axis3f *mag)
{
  return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

bool sensorsSimReadBaro(baro_t *baro)
{
  return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

void sensorsSimAcquire(sensorData_t *sensors, const uint32_t tick)
{
  sensorsReadGyro(&sensors->gyro);
  sensorsReadAcc(&sensors->acc);
  sensorsReadMag(&sensors->mag);
  sensorsReadBaro(&sensors->baro);
}

bool sensorsSimAreCalibrated() {
  return gyroBiasFound;
}

static void sensorsTask(void *param)
{
  systemWaitStart();

  CRTPPacket p;
  crtprosInitTaskQueue(CRTP_PORT_SETPOINT_SIM);
  bool lastGyroBiasFound = false;

  while (1)
  {
    // Wait for a new sensor message to arrive
    crtprosReceivePacketBlock(CRTP_PORT_SETPOINT_SIM, &p);
    switch (p.data[0])
    {
      case SENSOR_GYRO_ACC_SIM:
        processAccGyroMeasurements(&(p.data[1]));
        xQueueOverwrite(accelerometerDataQueue, &sensors.acc);
        xQueueOverwrite(gyroDataQueue, &sensors.gyro);
        measurement.type = MeasurementTypeAcceleration;
        measurement.data.acceleration.acc = sensors.acc;
        estimatorEnqueue(&measurement);
        measurement.type = MeasurementTypeGyroscope;
        measurement.data.gyroscope.gyro = sensors.gyro;
        estimatorEnqueue(&measurement);
        break;
      case SENSOR_MAG_SIM:
        processMagnetometerMeasurements(&(p.data[1]));
        xQueueOverwrite(magnetometerDataQueue, &sensors.mag);
        break;
      case SENSOR_BARO_SIM:
        processBarometerMeasurements(&(p.data[1]));
        xQueueOverwrite(barometerDataQueue, &sensors.baro);
        measurement.type = MeasurementTypeBarometer;
        measurement.data.barometer.baro = sensors.baro;
        estimatorEnqueue(&measurement);
        break;
      case SENSOR_MR_SIM:
        processMultirangerMeasurements(&(p.data[1]));
        break;
      default :
        break;
    }

    // Answer while gyro bias not found
    if (!lastGyroBiasFound && gyroBiasFound){
      p.data[0] = gyroBiasFound;
      p.size = 1;
      lastGyroBiasFound =  gyroBiasFound;
      crtprosSendPacket(&p);
    }
    xSemaphoreGive(dataReady);
  }
}

void sensorsSimWaitDataReady(void)
{
  xSemaphoreTake(dataReady, portMAX_DELAY);
}

void processBarometerMeasurements(const uint8_t *buffer)
{
  const baro_t *baroData = (baro_t *) buffer;

  sensors.baro.pressure = baroData->pressure;
  sensors.baro.temperature = baroData->temperature;
  sensors.baro.asl = baroData->asl;
}

void processMagnetometerMeasurements(const uint8_t *buffer)
{
  // (real value in G / SENSORS_G_PER_LSB_CFG)
  const Axis3i16 *magData = (Axis3i16 *) buffer;

  sensors.mag.x = (float)magData->x / MAG_GAUSS_PER_LSB;
  sensors.mag.y = (float)magData->y / MAG_GAUSS_PER_LSB;
  sensors.mag.z = (float)magData->z / MAG_GAUSS_PER_LSB;
}

void processAccGyroMeasurements(const uint8_t *buffer)
{
  Axis3f accScaled;

  // (real value in G / SENSORS_G_PER_LSB_CFG) from gazebo SIM
  const Axis3i16 *accData = (Axis3i16 *) buffer;
  //(real value in deg / SENSORS_DEG_PER_LSB_CFG) from gazebo SIM
  const Axis3i16 *gyroData = (Axis3i16 *) (buffer + sizeof(Axis3i16));


#ifdef GYRO_BIAS_LIGHT_WEIGHT
  gyroBiasFound = processGyroBiasNoBuffer(gyroData->x, gyroData->y, gyroData->z, &gyroBias);
#else
  gyroBiasFound = processGyroBias(gyroData->x, gyroData->y, gyroData->z, &gyroBias);
#endif
  if (gyroBiasFound)
  {
     processAccScale(accData->x, accData->y, accData->z);
  }

  sensors.gyro.x =  (gyroData->x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
  sensors.gyro.y =  (gyroData->y - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
  sensors.gyro.z =  (gyroData->z - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
  applyAxis3fLpf((lpf2pData*)(&gyroLpf), &sensors.gyro);

  accScaled.x =  (accData->x) * SENSORS_G_PER_LSB_CFG / accScale;
  accScaled.y =  (accData->y) * SENSORS_G_PER_LSB_CFG / accScale;
  accScaled.z =  (accData->z) * SENSORS_G_PER_LSB_CFG / accScale;
  sensorsAccAlignToGravity(&accScaled, &sensors.acc);
  applyAxis3fLpf((lpf2pData*)(&accLpf), &sensors.acc);
}

void processMultirangerMeasurements(const uint8_t* buffer)
{
  float sensorData[4];
  memcpy(sensorData, buffer, sizeof(float)*4);

  rangeSet(rangeFront, sensorData[0]);
  rangeSet(rangeBack, sensorData[1]);
  rangeSet(rangeLeft, sensorData[2]);
  rangeSet(rangeRight, sensorData[3]);
}

static void sensorsDeviceInit(void)
{
  isMagnetometerPresent = true;
  isBarometerPresent = true;

  DEBUG_PRINT("AK8963 I2C connection [OK].\n");
  DEBUG_PRINT("LPS25H I2C connection [OK].\n");

  cosPitch = cosf(configblockGetCalibPitch() * (float) M_PI/180);
  sinPitch = sinf(configblockGetCalibPitch() * (float) M_PI/180);
  cosRoll = cosf(configblockGetCalibRoll() * (float) M_PI/180);
  sinRoll = sinf(configblockGetCalibRoll() * (float) M_PI/180);

  // Init second order filer for accelerometer
  for (uint8_t i = 0; i < 3; i++)
  {
    lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
    lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
  }
}

static void sensorsTaskInit(void)
{
  // Use to have access to the sensors data
  dataReady = xSemaphoreCreateBinary();

  accelerometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
  gyroDataQueue = xQueueCreate(1, sizeof(Axis3f));
  magnetometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
  barometerDataQueue = xQueueCreate(1, sizeof(baro_t));

  xTaskCreate(sensorsTask, SENSORS_TASK_NAME, SENSORS_TASK_STACKSIZE, NULL, SENSORS_TASK_PRI, NULL);
}


void sensorsSimInit(void)
{
  if (isInit)
  {
    return;
  }

  sensorsBiasObjInit(&gyroBiasRunning);
  sensorsDeviceInit();
  sensorsTaskInit();

  isInit = true;
}

bool sensorsSimTest(void)
{
  bool testStatus = true;

  if (!isInit)
  {
    DEBUG_PRINT("Error while initializing sensor task\r\n");
    testStatus = false;
  }else {
    isMpu6500TestPassed = true;
    isLPS25HTestPassed = true;
    isAK8963TestPassed = true;
    testStatus= true;
    DEBUG_PRINT("SENSORS TEST PASSED [OK].\n");
  }

  return testStatus;
}

/**
 * Calculates accelerometer scale out of SENSORS_ACC_SCALE_SAMPLES samples. Should be called when
 * platform is stable.
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
  static bool accBiasFound = false;
  static uint32_t accScaleSumCount = 0;

  if (!accBiasFound)
  {
    accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
    accScaleSumCount++;

    if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
    {
      accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
      accBiasFound = true;
    }
  }

  return accBiasFound;
}

#ifdef GYRO_BIAS_LIGHT_WEIGHT
/**
 * Calculates the bias out of the first SENSORS_BIAS_SAMPLES gathered. Requires no buffer
 * but needs platform to be stable during startup.
 */
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
  static uint32_t gyroBiasSampleCount = 0;
  static bool gyroBiasNoBuffFound = false;
  static Axis3i64 gyroBiasSampleSum;
  static Axis3i64 gyroBiasSampleSumSquares;

  if (!gyroBiasNoBuffFound)
  {
    // If the gyro has not yet been calibrated:
    // Add the current sample to the running mean and variance
    gyroBiasSampleSum.x += gx;
    gyroBiasSampleSum.y += gy;
    gyroBiasSampleSum.z += gz;
#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
    gyroBiasSampleSumSquares.x += gx * gx;
    gyroBiasSampleSumSquares.y += gy * gy;
    gyroBiasSampleSumSquares.z += gz * gz;
#endif
    gyroBiasSampleCount += 1;

    // If we then have enough samples, calculate the mean and standard deviation
    if (gyroBiasSampleCount == SENSORS_BIAS_SAMPLES)
    {
      gyroBiasOut->x = (float)(gyroBiasSampleSum.x) / SENSORS_BIAS_SAMPLES;
      gyroBiasOut->y = (float)(gyroBiasSampleSum.y) / SENSORS_BIAS_SAMPLES;
      gyroBiasOut->z = (float)(gyroBiasSampleSum.z) / SENSORS_BIAS_SAMPLES;

#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
      gyroBiasStdDev.x = sqrtf((float)(gyroBiasSampleSumSquares.x) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->x * gyroBiasOut->x));
      gyroBiasStdDev.y = sqrtf((float)(gyroBiasSampleSumSquares.y) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->y * gyroBiasOut->y));
      gyroBiasStdDev.z = sqrtf((float)(gyroBiasSampleSumSquares.z) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->z * gyroBiasOut->z));
#endif
      gyroBiasNoBuffFound = true;
    }
  }

  return gyroBiasNoBuffFound;
}
#else
/**
 * Calculates the bias first when the gyro variance is below threshold. Requires a buffer
 * but calibrates platform first when it is stable.
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
  sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

  if (!gyroBiasRunning.isBiasValueFound)
  {
    sensorsFindBiasValue(&gyroBiasRunning);
    if (gyroBiasRunning.isBiasValueFound)
    {
#ifndef CONFIG_SITL_CF2
      soundSetEffect(SND_CALIB);
      ledseqRun(SYS_LED, seq_calibrated);
#endif
      DEBUG_PRINT("GYro Bias found \n");
    }
  }

  gyroBiasOut->x = gyroBiasRunning.bias.x;
  gyroBiasOut->y = gyroBiasRunning.bias.y;
  gyroBiasOut->z = gyroBiasRunning.bias.z;

  return gyroBiasRunning.isBiasValueFound;
}
#endif

static void sensorsBiasObjInit(BiasObj* bias)
{
  bias->isBufferFilled = false;
  bias->bufHead = bias->buffer;
}


/**
 * Calculates the variance and mean for the bias buffer.
 */
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut)
{
  uint32_t i;
  int64_t sum[GYRO_NBR_OF_AXES] = {0};
  int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }

  varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

  meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * Calculates the mean for the bias buffer.
 */
/*static void __attribute__((used)) sensorsCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut)
{
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
  }

  meanOut->x = sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}*/

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z)
{
  bias->bufHead->x = x;
  bias->bufHead->y = y;
  bias->bufHead->z = z;
  bias->bufHead++;

  if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
  {
    bias->bufHead = bias->buffer;
    bias->isBufferFilled = true;
  }
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
static bool sensorsFindBiasValue(BiasObj* bias)
{
  static int32_t varianceSampleTime;
  bool foundBias = false;

  if (bias->isBufferFilled)
  {
    Axis3f variance;
    Axis3f mean;

    sensorsCalculateVarianceAndMean(bias, &variance, &mean);

    /*if (variance.x < GYRO_VARIANCE_THRESHOLD_X &&
        variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
        variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount()))
    {*/
    if (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount())
    {
      varianceSampleTime = xTaskGetTickCount();
      bias->bias.x = mean.x;
      bias->bias.y = mean.y;
      bias->bias.z = mean.z;
      foundBias = true;
      bias->isBiasValueFound = true;
    }
  }

  return foundBias;
}

bool sensorsSimManufacturingTest(void)
{
  return true;
}

/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data gathered from the UI and written in the config-block to
 * rotate the accelerometer to be aligned with gravity.
 */
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out)
{
  Axis3f rx;
  Axis3f ry;

  // Rotate around x-axis
  rx.x = in->x;
  rx.y = in->y * cosRoll - in->z * sinRoll;
  rx.z = in->y * sinRoll + in->z * cosRoll;

  // Rotate around y-axis
  ry.x = rx.x * cosPitch - rx.z * sinPitch;
  ry.y = rx.y;
  ry.z = -rx.x * sinPitch + rx.z * cosPitch;

  out->x = ry.x;
  out->y = ry.y;
  out->z = ry.z;
}

void sensorsSimSetAccMode(accModes accMode)
{
  switch (accMode)
  {
    case ACC_MODE_PROPTEST:
      // mpu6500SetAccelDLPF(MPU6500_ACCEL_DLPF_BW_460);
      for (uint8_t i = 0; i < 3; i++)
      {
        lpf2pInit(&accLpf[i],  1000, 500);
      }
      break;
    case ACC_MODE_FLIGHT:
    default:
      // mpu6500SetAccelDLPF(MPU6500_ACCEL_DLPF_BW_41);
      for (uint8_t i = 0; i < 3; i++)
      {
        lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
      }
      break;
  }
}

static void applyAxis3fLpf(lpf2pData *data, Axis3f* in)
{
  for (uint8_t i = 0; i < 3; i++) {
    in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
  }
}

#ifdef GYRO_ADD_RAW_AND_VARIANCE_LOG_VALUES
LOG_GROUP_START(gyro)
LOG_ADD(LOG_INT16, xRaw, &gyroRaw.x)
LOG_ADD(LOG_INT16, yRaw, &gyroRaw.y)
LOG_ADD(LOG_INT16, zRaw, &gyroRaw.z)
LOG_ADD(LOG_FLOAT, xVariance, &gyroBiasRunning.variance.x)
LOG_ADD(LOG_FLOAT, yVariance, &gyroBiasRunning.variance.y)
LOG_ADD(LOG_FLOAT, zVariance, &gyroBiasRunning.variance.z)
LOG_GROUP_STOP(gyro)
#endif

PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, HMC5883L_SIM, &isMagnetometerPresent)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611_SIM, &isBarometerPresent) // TODO: Rename MS5611 to LPS25H. Client needs to be updated at the same time.
PARAM_GROUP_STOP(imu_sensors)

PARAM_GROUP_START(imu_tests)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MPU6500_SIM, &isMpu6500TestPassed)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, HMC5883L_SIM, &isAK8963TestPassed)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611_SIM, &isLPS25HTestPassed) // TODO: Rename MS5611 to LPS25H. Client needs to be updated at the same time.
PARAM_GROUP_STOP(imu_tests)
