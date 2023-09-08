/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018 Bitcraze AB
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
 */

#ifndef __ARM_MATH_SITL_H__
#define __ARM_MATH_SITL_H__

#include "FreeRTOSConfig.h"
#include "cfassert.h"
#include "math.h"

#include <string.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>

#define PI                            3.1415926f
#define arm_matrix_instance_f32       Matrixf
#define arm_sqrt(x)                   sqrtf(x)
#define arm_sqrt_f32(x)               sqrtf(x)
#define arm_cos_f32(x)                cosf(x)
#define arm_sin_f32(x)                sinf(x)

typedef struct{
  uint16_t numRows;
  uint16_t numCols;
  float *pData;
} Matrixf;

typedef float float32_t;

#define DEG_TO_RAD (PI/180.0f)
#define RAD_TO_DEG (180.0f/PI)

#define MIN(a, b) ((b) < (a) ? (b) : (a))
#define MAX(a, b) ((b) > (a) ? (b) : (a))

static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ 
  bool is_valid = (pSrc->numRows == pDst->numCols) && (pSrc->numCols == pDst->numRows);
  configASSERT(is_valid);
  uint8_t i,j;
  for (i=0 ; i< pSrc->numRows; i++){
    for(j=0 ; j< pSrc->numCols; j++){
      pDst->pData[j * pDst->numCols + i] = pSrc->pData[i*pSrc->numCols + j];
    }
  } 
}

static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ 
  bool is_valid = (pSrcA->numCols == pSrcB->numRows) && (pSrcA->numRows == pDst->numRows) && (pSrcB->numCols == pDst->numCols);
  configASSERT(is_valid);
  uint8_t i,j,k;
  for(i=0; i< pDst->numRows ; i++){
    for(j=0; j<pDst->numCols ; j++){
      pDst->pData[j+ i*pDst->numCols] = 0.0f;
    }
  }
  for(i=0; i< pDst->numRows ; i++){
    for(j=0; j<pDst->numCols ; j++){
      for(k=0; k<pSrcA->numCols; k++){
        pDst->pData[j+ i*pDst->numCols] += pSrcA->pData[i*pSrcA->numCols + k ] * pSrcB->pData[k*pSrcB->numCols + j];
      }
    }
  }
}

// Matrix data must be aligned on 4 byte bundaries
static inline void assert_aligned_4_bytes(const arm_matrix_instance_f32* matrix) {
  const uint32_t address = (uint32_t)matrix->pData;
  ASSERT((address & 0x3) == 0);
}

static inline float limPos(float in) {
  if (in < 0.0f) {
    return 0.0f;
  }

  return in;
}

static inline float clip1(float a) {
  if (a < -1.0f) {
    return -1.0f;
  }

  if (a > 1.0f) {
    return 1.0f;
  }

  return a;
}

/* TODO: Implementation of this function needs to be tested! */
static inline void mat_scale(const arm_matrix_instance_f32 * pSrcA, float32_t scale, arm_matrix_instance_f32 * pDst)
{
  // ASSERT(ARM_MATH_SUCCESS == arm_mat_scale_f32(pSrcA, scale, pDst));
  const size_t colsSrc = pSrcA->numCols;
  const size_t rowsSrc = pSrcA->numRows;
  const size_t colsDst = pDst->numCols;
  const size_t rowsDst = pDst->numRows;
  const double scaleFactor = (double)scale;

  double matValue;
  /* size1 member of gsl_matrix refers to rows
   * and size2 refers to columns */
  gsl_matrix* srcMat = gsl_matrix_alloc(rowsSrc, colsSrc);
  gsl_matrix* dstMat = gsl_matrix_alloc(rowsDst, colsDst);

  /* Fill the GSL source matrix */
  for(int i=0; i<rowsSrc ; i++){
    for(int j=0; j<colsSrc ; j++){
      matValue = (double)pSrcA->pData[j+ i*pSrcA->numCols];
      gsl_matrix_set(srcMat, i, j, matValue);
    }
  }

  gsl_matrix_scale(srcMat, scaleFactor);

  /* Fill the Matrixf destination matrix */
  for(int i=0; i<rowsDst ; i++){
    for(int j=0; j<colsDst ; j++){
      matValue = gsl_matrix_get(srcMat, i, j);
      pDst->pData[j+ i*pDst->numCols] = (float)matValue;
    }
  }

  gsl_matrix_free(dstMat);
  gsl_matrix_free(srcMat);
}

/* TODO: Implementation of this function needs to be tested! */
static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst) {
  // assert_aligned_4_bytes(pSrc);
  // assert_aligned_4_bytes(pDst);

  // ASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst));
  const size_t colsSrc = pSrc->numCols;
  const size_t rowsSrc = pSrc->numRows;
  const size_t colsDst = pDst->numCols;
  const size_t rowsDst = pDst->numRows;
  size_t size = rowsSrc * colsDst;

  int signum;
  double matValue;
  /* size1 member of gsl_matrix refers to rows
   * and size2 refers to columns */
  gsl_matrix* srcMat = gsl_matrix_alloc(rowsSrc, colsSrc);
  gsl_matrix* dstMat = gsl_matrix_alloc(rowsDst, colsDst);

  /* Fill the GSL source matrix */
  for(int i=0; i<rowsSrc ; i++){
    for(int j=0; j<colsSrc ; j++){
      matValue = (double)pSrc->pData[j+ i*pSrc->numCols];
      gsl_matrix_set(srcMat, i, j, matValue);
    }
  }

  gsl_permutation *p = gsl_permutation_alloc(rowsDst);

  // Compute the LU decomposition of this matrix
  gsl_linalg_LU_decomp(srcMat, p, &signum);

  // Compute the  inverse of the LU decomposition
  gsl_matrix *inv = gsl_matrix_alloc(rowsDst, colsDst);
  gsl_linalg_LU_invert(srcMat, p, inv);

  gsl_permutation_free(p);

  /* Fill the Matrixf destination matrix */
  for(int i=0; i<rowsDst ; i++){
    for(int j=0; j<colsDst ; j++){
      matValue = gsl_matrix_get(inv, i, j);
      pDst->pData[j+ i*pDst->numCols] = (float)matValue;
    }
  }

  gsl_matrix_free(dstMat);
  gsl_matrix_free(srcMat);
}

// copy float matrix
static inline void matrixcopy(int ROW, int COLUMN, float destmat[ROW][COLUMN], float srcmat[ROW][COLUMN]){
    //TODO: check the dimension of the matrices
    for (int i=0; i<ROW; i++){
        for(int j=0; j<COLUMN; j++){
            destmat[i][j] = srcmat[i][j];
        }
    }
}

// copy float vector
static inline void vectorcopy(int DIM, float destVec[DIM], float srcVec[DIM]){
    //TODO: check the dimension of the vector
    for (int i=0; i<DIM; i++){
        destVec[i] = srcVec[i];
    }
}

#endif // __ARM_MATH_SITL_H__
