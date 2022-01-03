/**
 ******************************************************************************
 * @file    vl53l4cd_calibration.cpp
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    29 November 2021
 * @brief   Implementation of the VL53L4CD APIs for calibration.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2021 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include "vl53l4cd_class.h"

VL53L4CD_ERROR VL53L4CD::VL53L4CD_CalibrateOffset(
  int16_t TargetDistInMm,
  int16_t *p_measured_offset_mm,
  int16_t nb_samples)
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
  uint8_t i, tmp, continue_loop;
  uint16_t j, tmpOff;
  int16_t AvgDistance = 0;
  VL53L4CD_Result_t results;

  if (((nb_samples < (int16_t)5) || (nb_samples > (int16_t)255))
      || ((TargetDistInMm < (int16_t)50)
          || (TargetDistInMm > (int16_t)1000))) {
    status |= (uint8_t)VL53L4CD_ERROR_INVALID_ARGUMENT;
  } else {
    status |= VL53L4CD_WrWord(dev, VL53L4CD_RANGE_OFFSET_MM, 0x0);
    status |= VL53L4CD_WrWord(dev, VL53L4CD_INNER_OFFSET_MM, 0x0);
    status |= VL53L4CD_WrWord(dev, VL53L4CD_OUTER_OFFSET_MM, 0x0);

    /* Device heat loop (10 samples) */
    status |= VL53L4CD_StartRanging();
    for (i = 0; i < (uint8_t)10; i++) {
      tmp = (uint8_t)0;
      j = (uint16_t)0;
      continue_loop = (uint8_t)1;
      do {
        status |= VL53L4CD_CheckForDataReady(&tmp);
        if (tmp == (uint8_t)1) { /* Data ready */
          continue_loop = (uint8_t)0;
        } else if (j < (uint16_t)5000) { /* Wait for answer*/
          j++;
        } else { /* Timeout 5000ms reached */
          continue_loop = (uint8_t)0;
          status |= (uint8_t)VL53L4CD_ERROR_TIMEOUT;
        }
        WaitMs(1);
      } while (continue_loop == (uint8_t)1);
      status |= VL53L4CD_GetResult(&results);
      status |= VL53L4CD_ClearInterrupt();
    }
    status |= VL53L4CD_StopRanging();

    /* Device ranging */
    status |= VL53L4CD_StartRanging();
    for (i = 0; i < (uint8_t)nb_samples; i++) {
      tmp = (uint8_t)0;
      j = (uint16_t)0;
      continue_loop = (uint8_t)1;
      do {
        status |= VL53L4CD_CheckForDataReady(&tmp);
        if (tmp == (uint8_t)1) { /* Data ready */
          continue_loop = (uint8_t)0;
        } else if (j < (uint16_t)5000) { /* Wait for answer*/
          j++;
        } else { /* Timeout 5000ms reached */
          continue_loop = (uint8_t)0;
          status |= (uint8_t)VL53L4CD_ERROR_TIMEOUT;
        }
        WaitMs(1);
      } while (continue_loop == (uint8_t)1);

      status |= VL53L4CD_GetResult(&results);
      status |= VL53L4CD_ClearInterrupt();
      AvgDistance += (int16_t)results.distance_mm;
    }

    status |= VL53L4CD_StopRanging();
    AvgDistance = AvgDistance / nb_samples;
    *p_measured_offset_mm = (int16_t)TargetDistInMm - AvgDistance;
    tmpOff = (uint16_t) * p_measured_offset_mm * (uint16_t)4;
    status |= VL53L4CD_WrWord(dev, VL53L4CD_RANGE_OFFSET_MM, tmpOff);
  }

  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_CalibrateXtalk(
  int16_t TargetDistInMm,
  uint16_t *p_measured_xtalk_kcps,
  int16_t nb_samples)
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
  uint8_t i, tmp, continue_loop;
  float_t AverageSignal = (float_t)0.0;
  float_t AvgDistance = (float_t)0.0;
  float_t AverageSpadNb = (float_t)0.0;
  float_t tmp_xtalk;
  VL53L4CD_Result_t results;

  uint16_t calXtalk, j;

  if (((nb_samples < (int16_t)5) || (nb_samples > (int16_t)255))
      || ((TargetDistInMm < (int16_t)50)
          || (TargetDistInMm > (int16_t)5000))) {
    status |= (uint8_t)VL53L4CD_ERROR_INVALID_ARGUMENT;
  } else {
    /* Device heat loop (10 samples) */
    status |= VL53L4CD_StartRanging();
    for (i = 0; i < (uint8_t)10; i++) {
      tmp = (uint8_t)0;
      j = (uint16_t)0;
      continue_loop = (uint8_t)1;
      do {
        status |= VL53L4CD_CheckForDataReady(&tmp);
        if (tmp == (uint8_t)1) { /* Data ready */
          continue_loop = (uint8_t)0;
        } else if (j < (uint16_t)5000) { /* Wait for answer*/
          j++;
        } else { /* Timeout 5000ms reached */
          continue_loop = (uint8_t)0;
          status |= (uint8_t)VL53L4CD_ERROR_TIMEOUT;
        }
        WaitMs(1);
      } while (continue_loop == (uint8_t)1);
      status |= VL53L4CD_GetResult(&results);
      status |= VL53L4CD_ClearInterrupt();
    }
    status |= VL53L4CD_StopRanging();

    /* Device ranging loop */
    status |= VL53L4CD_StartRanging();
    for (i = 0; i < (uint8_t)nb_samples; i++) {
      tmp = (uint8_t)0;
      j = (uint16_t)0;
      continue_loop = (uint8_t)1;
      do {
        status |= VL53L4CD_CheckForDataReady(&tmp);
        if (tmp == (uint8_t)1) { /* Data ready */
          continue_loop = (uint8_t)0;
        } else if (j < (uint16_t)5000) { /* Wait for answer*/
          j++;
        } else { /* Timeout 5000ms reached */
          continue_loop = (uint8_t)0;
          status |= (uint8_t)VL53L4CD_ERROR_TIMEOUT;
        }
        WaitMs(1);
      } while (continue_loop == (uint8_t)1);

      status |= VL53L4CD_GetResult(&results);
      status |= VL53L4CD_ClearInterrupt();
      AvgDistance += (float_t)results.distance_mm;
      AverageSpadNb += (float_t)results.number_of_spad;
      AverageSignal += (float_t)results.signal_rate_kcps;
    }
    status |= VL53L4CD_StopRanging();
    AvgDistance = AvgDistance / (float_t)nb_samples;
    AverageSpadNb = AverageSpadNb / (float_t)nb_samples;
    AverageSignal = AverageSignal / (float_t)nb_samples;

    tmp_xtalk = (float_t)512.0 * (AverageSignal *
                                  ((float_t)1.0 - (AvgDistance
                                                   / (float_t)TargetDistInMm))) / AverageSpadNb;
    calXtalk = (uint16_t)tmp_xtalk;
    *p_measured_xtalk_kcps = (uint16_t)(calXtalk * (uint16_t)1000) >> 9;
    status |= VL53L4CD_WrWord(dev,
                              VL53L4CD_XTALK_PLANE_OFFSET_KCPS, calXtalk);

  }

  return status;
}
