/**
 ******************************************************************************
 * @file    vl53l4cd_api.h
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    29 November 2021
 * @brief   Header file for the VL53L4CD main structures.
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

#ifndef VL53L4CD_API_H_
#define VL53L4CD_API_H_

/**
 *  @brief Driver version
 */

#define VL53L4CD_IMPLEMENTATION_VER_MAJOR       1
#define VL53L4CD_IMPLEMENTATION_VER_MINOR       0
#define VL53L4CD_IMPLEMENTATION_VER_BUILD       0
#define VL53L4CD_IMPLEMENTATION_VER_REVISION    0

/**
 *  @brief Driver error type
 */

typedef uint8_t VL53L4CD_ERROR;

#define VL53L4CD_ERROR_NONE         ((uint8_t)0U)
#define VL53L4CD_ERROR_INVALID_ARGUMENT   ((uint8_t)254U)
#define VL53L4CD_ERROR_TIMEOUT        ((uint8_t)255U)


/**
 *  @brief Inner Macro for API. Not for user, only for development.
 */

#define VL53L4CD_SOFT_RESET     ((uint16_t)0x0000))
#define VL53L4CD_I2C_SLAVE_DEVICE_ADDRESS      ((uint16_t)0x0001)
#define VL53L4CD_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND  ((uint16_t)0x0008)
#define VL53L4CD_XTALK_PLANE_OFFSET_KCPS ((uint16_t)0x0016)
#define VL53L4CD_XTALK_X_PLANE_GRADIENT_KCPS     ((uint16_t)0x0018)
#define VL53L4CD_XTALK_Y_PLANE_GRADIENT_KCPS     ((uint16_t)0x001A)
#define VL53L4CD_RANGE_OFFSET_MM     ((uint16_t)0x001E)
#define VL53L4CD_INNER_OFFSET_MM     ((uint16_t)0x0020)
#define VL53L4CD_OUTER_OFFSET_MM     ((uint16_t)0x0022)
#define VL53L4CD_I2C_FAST_MODE_PLUS     ((uint16_t)0x002D)
#define VL53L4CD_GPIO_HV_MUX_CTRL      ((uint16_t)0x0030)
#define VL53L4CD_GPIO_TIO_HV_STATUS    ((uint16_t)0x0031)
#define VL53L4CD_SYSTEM_INTERRUPT  ((uint16_t)0x0046)
#define VL53L4CD_RANGE_CONFIG_A     ((uint16_t)0x005E)
#define VL53L4CD_RANGE_CONFIG_B      ((uint16_t)0x0061)
#define VL53L4CD_RANGE_CONFIG_SIGMA_THRESH     ((uint16_t)0x0064)
#define VL53L4CD_MIN_COUNT_RATE_RTN_LIMIT_MCPS    ((uint16_t)0x0066)
#define VL53L4CD_INTERMEASUREMENT_MS ((uint16_t)0x006C)
#define VL53L4CD_THRESH_HIGH    ((uint16_t)0x0072)
#define VL53L4CD_THRESH_LOW     ((uint16_t)0x0074)
#define VL53L4CD_SYSTEM_INTERRUPT_CLEAR        ((uint16_t)0x0086)
#define VL53L4CD_SYSTEM_START     ((uint16_t)0x0087)
#define VL53L4CD_RESULT_RANGE_STATUS   ((uint16_t)0x0089)
#define VL53L4CD_RESULT_SPAD_NB   ((uint16_t)0x008C)
#define VL53L4CD_RESULT_SIGNAL_RATE   ((uint16_t)0x008E)
#define VL53L4CD_RESULT_AMBIENT_RATE   ((uint16_t)0x0090)
#define VL53L4CD_RESULT_SIGMA   ((uint16_t)0x0092)
#define VL53L4CD_RESULT_DISTANCE   ((uint16_t)0x0096)


#define VL53L4CD_RESULT_OSC_CALIBRATE_VAL      ((uint16_t)0x00DE)
#define VL53L4CD_FIRMWARE_SYSTEM_STATUS        ((uint16_t)0x00E5)
#define VL53L4CD_IDENTIFICATION_MODEL_ID       ((uint16_t)0x010F)

/**
 *  @brief defines Software Version
 */

typedef struct {
  uint8_t      major;    /*!< major number */
  uint8_t      minor;    /*!< minor number */
  uint8_t      build;    /*!< build number */
  uint32_t     revision; /*!< revision number */
} VL53L4CD_Version_t;

/**
 *  @brief Packed reading results type
 */

typedef struct {

  /* Status of measurements. If the status is equal to 0, the data are valid*/
  uint8_t range_status;
  /* Measured distance in millimeters */
  uint16_t distance_mm;
  /* Ambient noise in kcps */
  uint16_t ambient_rate_kcps;
  /* Ambient noise in kcps/SPAD */
  uint16_t ambient_per_spad_kcps;
  /* Measured signal of the target in kcps */
  uint16_t signal_rate_kcps;
  /* Measured signal of the target in kcps/SPAD */
  uint16_t signal_per_spad_kcps;
  /* Number of SPADs enabled */
  uint16_t number_of_spad;
  /* Estimated measurements std deviation in mm */
  uint16_t sigma_mm;
} VL53L4CD_Result_t;

#endif  //VL53L4CD_API_H_
