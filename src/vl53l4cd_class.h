/**
 ******************************************************************************
 * @file    vl53l4cd_class.h
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    29 November 2021
 * @brief   Abstract Class for VL53L4CD sensor.
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

#ifndef __VL53L4CD_CLASS_H
#define __VL53L4CD_CLASS_H

/* Includes ------------------------------------------------------------------*/
#include "Arduino.h"
#include "Wire.h"
#include "vl53l4cd_api.h"

typedef float float_t;

/* Classes -------------------------------------------------------------------*/
/** Class representing a VL53LX sensor component
 */

class VL53L4CD {
  public:
    /** Constructor
     * @param[in] i2c device I2C to be used for communication
     * @param[in] xshut_pin pin to be used as component LPn
     * @param[in] address I2C address to be used for the sensor instance
     */
    VL53L4CD(TwoWire *i2c, int xshut_pin) : dev_i2c(i2c), xshut(xshut_pin)
    {
      dev = 0x52;
    }


    /** Destructor
     */
    virtual ~VL53L4CD() {}

    virtual int begin()
    {
      if (xshut >= 0) {
        pinMode(xshut, OUTPUT);
        digitalWrite(xshut, LOW);
      }
      return 0;
    }

    virtual int end()
    {
      if (xshut >= 0) {
        pinMode(xshut, INPUT);
      }
      return 0;
    }

    /*** Interface Methods ***/
    /*** High level API ***/
    /**
     * @brief       PowerOn the sensor
     * @return      void
     */
    virtual void VL53L4CD_On(void)
    {
      if (xshut >= 0) {
        digitalWrite(xshut, HIGH);
      }
      delay(10);
    }

    /**
     * @brief       PowerOff the sensor
     * @return      void
     */
    virtual void VL53L4CD_Off(void)
    {
      if (xshut >= 0) {
        digitalWrite(xshut, LOW);
      }
      delay(10);
    }

    /**
     * @brief Initialize the sensor with default values
     * @param (uint8_t) addr : New I2C address.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */

    VL53L4CD_ERROR InitSensor(uint8_t addr = 0x52)
    {
      VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
      uint16_t sensor_id;
      VL53L4CD_Off();
      VL53L4CD_On();

      if (addr != dev) {
        status = VL53L4CD_SetI2CAddress(addr);
      }

      if (status != VL53L4CD_ERROR_NONE) {
        return VL53L4CD_ERROR_TIMEOUT;
      }

      status = VL53L4CD_GetSensorId(&sensor_id);

      if (status != VL53L4CD_ERROR_NONE && (sensor_id != 0xebaa)) {
        return VL53L4CD_ERROR_TIMEOUT;
      }

      status = VL53L4CD_SensorInit();

      if (status != VL53L4CD_ERROR_NONE) {
        return VL53L4CD_ERROR_TIMEOUT;
      }

      return VL53L4CD_ERROR_NONE;
    }


    /* Main APIs */
    /**
     * @brief This function programs the software driver version.
     * @param (VL53L4CD_Version_t) pVersion : Pointer of structure, containing the
     * software version.
     * @return (VL53L4CD_ERROR) status : 0 if SW version is OK.
     */

    VL53L4CD_ERROR VL53L4CD_GetSWVersion(
      VL53L4CD_Version_t *pVersion);


    /**
     * @brief This function sets a new I2C address to a sensor. It can be used for
     * example when multiple sensors share the same I2C bus.
     * @param (uint8_t) new_address : New I2C address.
     * @return (VL53L4CD_ERROR) status : 0 if I2C address has been correctly
     * programmed.
     */

    VL53L4CD_ERROR VL53L4CD_SetI2CAddress(
      uint8_t new_address);

    /**
     * @brief This function is used to get the sensor id of VL53L4CD. The sensor id
     * should be 0xEBAA.
     * @param (uint16_t) *p_id : Sensor id.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */

    VL53L4CD_ERROR VL53L4CD_GetSensorId(
      uint16_t *p_id);

    /**
     * @brief This function is used to initialize the sensor.
     * @return (VL53L4CD_ERROR) status : 0 if init is OK.
     */

    VL53L4CD_ERROR VL53L4CD_SensorInit();

    /**
     * @brief This function clears the interrupt. It needs to be called after a
     * ranging data reading to arm the interrupt for the next data ready event.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */

    VL53L4CD_ERROR VL53L4CD_ClearInterrupt();

    /**
     * @brief This function starts a ranging session. The ranging operation is
     * continuous. The clear interrupt has to be done after each get data to allow
     * the interrupt to raise when the next data is ready.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */

    VL53L4CD_ERROR VL53L4CD_StartRanging();

    /**
     * @brief This function stops the ranging in progress.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */

    VL53L4CD_ERROR VL53L4CD_StopRanging();

    /**
     * @brief This function check if a new data is available by polling a dedicated
     * register.
     * @param (uint8_t) *p_is_data_ready : Pointer containing a flag to know if a
     * data is ready : 0 = no data ready, 1 = data ready.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */

    VL53L4CD_ERROR VL53L4CD_CheckForDataReady(
      uint8_t *p_is_data_ready);

    /**
     * @brief This function sets new range timing. Timing are composed of
     * TimingBudget and InterMeasurement. TimingBudget represents the timing during
     * VCSEL enabled, and InterMeasurement the time between two measurements.
     * The sensor can have different ranging mode depending of the configuration,
     * please refer to the user manual for more information.
     * @param (uint32_t) timing_budget_ms :  New timing budget in ms. Value can be
     * between 10ms and 200ms. Default is 50ms.
     * @param (uint32_t) inter_measurement_ms :  New inter-measurement in ms. If the
     * value is equal to 0, the ranging period is defined by the timing budget.
     * Otherwise, inter-measurement must be > timing budget. When all the timing
     * budget is consumed, the device goes in low power mode until inter-measurement
     * is done.
     * @return (uint8_t) status : 0 if OK.
     */

    VL53L4CD_ERROR VL53L4CD_SetRangeTiming(
      uint32_t timing_budget_ms,
      uint32_t inter_measurement_ms);

    /**
     * @brief This function gets the current range timing. Timing are composed of
     * TimingBudget and InterMeasurement. TimingBudget represents the timing during
     * VCSEL enabled, and InterMeasurement the time between two measurements.
     * The sensor can have different ranging mode depending of the configuration,
     * please refer to the user manual for more information.
     * @param (uint32_t) *p_timing_budget_ms :  Pointer containing the current
     * timing budget in ms.
     * @param (uint32_t) *p_inter_measurement_ms :  Pointer containing the current
     * inter-measurement in ms.
     * @return (uint8_t) status : 0 if OK.
     */

    VL53L4CD_ERROR VL53L4CD_GetRangeTiming(
      uint32_t *p_timing_budget_ms,
      uint32_t *p_inter_measurement_ms);

    /**
     * @brief This function gets the results reported by the sensor.
     * @param (VL53L4CD_Result_t) *pResult :  Pointer of structure, filled with the
     * ranging results.
     * @return (uint8_t) status : 0 if OK.
     */

    VL53L4CD_ERROR VL53L4CD_GetResult(VL53L4CD_Result_t *pResult);

    /**
     * @brief This function sets a new offset correction in mm. Offset corresponds
     * to the difference in millimeters between real distance and measured distance.
     * @param (int16_t) OffsetValueInMm :  Offset value in millimeters. The minimum
     *  value is -1024mm and maximum is 1023mm.
     * @return (uint8_t) status : 0 if OK.
     */

    VL53L4CD_ERROR VL53L4CD_SetOffset(int16_t OffsetValueInMm);

    /**
     * @brief This function gets the current offset correction in mm. Offset
     * corresponds to the difference in millimeters between real distance and
     * measured distance.
     * @param (int16_t) OffsetValueInMm :  Offset value in millimeters. The minimum
     *  value is -1024mm and maximum is 1023mm.
     * @return (uint8_t) status : 0 if OK.
     */

    VL53L4CD_ERROR VL53L4CD_GetOffset(int16_t *Offset);

    /**
     * @brief This function sets a new Xtalk value in kcps. Xtalk represents the
     * correction to apply to the sensor when a protective coverglass is placed
     * at the top of the sensor.
     * @param (uint16_t) XtalkValueKcps : New xtalk value in kcps. The default
     * value is 0 kcps (no coverglass). Minimum is 0 kcps , and maximum is 128
     * kcps.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */

    VL53L4CD_ERROR VL53L4CD_SetXtalk(uint16_t XtalkValueKcps);

    /**
     * @brief This function gets the current Xtalk value in kcps. Xtalk represents
     * the correction to apply to the sensor when a protective coverglass is placed
     * at the top of the sensor.
     * @param (uint16_t) p_xtalk_kcps : Pointer of current xtalk value in kcps.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */

    VL53L4CD_ERROR VL53L4CD_GetXtalk(uint16_t *p_xtalk_kcps);

    /**
     * @brief This function sets new detection thresholds. The detection
     * thresholds can be programmed to generate an interrupt on pin 7 (GPIO1), only
     * when a condition on distance is reach. Example:
     * VL53L4CD_SetDistanceThreshold(dev,100,300,0): Below 100 mm
     * VL53L4CD_SetDistanceThreshold(dev,100,300,1): Above 300 mm
     * VL53L4CD_SetDistanceThreshold(dev,100,300,2): Below 100mm or above 300mm
     * VL53L4CD_SetDistanceThreshold(dev,100,300,3): Above 100mm or below 300mm
     * @param (uint16_t) distance_low_mm : Low distance threshold in millimeters.
     * @param (uint16_t) distance_high_mm : High distance threshold in millimeters.
     * @param (uint8_t) window : Interrupt windows (0=below low threshold;
     * 1=above high threshold; 2=out of low/high windows; 3=in low/high windows)
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */

    VL53L4CD_ERROR VL53L4CD_SetDetectionThresholds(
      uint16_t distance_low_mm,
      uint16_t distance_high_mm,
      uint8_t window);


    /**
     * @brief This function gets the current detection thresholds. The detection
     * thresholds can be programmed to generate an interrupt on pin 7 (GPIO1), only
     * when a condition on distance is reach.
     * @param (uint16_t) *p_distance_low_mm : Pointer of low distance threshold in
     * millimeters.
     * @param (uint16_t) *p_distance_high_mm : Pointer of high distance threshold in
     * millimeters.
     * @param (uint8_t) *p_window : Interrupt windows (0=below low threshold;
     * 1=above high threshold; 2=out of low/high windows; 3=in low/high windows)
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */

    VL53L4CD_ERROR VL53L4CD_GetDetectionThresholds(
      uint16_t *p_distance_low_mm,
      uint16_t *p_distance_high_mm,
      uint8_t *p_window);

    /**
     * @brief This function sets a new signal threshold in kcps. If a
     * target has a lower signal as the programmed value, the result status in
     * structure 'VL53L4CD_Result_t' will be equal to 2.
     * @param (uint16_t) signal_kcps : New signal threshold in kcps. The default
     * value is 1024 kcps. Minimum is 0 kcps (no threshold), and maximum is 16384
     * kcps.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */

    VL53L4CD_ERROR VL53L4CD_SetSignalThreshold(uint16_t signal_kcps);

    /**
     * @brief This function returns the current signal threshold in kcps. If a
     * target has a lower signal as the programmed value, the result status in
     * structure 'VL53L4CD_Result_t' will be equal to 2.
     * @param (uint16_t) *p_signal_kcps : Pointer of signal threshold in kcps.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */

    VL53L4CD_ERROR VL53L4CD_GetSignalThreshold(
      uint16_t *p_signal_kcps);

    /**
     * @brief This function programs a new sigma threshold. The sigma corresponds to
     * the standard deviation of the returned pulse. If the computed sigma is above
     * the programmed value, the result status in structure 'VL53L4CD_Result_t'
     * will be equal to 1.
     * @param (uint16_t) sigma_mm : New sigma threshold in mm. The default value is
     * 15mm. Minimum is 0mm (not threshold), and maximum is 16383mm.
     * @return (VL53L4CD_ERROR) status : 0 if programming is or 255 if value is too
     * high.
     */

    VL53L4CD_ERROR VL53L4CD_SetSigmaThreshold(
      uint16_t  sigma_mm);

    /**
     * @brief This function gets the current sigma threshold. The sigma corresponds
     * to the standard deviation of the returned pulse. If the computed sigma is
     * above the programmed value, the result status in structure
     * 'VL53L4CD_Result_t' will be equal to 1.
     * @param (uint16_t) *p_sigma_mm : Current sigma threshold in mm.
     * @return (VL53L4CD_ERROR) status : 0 if programming is OK.
     */

    VL53L4CD_ERROR VL53L4CD_GetSigmaThreshold(
      uint16_t  *p_sigma_mm);

    /**
     * @brief This function can be called when the temperature might have changed by
     * more than 8 degrees Celsius. The function can only be used if the sensor is
     * not ranging, otherwise, the ranging needs to be stopped using function
     * 'VL53L4CD_StopRanging()'. After calling this function, the ranging can
     * restart normally.
     * @return (VL53L4CD_ERROR) status : 0 if update is OK.
     */

    VL53L4CD_ERROR VL53L4CD_StartTemperatureUpdate();

    /* Calibration APIs */

    /**
     * @brief This function can be used to perform an offset calibration. Offset
     * corresponds to the difference in millimeters between real distance and
     * measured distance. ST recommend to perform offset at 100m, on a grey17%
     * reflective target, but any other distance and reflectance can be used.
     * The function returns the offset value found and programs the offset
     * compensation into the device.
     * @param (int16_t) TargetDistInMm : Real distance between the sensor and the
     * target in millimeters. ST recommend 100mm. Min distance is 50mm and max is
     * 1000mm.
     * @param (int16_t) nb_samples : Number of samples (between 5 and 255). A higher
     * number of samples increases the accuracy, but it also takes more time. ST
     * recommend to use at least 10 samples.
     * @return (VL53L4CD_ERROR) status : 0 if OK, or 255 if something occurred (e.g
     * invalid nb of samples).
     */

    VL53L4CD_ERROR VL53L4CD_CalibrateOffset(
      int16_t TargetDistInMm,
      int16_t *p_measured_offset_mm,
      int16_t nb_samples);


    /**
     * @brief This function can be used to perform a Xtalk calibration. Xtalk
     * represents the correction to apply to the sensor when a protective coverglass
     * is placed at the top of the sensor. The distance for calibration depends of
     * the coverglass, it needs to be characterized. Please refer to the User Manual
     * for more information.
     * The function returns the Xtalk value found and programs the Xtalk
     * compensation into the device.
     * @param uint16_t) TargetDistInMm : Real distance between the sensor and the
     * target in millimeters. This distance needs to be characterized, as described
     * into the User Manual.
     * @param (int16_t) nb_samples : Number of samples (between 5 and 255). A higher
     * number of samples increases the accuracy, but it also takes more time. ST
     * recommend to use at least 10 samples.
     * @return (VL53L4CD_ERROR) status : 0 if OK, or 255 if something occurred (e.g
     * invalid nb of samples).
     */

    VL53L4CD_ERROR VL53L4CD_CalibrateXtalk(
      int16_t TargetDistInMm,
      uint16_t *p_measured_xtalk_kcps,
      int16_t nb_samples);

  protected:

    /**
     * @brief Read 32 bits through I2C.
     */

    uint8_t VL53L4CD_RdDWord(uint16_t dev, uint16_t registerAddr, uint32_t *value);

    /**
     * @brief Read 16 bits through I2C.
     */

    uint8_t VL53L4CD_RdWord(uint16_t dev, uint16_t registerAddr, uint16_t *value);

    /**
     * @brief Read 8 bits through I2C.
     */

    uint8_t VL53L4CD_RdByte(uint16_t dev, uint16_t registerAddr, uint8_t *value);

    /**
     * @brief Write 8 bits through I2C.
     */

    uint8_t VL53L4CD_WrByte(uint16_t dev, uint16_t registerAddr, uint8_t value);

    /**
     * @brief Write 16 bits through I2C.
     */

    uint8_t VL53L4CD_WrWord(uint16_t dev, uint16_t RegisterAdress, uint16_t value);

    /**
     * @brief Write 32 bits through I2C.
     */

    uint8_t VL53L4CD_WrDWord(uint16_t dev, uint16_t RegisterAdress, uint32_t value);

    /**
     * @brief Wait during N milliseconds.
     */

    void WaitMs(uint32_t TimeMs);

    /**
    * @brief Read multiple bytes through I2C.
    */

    uint8_t VL53L4CD_I2CRead(uint8_t DeviceAddr, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size);

    /**
    * @brief Write multiple bytes through I2C.
    */

    uint8_t VL53L4CD_I2CWrite(uint8_t DeviceAddr, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size);

  protected:

    /* IO Device */
    TwoWire *dev_i2c;
    /* Digital out pin */
    int xshut;
    /* I2C address to use */
    uint16_t dev;
};

#endif /* __VL53L4CD_CLASS_H */
