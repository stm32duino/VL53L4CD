/**
 ******************************************************************************
 * @file    platform.cpp
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    29 November 2021
 * @brief   Implementation of the platform dependent APIs.
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

#ifndef DEFAULT_I2C_BUFFER_LEN
  #ifdef BUFFER_LENGTH
    #define DEFAULT_I2C_BUFFER_LEN  BUFFER_LENGTH
  #else
    #define DEFAULT_I2C_BUFFER_LEN  32
  #endif
#endif

uint8_t VL53L4CD::VL53L4CD_RdDWord(uint16_t dev, uint16_t RegisterAdress, uint32_t *value)
{
  uint8_t status = 0;
  uint8_t buffer[4] = {0, 0, 0, 0};

  status = VL53L4CD_I2CRead(dev, RegisterAdress, buffer, 4);
  if (!status) {
    *value = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];
  }
  return status;
}

uint8_t VL53L4CD::VL53L4CD_RdWord(uint16_t dev, uint16_t RegisterAdress, uint16_t *value)
{
  uint8_t status = 0;
  uint8_t buffer[2] = {0, 0};

  status = VL53L4CD_I2CRead(dev, RegisterAdress, buffer, 2);
  if (!status) {
    *value = (buffer[0] << 8) + buffer[1];
  }
  return status;
}

uint8_t VL53L4CD::VL53L4CD_RdByte(uint16_t dev, uint16_t RegisterAdress, uint8_t *value)
{
  uint8_t status = 0;
  status = VL53L4CD_I2CRead(dev, RegisterAdress, value, 1);
  return status;
}

uint8_t VL53L4CD::VL53L4CD_WrByte(uint16_t dev, uint16_t RegisterAdress, uint8_t value)
{
  uint8_t status = 0;
  status = VL53L4CD_I2CWrite(dev, RegisterAdress, &value, 1);
  return status;
}

uint8_t VL53L4CD::VL53L4CD_WrWord(uint16_t dev, uint16_t RegisterAdress, uint16_t value)
{
  uint8_t status = 0;
  uint8_t buffer[2];

  buffer[0] = value >> 8;
  buffer[1] = value & 0x00FF;
  status = VL53L4CD_I2CWrite(dev, RegisterAdress, (uint8_t *)buffer, 2);
  return status;
}

uint8_t VL53L4CD::VL53L4CD_WrDWord(uint16_t dev, uint16_t RegisterAdress, uint32_t value)
{
  uint8_t status = 0;
  uint8_t buffer[4];

  buffer[0] = (value >> 24) & 0xFF;
  buffer[1] = (value >> 16) & 0xFF;
  buffer[2] = (value >>  8) & 0xFF;
  buffer[3] = (value >>  0) & 0xFF;
  status = VL53L4CD_I2CWrite(dev, RegisterAdress, (uint8_t *)buffer, 4);
  return status;
}

void VL53L4CD::WaitMs(uint32_t TimeMs)
{
  delay(TimeMs);
}

uint8_t VL53L4CD::VL53L4CD_I2CRead(uint8_t DeviceAddr, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size)
{
  int status = 0;
  uint8_t buffer[2];

  // Loop until the port is transmitted correctly
  do {
    dev_i2c->beginTransmission((uint8_t)((DeviceAddr >> 1) & 0x7F));

    // Target register address for transfer
    buffer[0] = (uint8_t)(RegisterAddress >> 8);
    buffer[1] = (uint8_t)(RegisterAddress & 0xFF);
    dev_i2c->write(buffer, 2);

    status = dev_i2c->endTransmission(false);

    // Fix for some STM32 boards
    // Reinitialize the i2c bus with the default parameters
#ifdef ARDUINO_ARCH_STM32
    if (status) {
      dev_i2c->end();
      dev_i2c->begin();
    }
#endif
    // End of fix

  } while (status != 0);

  uint32_t i = 0;
  if (size > DEFAULT_I2C_BUFFER_LEN) {
    while (i < size) {
      // If still more than DEFAULT_I2C_BUFFER_LEN bytes to go, DEFAULT_I2C_BUFFER_LEN,
      // else the remaining number of bytes
      uint8_t current_read_size = (size - i > DEFAULT_I2C_BUFFER_LEN ? DEFAULT_I2C_BUFFER_LEN : size - i);
      dev_i2c->requestFrom(((uint8_t)((DeviceAddr >> 1) & 0x7F)),
                           current_read_size);
      while (dev_i2c->available()) {
        p_values[i] = dev_i2c->read();
        i++;
      }
    }
  } else {
    dev_i2c->requestFrom(((uint8_t)((DeviceAddr >> 1) & 0x7F)), size);
    while (dev_i2c->available()) {
      p_values[i] = dev_i2c->read();
      i++;
    }
  }

  return i != size;
}

uint8_t VL53L4CD::VL53L4CD_I2CWrite(uint8_t DeviceAddr, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size)
{
  uint32_t i = 0;
  uint8_t buffer[2];

  while (i < size) {
    // If still more than DEFAULT_I2C_BUFFER_LEN bytes to go, DEFAULT_I2C_BUFFER_LEN,
    // else the remaining number of bytes
    size_t current_write_size = (size - i > DEFAULT_I2C_BUFFER_LEN ? DEFAULT_I2C_BUFFER_LEN : size - i);

    dev_i2c->beginTransmission((uint8_t)((DeviceAddr >> 1) & 0x7F));

    // Target register address for transfer
    buffer[0] = (uint8_t)((RegisterAddress + i) >> 8);
    buffer[1] = (uint8_t)((RegisterAddress + i) & 0xFF);
    dev_i2c->write(buffer, 2);
    if (dev_i2c->write(p_values + i, current_write_size) == 0) {
      return 1;
    } else {
      i += current_write_size;
      if (size - i) {
        // Flush buffer but do not send stop bit so we can keep going
        dev_i2c->endTransmission(false);
      }
    }
  }

  return dev_i2c->endTransmission(true);
}
