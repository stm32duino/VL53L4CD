/**
 ******************************************************************************
 * @file    VL53L4CD_Sat_HelloWorld.ino
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    29 November 2021
 * @brief   Arduino test application for the STMicrolectronics VL53L4CD
 *          proximity sensor satellite based on FlightSense.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
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

/*
 * To use this sketch you need to connect the VL53L4CD satellite sensor directly to the Nucleo board with wires in this way:
 * pin 1 (GND) of the VL53L4CD satellite connected to GND of the Nucleo board
 * pin 2 (VDD) of the VL53L4CD satellite connected to 3V3 pin of the Nucleo board
 * pin 3 (SCL) of the VL53L4CD satellite connected to pin D15 (SCL) of the Nucleo board
 * pin 4 (SDA) of the VL53L4CD satellite connected to pin D14 (SDA) of the Nucleo board
 * pin 5 (GPIO1) of the VL53L4CD satellite connected to pin A2 of the Nucleo board
 * pin 6 (XSHUT) of the VL53L4CD satellite connected to pin A1 of the Nucleo board
 */
/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#define DEV_I2C Wire
#define SerialPort Serial

#ifndef LED_BUILTIN
  #define LED_BUILTIN 13
#endif
#define LedPin LED_BUILTIN

// Components.
VL53L4CD sensor_vl53l4cd_sat(&DEV_I2C, A1);

/* Setup ---------------------------------------------------------------------*/

void setup()
{
  // Led.
  pinMode(LedPin, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);
  SerialPort.println("Starting...");

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L4CD satellite component.
  sensor_vl53l4cd_sat.begin();

  // Switch off VL53L4CD satellite component.
  sensor_vl53l4cd_sat.VL53L4CD_Off();

  //Initialize VL53L4CD satellite component.
  sensor_vl53l4cd_sat.InitSensor();

  // Program the highest possible TimingBudget, without enabling the
  // low power mode. This should give the best accuracy
  sensor_vl53l4cd_sat.VL53L4CD_SetRangeTiming(200, 0);

  // Start Measurements
  sensor_vl53l4cd_sat.VL53L4CD_StartRanging();
}

void loop()
{
  uint8_t NewDataReady = 0;
  VL53L4CD_Result_t results;
  uint8_t status;
  char report[64];

  do {
    status = sensor_vl53l4cd_sat.VL53L4CD_CheckForDataReady(&NewDataReady);
  } while (!NewDataReady);

  //Led on
  digitalWrite(LedPin, HIGH);

  if ((!status) && (NewDataReady != 0)) {
    // (Mandatory) Clear HW interrupt to restart measurements
    sensor_vl53l4cd_sat.VL53L4CD_ClearInterrupt();

    // Read measured distance. RangeStatus = 0 means valid data
    sensor_vl53l4cd_sat.VL53L4CD_GetResult(&results);
    snprintf(report, sizeof(report), "Status = %3u, Distance = %5u mm, Signal = %6u kcps/spad\r\n",
             results.range_status,
             results.distance_mm,
             results.signal_per_spad_kcps);
    SerialPort.print(report);
  }

  //Led off
  digitalWrite(LedPin, LOW);
}
