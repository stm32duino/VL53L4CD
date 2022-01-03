# VL53L4CD
Arduino library to support the VL53L4CD Time-of-Flight ranging sensor.

## API

This sensor uses I2C to communicate. And I2C instance is required to access to the sensor.
The APIs provide simple distance measure in both polling and interrupt modes.

## Examples

There are 2 examples with the VL53L4CD library.

In order to use these examples you need to connect the VL53L4CD satellite sensor directly to the Nucleo board with wires as explained below:
- pin 1 (GND) of the VL53L4CD satellite connected to GND of the Nucleo board
- pin 2 (VDD) of the VL53L4CD satellite connected to 3V3 pin of the Nucleo board
- pin 3 (SCL) of the VL53L4CD satellite connected to pin D15 (SCL) of the Nucleo board
- pin 4 (SDA) of the VL53L4CD satellite connected to pin D14 (SDA) of the Nucleo board
- pin 5 (GPIO1) of the VL53L4CD satellite connected to pin A2 of the Nucleo board
- pin 6 (XSHUT) of the VL53L4CD satellite connected to pin A1 of the Nucleo board

* VL53L4CD_Sat_HelloWorld: This example code is to show how to get proximity
  values of the VL53L4CD satellite sensor in polling mode.

* VL53L4CD_Sat_HelloWorld_Interrupt: This example code is to show how to get proximity
  values of the VL53L4CD satellite sensor in interrupt mode.

## Documentation

You can find the source files at  
https://github.com/stm32duino/VL53L4CD

The VL53L4CD datasheet is available at  
https://www.st.com/content/st_com/en/products/imaging-and-photonics-solutions/proximity-sensors/vl53l4cd.html
