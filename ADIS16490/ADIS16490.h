////////////////////////////////////////////////////////////////////////////////////////////////////////
//  April 2017
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16490.h
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This library provides all the functions necessary to interface the ADIS16490 IMU with a 
//  PJRC 32-Bit Teensy 3.2 Development Board. Functions for SPI configuration, reads and writes, 
//  page manipulation and verification, and scaling are included. This library may be used for 
//  the entire ADIS1648X family of devices with some modification.
//
//  Permission is hereby granted, free of charge, to any person obtaining
//  a copy of this software and associated documentation files (the
//  "Software"), to deal in the Software without restriction, including
//  without limitation the rights to use, copy, modify, merge, publish,
//  distribute, sublicense, and/or sell copies of the Software, and to
//  permit persons to whom the Software is furnished to do so, subject to
//  the following conditions:
//
//  The above copyright notice and this permission notice shall be
//  included in all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
//  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ADIS16490_h
#define ADIS16490_h
#include "Arduino.h"
#include <SPI.h>

// User Register Memory Map Page 0
#define PAGE_ID       0x0000
#define DATA_CNT      0x0004
#define SYS_E_FLAG    0x0008
#define DIAG_STS      0x000A
#define TEMP_OUT      0x000E
#define X_GYRO_LOW    0x0010
#define X_GYRO_OUT    0x0012
#define Y_GYRO_LOW    0x0014
#define Y_GYRO_OUT    0x0016
#define Z_GYRO_LOW    0x0018
#define Z_GYRO_OUT    0x001A
#define X_ACCL_LOW    0x001C
#define X_ACCL_OUT    0x001E
#define Y_ACCL_LOW    0x0020
#define Y_ACCL_OUT    0x0022
#define Z_ACCL_LOW    0x0024
#define Z_ACCL_OUT    0x0026
#define TIME_STAMP    0x0028
#define X_DELTANG_LOW 0x0040
#define X_DELTANG_OUT 0x0042
#define Y_DELTANG_LOW 0x0044
#define Y_DELTANG_OUT 0x0046
#define Z_DELTANG_LOW 0x0048
#define Z_DELTANG_OUT 0x004A
#define X_DELTVEL_LOW 0x004C
#define X_DELTVEL_OUT 0x004E
#define Y_DELTVEL_LOW 0x0050
#define Y_DELTVEL_OUT 0x0052
#define Z_DELTVEL_LOW 0x0054
#define Z_DELTVEL_OUT 0x0056
#define PROD_ID       0x007E

// User Register Memory Map Page 2
#define PAGE_ID2      0x0200
#define X_GYRO_SCALE  0x0204
#define Y_GYRO_SCALE  0x0206
#define Z_GYRO_SCALE  0x0208
#define X_ACCL_SCALE  0x020A
#define Y_ACCL_SCALE  0x020C
#define Z_ACCL_SCALE  0x020E
#define XG_BIAS_LOW   0x0210
#define XG_BIAS_HIGH  0x0212
#define YG_BIAS_LOW   0x0214
#define YG_BIAS_HIGH  0x0216
#define ZG_BIAS_LOW   0x0218
#define ZG_BIAS_HIGH  0x021A
#define XA_BIAS_LOW   0x021C
#define XA_BIAS_HIGH  0x021E
#define YA_BIAS_LOW   0x0220
#define YA_BIAS_HIGH  0x0222
#define ZA_BIAS_LOW   0x0224
#define ZA_BIAS_HIGH  0x0226
#define USER_SCR_1    0x0274
#define USER_SCR_2    0x0276
#define USER_SCR_3    0x0278
#define USER_SCR_4    0x027A
#define FLSHCNT_LOW   0x027C
#define FLSHCNT_HIGH  0x027E

// User Register Memory Map Page 3
#define PAGE_ID3      0x0300
#define GLOB_CMD      0x0302
#define FNCTIO_CTRL   0x0306
#define GPIO_CTRL     0x0308
#define CONFIG        0x030A
#define DEC_RATE      0x030C
#define NULL_CNFG     0x030E
#define SYNC_SCALE    0x0310
#define FILTR_BNK_0   0x0316
#define FILTR_BNK_1   0x0318
#define FIRM_REV      0x0378
#define FIRM_DM       0x037A
#define FIRM_Y        0x037C
#define BOOT_REV      0x037E

// User Register Memory Map Page 4
#define PAGE_ID4      0x0400
#define CAL_SIGTR_LWR 0x0404
#define CAL_SIGTR_UPR 0x0406
#define CAL_DRVTN_LWR 0x0408
#define CAL_DRVTN_UPR 0x040A
#define CODE_SIGTR_LWR  0x040C
#define CODE_SIGTR_UPR  0x040E
#define CODE_DRVTN_LWR  0x0410
#define CODE_DRVTN_UPR  0x0412
#define SERIAL_NUM    0x0420

// FIR filter banks are contained on pages 5 ~ 12. Since these banks will
// likely not be written to individually, I've left it up to the user
// to iterate through the banks when writing their own coefficients.

// ADIS16490 class definition
class ADIS16490 {

public:
  // Constructor with configurable CS, data ready, and HW reset pins

  // ADIS16490(int CS, int DR, int RST, int MOSI, int MISO, int CLK);
  ADIS16490(int CS, int DR, int RST);

  // Destructor
  ~ADIS16490();

  // Performs hardware reset by sending pin 8 low on the DUT for 2 seconds
  int resetDUT(uint8_t ms);

  // Sets SPI bit order, clock divider, and data mode
  int configSPI();

  // Read single register from sensor
  int16_t regRead(uint16_t regAddr);

  // Write register
  int regWrite(uint16_t regAddr, int16_t regData);

  // Read a fixed set of sensor data
  int16_t *sensorRead(void);

  // Scale accelerator data
  float accelScale(int16_t sensorData);

  // Scale gyro data
  float gyroScale(int16_t sensorData);

  // Scale temperature data
  float tempScale(int16_t sensorData);

  // Scale delta angle data
  float deltaAngleScale(int16_t sensorData);

  // Scale delta velocity
  float deltaVelocityScale(int16_t sensorData);

private:
  // Chip select pin
  int _CS;

  // IRQ output pin for data ready
  int _DR;

  // Hardware reset pin
  int _RST;

  // SPI stall time
  int _stall = 5;

  // Current page
  int currentPage = 0x00;

};

#endif
