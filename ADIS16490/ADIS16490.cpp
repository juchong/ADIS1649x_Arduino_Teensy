////////////////////////////////////////////////////////////////////////////////////////////////////////
//  April 2017
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16490.cpp
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

#include "ADIS16490.h"

////////////////////////////////////////////////////////////////////////////
// Constructor with configurable CS, DR, and RST
////////////////////////////////////////////////////////////////////////////
// CS - Chip select pin
// DR - DR output pin for data ready
// RST - Hardware reset pin
////////////////////////////////////////////////////////////////////////////
ADIS16490::ADIS16490(int CS, int DR, int RST) {
  _CS = CS;
  _DR = DR;
  _RST = RST;
// Initialize SPI
  SPI.begin();
// Configure SPI controller
  configSPI();
// Set default pin states
  pinMode(_CS, OUTPUT); // Set CS pin to be an output
  pinMode(_DR, INPUT); // Set DR pin to be an input
  pinMode(_RST, OUTPUT); // Set RST pin to be an output
  digitalWrite(_CS, HIGH); // Initialize CS pin to be high
  digitalWrite(_RST, HIGH); // Initialize RST pin to be high
}
 
////////////////////////////////////////////////////////////////////////////
// Destructor
////////////////////////////////////////////////////////////////////////////
ADIS16490::~ADIS16490() {
  // Close SPI bus
  SPI.end();
}

////////////////////////////////////////////////////////////////////////////
// Performs a hardware reset by setting _RST pin low for delay (in ms).
////////////////////////////////////////////////////////////////////////////
int ADIS16490::resetDUT(uint8_t ms) {
  digitalWrite(_RST, LOW);
  delayMicroseconds(500);
  digitalWrite(_RST, HIGH);
  delay(ms);
  return(1);
}

////////////////////////////////////////////////////////////////////////////
// Sets SPI bit order, clock divider, and data mode. This function is useful
// when there are multiple SPI devices using different settings.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
int ADIS16490::configSPI() {
  SPISettings IMUSettings(2000000, MSBFIRST, SPI_MODE3);
  SPI.beginTransaction(IMUSettings);
  return(1);
}

////////////////////////////////////////////////////////////////////////////////////////////
// Reads two bytes (one word) in two sequential registers over SPI
////////////////////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be read
// return - (int) signed 16 bit 2's complement number
////////////////////////////////////////////////////////////////////////////////////////////
int16_t ADIS16490::regRead(uint16_t regAddr) {
  // Separate page ID from register address
  uint8_t page = ((regAddr >> 8) & 0xFF);
  uint8_t address = (regAddr & 0xFF);

  // Check whether the sensor is currently on the requested page
  if (currentPage != page) {
    // Write desired page to PAGE_ID register
    digitalWrite(_CS, LOW); // Set CS low to enable device
    SPI.transfer(0x80); // Write high byte from low word to SPI bus
    SPI.transfer(page); // Write low byte from low word to SPI bus
    digitalWrite(_CS, HIGH); // Set CS high to disable device
    // Write new current page to tracking variable
    currentPage = page; 
    delayMicroseconds(_stall); // Stall time delay
  }

  // Write desired register address
  digitalWrite(_CS, LOW); // Set CS low to enable device
  SPI.transfer(address); // Write address over SPI bus
  SPI.transfer(0x00); // Write 0x00 to the SPI bus fill the 16 bit transaction requirement
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(_stall); // Stall time delay

  // Read data from requested register
  digitalWrite(_CS, LOW); // Set CS low to enable device
  uint16_t _dataOut = (SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF); // Concatenate upper and lower bytes
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(_stall); // Stall time delay

  // Shift MSB data left by 8 bits, mask LSB data with 0xFF, and OR both bits.

  return(_dataOut);
}

////////////////////////////////////////////////////////////////////////////
// Writes one byte of data to the specified register over SPI.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be written
// regData - data to be written to the register
////////////////////////////////////////////////////////////////////////////
int ADIS16490::regWrite(uint16_t regAddr, int16_t regData) {
  // Separate page ID from register address
  uint8_t page = ((regAddr >> 8) & 0xFF);
  uint8_t address = (regAddr & 0xFF);

  // Check whether the sensor is currently on the requested page
  if (currentPage != page) {
    // Write desired page to PAGE_ID register
    digitalWrite(_CS, LOW); // Set CS low to enable device
    SPI.transfer(0x80); // Write high byte from low word to SPI bus
    SPI.transfer(page); // Write low byte from low word to SPI bus
    digitalWrite(_CS, HIGH); // Set CS high to disable device
    // Write new current page to tracking variable
    currentPage = page; 
    delayMicroseconds(_stall); // Stall time delay
  }

  // Sanity-check address and register data
  uint16_t addr = (((address & 0x7F) | 0x80) << 8); // Toggle sign bit, and check that the address is 8 bits
  uint16_t lowWord = (addr | (regData & 0xFF)); // OR Register address (A) with data(D) (AADD)
  uint16_t highWord = ((addr | 0x100) | ((regData >> 8) & 0xFF)); // OR Register address with data and increment address

  // Write highWord to SPI bus
  digitalWrite(_CS, LOW); // Set CS low to enable device
  SPI.transfer(lowWord >> 8); // Write high byte from low word to SPI bus
  SPI.transfer(lowWord & 0xFF); // Write low byte from low word to SPI bus
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(_stall); // Stall time delay

  // Write lowWord to SPI bus
  digitalWrite(_CS, LOW); // Set CS low to enable device
  SPI.transfer(highWord >> 8); // Write high byte from high word to SPI bus
  SPI.transfer(highWord & 0xFF); // Write low byte from high word to SPI bus
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(_stall); // Stall time delay

  return(1);
}

////////////////////////////////////////////////////////////////////////////
// Reads a fixed set of registers from the sensor.
// Returns a pointer to an array of sensor data. 
////////////////////////////////////////////////////////////////////////////
// No inputs required.
////////////////////////////////////////////////////////////////////////////
int16_t *ADIS16490::sensorRead(void) {
  // Set up temporary variables
  uint8_t sensorbyte[18];
  static int16_t sensorwords[9];

  // Check whether the sensor is currently on the requested page
  if (currentPage != 0x00) {
    // Write desired page to PAGE_ID register
    digitalWrite(_CS, LOW); // Set CS low to enable device
    SPI.transfer(0x80); // Write high byte from low word to SPI bus
    SPI.transfer(0x00); // Write low byte from low word to SPI bus
    digitalWrite(_CS, HIGH); // Set CS high to disable device
    // Write new current page to tracking variable
    currentPage = 0x00; 
    delayMicroseconds(10); // Stall time delay
  }

  // Write initial register address and discard erroneous data
  digitalWrite(_CS, LOW); // Set CS low to enable device
  SPI.transfer(DIAG_STS); // Write address over SPI bus
  SPI.transfer(0x00); // Write 0x00 to the SPI bus to complete word
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(10); // Stall time delay

  // Read data from requested register and transfer the next address in the same frame
  digitalWrite(_CS, LOW); // Set CS low to enable device
  sensorbyte[0] = SPI.transfer(ALM_STS); // Send next address and place DIAG_STS MSB into variable
  sensorbyte[1] = SPI.transfer(0x00); // Complete word and place DIAG_STS LSB into variable
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(10); // Stall time delay

  // Read data from requested register and transfer the next address in the same frame
  digitalWrite(_CS, LOW); // Set CS low to enable device
  sensorbyte[2] = SPI.transfer(X_GYRO_OUT); // Send next address and place ALM_STS MSB into variable
  sensorbyte[3] = SPI.transfer(0x00); // Complete word and place ALM_STS LSB into variable
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(10); // Stall time delay

  // Read data from requested register and transfer the next address in the same frame
  digitalWrite(_CS, LOW); // Set CS low to enable device
  sensorbyte[4] = SPI.transfer(Y_GYRO_OUT); // Send next address and place X_GYRO_OUT MSB into variable
  sensorbyte[5] = SPI.transfer(0x00); // Complete word and place X_GYRO_OUT LSB into variable
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(10); // Stall time delay

  // Read data from requested register and transfer the next address in the same frame
  digitalWrite(_CS, LOW); // Set CS low to enable device
  sensorbyte[6] = SPI.transfer(Z_GYRO_OUT); // Send next address and place Y_GYRO_OUT MSB into variable
  sensorbyte[7] = SPI.transfer(0x00); // Complete word and place Y_GYRO_OUT LSB into variable
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(10); // Stall time delay

  // Read data from requested register and transfer the next address in the same frame
  digitalWrite(_CS, LOW); // Set CS low to enable device
  sensorbyte[8] = SPI.transfer(X_ACCL_OUT); // Send next address and place Z_GYRO_OUT MSB into variable
  sensorbyte[9] = SPI.transfer(0x00); // Complete word and place Z_GYRO_OUT LSB into variable
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(10); // Stall time delay

  // Read data from requested register and transfer the next address in the same frame
  digitalWrite(_CS, LOW); // Set CS low to enable device
  sensorbyte[10] = SPI.transfer(Y_ACCL_OUT); // Send next address and place X_ACCL_OUT MSB into variable
  sensorbyte[11] = SPI.transfer(0x00); // Complete word and place X_ACCL_OUT LSB into variable
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(10); // Stall time delay

  // Read data from requested register and transfer the next address in the same frame
  digitalWrite(_CS, LOW); // Set CS low to enable device
  sensorbyte[12] = SPI.transfer(Z_ACCL_OUT); // Send next address and place Y_ACCL_OUT MSB into variable
  sensorbyte[13] = SPI.transfer(0x00); // Complete word and place Y_ACCL_OUT LSB into variable
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(10); // Stall time delay

  // Read data from requested register and transfer the next address in the same frame
  digitalWrite(_CS, LOW); // Set CS low to enable device
  sensorbyte[14] = SPI.transfer(TEMP_OUT); // Send next address and place Z_ACCL_OUT MSB into variable
  sensorbyte[15] = SPI.transfer(0x00); // Complete word and place Z_ACCL_OUT LSB into variable
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(10); // Stall time delay

  // Read data from requested register and transfer the next address in the same frame
  digitalWrite(_CS, LOW); // Set CS low to enable device
  sensorbyte[16] = SPI.transfer(0x00); // Send dummy address and place TEMP_OUT MSB into variable
  sensorbyte[17] = SPI.transfer(0x00); // Complete word and place TEMP_OUT LSB into variable
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(10); // Stall time delay

  // Join bytes into words
  sensorwords[0] = ((sensorbyte[0] << 8) | (sensorbyte[1] & 0xFF)); //DIAG_STS
  sensorwords[1] = ((sensorbyte[2] << 8) | (sensorbyte[3] & 0xFF)); //ALM_STS
  sensorwords[2] = ((sensorbyte[4] << 8) | (sensorbyte[5] & 0xFF)); //XGYRO
  sensorwords[3] = ((sensorbyte[6] << 8) | (sensorbyte[7] & 0xFF)); //YGYRO
  sensorwords[4] = ((sensorbyte[8] << 8) | (sensorbyte[9] & 0xFF)); //ZGYRO
  sensorwords[5] = ((sensorbyte[10] << 8) | (sensorbyte[11] & 0xFF)); //XACCEL
  sensorwords[6] = ((sensorbyte[12] << 8) | (sensorbyte[13] & 0xFF)); //YACCEL
  sensorwords[7] = ((sensorbyte[14] << 8) | (sensorbyte[15] & 0xFF)); //ZACCEL
  sensorwords[8] = ((sensorbyte[16] << 8) | (sensorbyte[17] & 0xFF)); //TEMP

  return sensorwords;

}

/////////////////////////////////////////////////////////////////////////////////////////
// Converts accelerometer data output from the regRead() function and returns
// acceleration in mg's
/////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled accelerometer in mg's
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16490::accelScale(int16_t sensorData)
{
  float finalData = sensorData * 0.5; // mg/LSB
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts gyro data output from the regRead() function and returns gyro rate in deg/sec
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled gyro in degrees/sec
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16490::gyroScale(int16_t sensorData)
{
  float finalData = sensorData * 0.005; //degrees/sec/LSB
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts temperature data output from the regRead() function and returns temperature 
// in degrees Celcius
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled temperature in degrees Celcius
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16490::tempScale(int16_t sensorData)
{
  float finalData = (sensorData * 0.01429) + 25; // degrees C/LSB
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts integrated angle data output from the regRead() function and returns delta angle in degrees
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled delta angle in degrees
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16490::deltaAngleScale(int16_t sensorData)
{
  float finalData = sensorData * 0.022; // degrees/LSB
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts integrated velocity data output from the regRead() function and returns delta velocity in mm/sec
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled delta velocity in mm/sec
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16490::deltaVelocityScale(int16_t sensorData)
{
  float finalData = sensorData * 6.104; // mm/sec/LSB
  return finalData;
}
