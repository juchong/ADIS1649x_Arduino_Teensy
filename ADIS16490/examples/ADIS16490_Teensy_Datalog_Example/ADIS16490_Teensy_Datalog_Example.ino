////////////////////////////////////////////////////////////////////////////////////////////////////////
//  October 2016
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16490_Teensy_Datalog_Example.ino
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This Arduino project interfaces with an ADIS16490 using SPI and the 
//  accompanying C++ libraries, reads IMU data in LSBs, scales the data, and 
//  outputs measurements to a serial debug terminal (PuTTY) via the onboard 
//  USB serial port.
//
//  This project has been tested on a PJRC 32-Bit Teensy 3.2 Development Board, 
//  but should be compatible with any other embedded platform with some modification.
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
//  Pinout for a Teensy 3.2 Development Board
//  RST = D6
//  SCK = D13/SCK
//  CS = D10/CS
//  DOUT(MISO) = D12/MISO
//  DIN(MOSI) = D11/MOSI
//  DR = D2
//
////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <ADIS16490.h>
#include <SPI.h>

// Temporary Data Arrays
int16_t *sensorData;
String separator = ",";

// Call ADIS16490 Class
ADIS16490 IMU(10,2,6); // Chip Select, Data Ready, Reset Pin Assignments

void setup()
{
    Serial.begin(9600); // Initialize serial output via USB
    IMU.configSPI(); // Configure SPI communication
    delay(1000); // Give the part time to start up
    IMU.regWrite(FNCTIO_CTRL, 0x0C);  // Enable Data Ready, set polarity
    delay(20); 
    IMU.regWrite(DEC_RATE, 0x17), // Disable decimation
    delay(20);

    // Configure SPI settings for IMU
    IMU.configSPI();

    // Attach interrupt to pin 2. Trigger on the rising edge
    attachInterrupt(2, grabData, RISING); 
}

// Function used to read register values when an ISR is triggered using the IMU's DataReady output
void grabData()
{
    sensorData = IMU.sensorRead();
    // Print burst data to serial port. Data output rate is determined by the IMU decimation rate
    String data = ((*(sensorData + 2)) + separator + (*(sensorData + 3)) + separator + (*(sensorData + 4)) + separator + (*(sensorData + 5)) + separator + (*(sensorData + 6)) + separator + (*(sensorData + 7)) + separator + (*(sensorData + 8)));
    Serial.println(data); 
}

// Main loop
void loop()
{
    // Nothing to do here! The program is entirely interrupt-driven!
}
