////////////////////////////////////////////////////////////////////////////////////////////////////////
//  April 2017
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16490_Teensy_Example.ino
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

// Uncomment to enable debug
//#define DEBUG

// Temporary Data Arrays
int16_t sensorData[9];
float scaledData[7];

// Control registers
int MSC = 0;
int DECR = 0;

// Delay counter variable
int printCounter = 0;

// Call ADIS16490 Class
ADIS16490 IMU(10,2,6); // Chip Select, Data Ready, Reset Pin Assignments

void setup()
{
    Serial.begin(115200); // Initialize serial output via USB
    IMU.configSPI(); // Configure SPI communication
    delay(1000); // Give the part time to start up
    IMU.regWrite(FNCTIO_CTRL, 0x0C);  // Enable Data Ready, set polarity
    delay(20); 
    IMU.regWrite(DEC_RATE, 0x00), // Disable decimation
    delay(20);

    // Read the control registers once to print to screen
    MSC = IMU.regRead(FNCTIO_CTRL);
    DECR = IMU.regRead(DEC_RATE);

    // Configure SPI settings for IMU
    IMU.configSPI();

    // Attach interrupt to pin 2. Trigger on the rising edge
    attachInterrupt(2, grabData, RISING); 
}

// Function used to read register values when an ISR is triggered using the IMU's DataReady output
void grabData()
{
    sensorData[0] = IMU.regRead(DIAG_STS); 
    sensorData[1] = IMU.regRead(ALM_STS); 
    sensorData[2] = IMU.regRead(X_GYRO_OUT); 
    sensorData[3] = IMU.regRead(Y_GYRO_OUT); 
    sensorData[4] = IMU.regRead(Z_GYRO_OUT); 
    sensorData[5] = IMU.regRead(X_ACCL_OUT); 
    sensorData[6] = IMU.regRead(Y_ACCL_OUT); 
    sensorData[7] = IMU.regRead(Z_ACCL_OUT); 
    sensorData[8] = IMU.regRead(TEMP_OUT);
}

// Function used to scale all acquired data (scaling functions are included in ADIS16490.cpp)
void scaleData()
{
    scaledData[0] = IMU.gyroScale(sensorData[2]);   // XGYRO
    scaledData[1] = IMU.gyroScale(sensorData[3]);   // YGYRO
    scaledData[2] = IMU.gyroScale(sensorData[4]);   // ZGYRO
    scaledData[3] = IMU.accelScale(sensorData[5]);  // XACCL
    scaledData[4] = IMU.accelScale(sensorData[6]);  // YACCL
    scaledData[5] = IMU.accelScale(sensorData[7]);  // ZACCL
    scaledData[6] = IMU.tempScale(sensorData[8]);   // TEMP
}

// Main loop. Print data to the serial port. Sensor sampling is performed in the ISR
void loop()
{
    printCounter ++;
    if (printCounter >= 200000) // Delay for writing data to the serial port
    {
        detachInterrupt(2); //Detach interrupt to avoid overwriting data
        scaleData(); // Scale data acquired from the IMU

        //Clear the serial terminal and reset cursor
        //Only works on supported serial terminal programs (Putty)
        Serial.print("\033[2J");
        Serial.print("\033[H");

        // Print header
        Serial.println(" ");
        Serial.println("ADIS16490 Teensy Example Program");
        Serial.println("Juan Chong - October 2016");
        Serial.println(" ");

        // Print control registers to the serial port
        Serial.println("Control Registers");
        Serial.print("FNCTIO_CTRL: ");
        Serial.println(MSC,HEX);
        Serial.print("DEC_RATE: ");
        Serial.println(DECR,HEX);
        Serial.println(" ");
        Serial.println("Primary Output Registers");
        
        // Print scaled gyro data
        Serial.print("XGYRO: ");
        Serial.println(scaledData[0]);
        Serial.print("YGYRO: ");
        Serial.println(scaledData[1]);
        Serial.print("ZGYRO: ");
        Serial.println(scaledData[2]);
      
        // Print scaled accel data
        Serial.print("XACCL: ");
        Serial.println(scaledData[3]);
        Serial.print("YACCL: ");
        Serial.println(scaledData[4]);
        Serial.print("ZACCL: ");
        Serial.println(scaledData[5]);
        Serial.println(" ");

        Serial.println("Status Registers");

        // Print Status Registers
        Serial.print("DIAG_STS: ");
        Serial.println(sensorData[0]);
        Serial.print("ALM_STS: ");
        Serial.println(sensorData[1]);
       
        // Print scaled temp data
        Serial.print("TEMP: ");
        Serial.println(scaledData[6]);

#ifdef DEBUG 
        // Print unscaled gyro data
        Serial.print("XGYRO: ");
        Serial.println(sensorData[2]);
        Serial.print("YGYRO: ");
        Serial.println(sensorData[3]);
        Serial.print("ZGYRO: ");
        Serial.println(sensorData[4]);
      
        // Print unscaled accel data
        Serial.print("XACCL: ");
        Serial.println(sensorData[5]);
        Serial.print("YACCL: ");
        Serial.println(sensorData[6]);
        Serial.print("ZACCL: ");
        Serial.println(sensorData[7]);
        Serial.println(" ");
       
        // Print unscaled temp data
        Serial.print("TEMP: ");
        Serial.println(sensorData[8]);
#endif
        printCounter = 0;
        attachInterrupt(2, grabData, RISING);
    }
}
