// 2 servo planar stabilization system
// wp
// Jan 2016
//
// Based on Jeff Rowber's work found at
// https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.cpp
//
// Use at your own risk.
//
// This code is placed under the MIT License (MIT)
//
// Copyright (c) 2016 woojay poynter

//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.
//

// Servo Connection
// BROWN - gnd
// red - 5v
// yellow - d10 (pwm on Sero 1)
//        - d11 (servo 2)

// MPU Connection
//
// VCC - 5v
// GND - GND
// SCL - A5 (w/ 10k PuR)
// SDA - A4 (w/ 10k PuR)
// INT - D2 (not used)

#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define LED_PIN 13
bool blinkState = true;

Servo Servo1;   // First Servo off the chassis
Servo Servo2;   // Second Servo off the chassis

int Servo1Pos = 0;
int Servo2Pos = 0;

byte qua[4];
float r[3][3];
float ang[3];
byte euler[3];

float mpuPitch = 0;
float mpuRoll = 0;
float mpuYaw = 0;


// define MPU instance
MPU6050 mpu;                    // class default I2C address is 0x68; specific I2C addresses may be passed as a parameter here

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// relative ypr[x] usage based on sensor orientation when mounted, e.g. ypr[PITCH]
#define PITCH   1     // defines the position within ypr[x] variable for PITCH; may vary due to sensor orientation when mounted
#define ROLL  2     // defines the position within ypr[x] variable for ROLL; may vary due to sensor orientation when mounted
#define YAW   0     // defines the position within ypr[x] variable for YAW; may vary due to sensor orientation when mounted

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{

  Servo1.attach(10);  // attaches the servo on D11 to the servo object
  Servo2.attach(11);  // Second servo on D11
  delay(50);
  Servo1.write(0);  // These are command checks to see if the servos work and
  Servo2.write(60); // to help w/ the initial installation.
  delay(500);   // Make sure these movements are clear from the rest of the chassis.
  Servo1.write(180);
  Servo2.write(120);
  delay(500);
  Servo1.write(0);
  Servo2.write(90);
  delay(500);

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(9600);
  while (!Serial);      // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
//  Serial.println(F("Testing device connections..."));
//  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
//  Serial.println(F("Initializing DMP"));
  devStatus = mpu.dmpInitialize();


  // INPUT CALIBRATED OFFSETS HERE; SPECIFIC FOR EACH UNIT AND EACH MOUNTING CONFIGURATION!!!!

  mpu.setXGyroOffset(118);
  mpu.setYGyroOffset(-44);
  mpu.setZGyroOffset(337);
  mpu.setXAccelOffset(-651);
  mpu.setYAccelOffset(670);
  mpu.setZAccelOffset(1895);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
//    Serial.println(F("Enabling DMP"));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
//    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)"));
    mpuIntStatus = mpu.getIntStatus();

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed, 2 = DMP configuration updates failed (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed code = "));
    Serial.println(devStatus);
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

} // setup()



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop(void)
{
  processAccelGyro();
}   // loop()

void serialEvent() {
  while (Serial.available()) {
    if(q.w >= 0) qua[0] = (byte)(q.w*255.0);
    else qua[0] = (byte)((q.w + 1.0)*255.0/2.0);
    if(q.x >= 0) qua[1] = (byte)(q.x*255.0);
    else qua[1] = (byte)((q.x + 1.0)*255.0/2.0);
    if(q.y >= 0) qua[2] = (byte)(q.y*255.0);
    else qua[2] = (byte)((q.y + 1.0)*255.0/2.0);
    if(q.z >= 0) qua[3] = (byte)(q.z*255.0);
    else qua[3] = (byte)((q.z + 1.0)*255.0/2.0);
    char inChar = (char)Serial.read();
    Serial.print(qua[0]);
    Serial.print(" ");
    Serial.print(qua[1]);
    Serial.print(" ");
    Serial.print(qua[2]);
    Serial.print(" ");
    Serial.println(qua[3]);

//Não funfa
//    r[0][0] = 2.0*q.w*q.w - 1.0 + 2.0*q.x*q.x;
//    r[1][0] = 2.0*(q.x*q.y - q.x*q.z);
//    r[2][0] = 2.0*(q.x*q.z + q.w*q.y);
//    r[2][1] = 2.0*(q.y*q.z + q.w*q.x);
//    r[2][2] = 2.0*q.w*q.w - 1.0 + 2.0*q.z*q.z;
//    ang[0] = atan2(r[2][1],r[2][2]);
//    ang[1] = atan(r[2][0]/(sqrt(1.0 - r[2][0])*sqrt(1.0 - r[2][0])));
//    ang[2] = atan2(r[1][0],r[0][0]);
//    if(ang[0] >= 0.00) euler[0] = (byte)(ang[0]*255.0/90.0);
//    else euler[0] = (byte)((ang[0] + 90.0)*255.0/180.0);
//    if(ang[1] >= 0.00) euler[1] = (byte)(ang[1]*255.0/90.0);
//    else euler[1] = (byte)((ang[1] + 90.0)*255.0/180.0);
//    if(ang[2] >= 0.00) euler[2] = (byte)(ang[2]*255.0/90.0);
//    else euler[2] = (byte)((ang[2] + 90.0)*255.0/180.0);
//    Serial.print(euler[0]);
//    Serial.print(" ");
//    Serial.print(euler[1]);
//    Serial.print(" ");
//    Serial.println(euler[2]);
  }
}

// ================================================================
// ===                    PROCESS ACCEL/GYRO IF AVAILABLE       ===
// ================================================================

void processAccelGyro()
{

  // Get INT_STATUS byte
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return;
  }

  if (mpuIntStatus & 0x02)  // otherwise continue processing
  {
    // check for correct available data length
    if (fifoCount < packetSize)
      return; //  fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    fifoCount -= packetSize;

    // flush buffer to prevent overflow
    mpu.resetFIFO();

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpuRoll = ypr[ROLL] * 180 / M_PI;
    mpuPitch = ypr[PITCH] * 180 / M_PI;
    mpuYaw  = ypr[YAW] * 180 / M_PI;

    // flush buffer to prevent overflow
    mpu.resetFIFO();

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    // flush buffer to prevent overflow
    mpu.resetFIFO();

    Servo1.write(-mpuPitch + 90);
    Servo2.write(mpuRoll + 90);
//    Serial.println(-mpuPitch + 90);
//    Serial.println(mpuRoll + 90);
    //delay(10);

    // flush buffer to prevent overflow
    mpu.resetFIFO();

  } // if (mpuIntStatus & 0x02)
}  // processAccelGyro()
