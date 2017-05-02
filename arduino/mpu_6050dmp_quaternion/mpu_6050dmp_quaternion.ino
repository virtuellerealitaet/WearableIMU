//
// Sketch : Sensing orientation using MPU-6050/DMP
// 
// Michael Stengel <virtuellerealitaet@gmail.com>
// based on code of Jeff Rowberg's icddevlib <jeff@rowberg.net>, https://github.com/jrowberg/i2cdevlib
//
// adjust the DMP interrupt frequency in MPU6050_6Axis_MotionApps20.h
// https://www.i2cdevlib.com/forums/topic/12-a-question-to-change-the-sample-rate-in-mpu6050h-file/
// last byte in : const unsigned char dmpConfig[MPU6050_DMP_CONFIG_SIZE] PROGMEM
//
// 0x00 -> 200 Hz (noisy data)
// 0x01 -> 100 Hz (stable)
// 0x03 -> 50 Hz
//
// tested platforms : Arduino Uno, Arduino Mini, Teensy 3.6
//
// -------------------------------------
//
// WIRING SCHEME :  Arduino
//
// MPU-6050     Arduino
//   VCC   ---   3.3V
//   GND   ---   GND
//   SCL   ---   A5
//   SDA   ---   A4
//   INT   ---   DIGITAL PIN 2
//
// -------------------------------------
//
// WIRING SCHEME :  Teensy 3.x
//
// MPU-6050     Teensy 3.x
//   VCC   ---   3.3V
//   GND   ---   GND
//   SCL   ---   PIN 19 (A5)
//   SDA   ---   PIN 18 (A4)
//   INT   ---   PIN 2 (PWM)

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

#define INT_PIN 2

//#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
//bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion variables
Quaternion q;
//VectorFloat gravity;
//float euler[3];
//float ypr[3];

typedef union {
 float floatingPoint[4];
 byte binary[16];
} binaryFloat;


/*

// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;  
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
float         last_gyro_y_angle;
float         last_gyro_z_angle;

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}

inline unsigned long get_last_time() {return last_read_time;}
inline float get_last_x_angle() {return last_x_angle;}
inline float get_last_y_angle() {return last_y_angle;}
inline float get_last_z_angle() {return last_z_angle;}
inline float get_last_gyro_x_angle() {return last_gyro_x_angle;}
inline float get_last_gyro_y_angle() {return last_gyro_y_angle;}
inline float get_last_gyro_z_angle() {return last_gyro_z_angle;}



//  Use the following global variables 
//  to calibrate the gyroscope sensor and accelerometer readings
float    base_x_gyro = 0;
float    base_y_gyro = 0;
float    base_z_gyro = 0;
float    base_x_accel = 0;
float    base_y_accel = 0;
float    base_z_accel = 0;


// This global variable tells us how to scale gyroscope data
float    GYRO_FACTOR;

// This global varible tells how to scale acclerometer data
float    ACCEL_FACTOR;



// Variables to store the values from the sensor readings
int16_t ax, ay, az;
int16_t gx, gy, gz;

*/

// Buffer for data output
char dataOut[256];
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


/*

// ================================================================
// ===                CALIBRATION_ROUTINE                       ===
// ================================================================
// Simple calibration - just average first few readings to subtract
// from the later data
void calibrate_sensors() {
  int       num_readings = 10;

  // Discard the first reading (don't know if this is needed or
  // not, however, it won't hurt.)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Read and average the raw values
  for (int i = 0; i < num_readings; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    base_x_gyro += gx;
    base_y_gyro += gy;
    base_z_gyro += gz;
    base_x_accel += ax;
    base_y_accel += ay;
    base_y_accel += az;
  }
  
  base_x_gyro /= num_readings;
  base_y_gyro /= num_readings;
  base_z_gyro /= num_readings;
  base_x_accel /= num_readings;
  base_y_accel /= num_readings;
  base_z_accel /= num_readings;
}

*/


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    //
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.
    //
    Serial.begin(57600);
    while (!Serial);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection ..."));
        attachInterrupt(INT_PIN, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // Set the full scale range of the gyro
        //uint8_t FS_SEL = 0;
        //mpu.setFullScaleGyroRange(FS_SEL);

        // get default full scale value of gyro - may have changed from default
        // function call returns values between 0 and 3
        //uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();
        //Serial.print("FS_SEL = ");
        //Serial.println(READ_FS_SEL);
        //GYRO_FACTOR = 131.0/(FS_SEL + 1);
        

        // get default full scale value of accelerometer - may not be default value.  
        // Accelerometer scale factor doesn't reall matter as it divides out
        //uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange();
        //Serial.print("AFS_SEL = ");
        //Serial.println(READ_AFS_SEL);
        //ACCEL_FACTOR = 16384.0/(AFS_SEL + 1);
        
        // Set the full scale range of the accelerometer
        //uint8_t AFS_SEL = 0;
        //mpu.setFullScaleAccelRange(AFS_SEL);

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    //pinMode(LED_PIN, OUTPUT);
    
    // get calibration values for sensors
    //calibrate_sensors();
    //set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    //const float RADIANS_TO_DEGREES = 57.2958; //180/3.14159
 
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    /*
    unsigned long t_now = millis();

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        
        // Keep calculating the values of the complementary filter angles for comparison with DMP here
        // Read the raw accel/gyro values from the MPU-6050
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
              
        // Get time of last raw data read
        unsigned long t_now = millis();
          
        // Remove offsets and scale gyro data  
        float gyro_x = (gx - base_x_gyro)/GYRO_FACTOR;
        float gyro_y = (gy - base_y_gyro)/GYRO_FACTOR;
        float gyro_z = (gz - base_z_gyro)/GYRO_FACTOR;
        float accel_x = ax; // - base_x_accel;
        float accel_y = ay; // - base_y_accel;
        float accel_z = az; // - base_z_accel;
        
              
        float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_z = 0;

        // Compute the (filtered) gyro angles
        float dt =(t_now - get_last_time())/1000.0;
        float gyro_angle_x = gyro_x*dt + get_last_x_angle();
        float gyro_angle_y = gyro_y*dt + get_last_y_angle();
        float gyro_angle_z = gyro_z*dt + get_last_z_angle();
        
        // Compute the drifting gyro angles
        float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
        float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
        float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();     
        
        // Apply the complementary filter to figure out the change in angle - choice of alpha is
        // estimated now.  Alpha depends on the sampling rate...
        const float alpha = 0.96;
        float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
        float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
        float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle
        
        // Update the saved data with the latest values
        set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);       
    }
    */

    if (mpuInterrupt || fifoCount >= packetSize)
    {
        // reset interrupt flag and get INT_STATUS byte
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
    
        // get current FIFO count
        fifoCount = mpu.getFIFOCount();
    
        // check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            Serial.println(F("FIFO overflow!"));
    
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & 0x02) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;
            
            // Obtain Euler angles from buffer
            //mpu.dmpGetQuaternion(&q, fifoBuffer);
            //mpu.dmpGetEuler(euler, &q);
            
            // Obtain YPR angles from buffer
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            //mpu.dmpGetGravity(&gravity, &q);
            //mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
           // Output complementary data and DMP data to the serial port.  The signs on the data needed to be
           // fudged to get the angle direction correct.
           //Serial.print("CMP:");
           //Serial.print(get_last_x_angle(), 2);
           //Serial.print(":");
           //Serial.print(get_last_y_angle(), 2);
           //Serial.print(":");
           //Serial.println(-get_last_z_angle(), 2);
           //Serial.print("DMP:");
           //Serial.print(ypr[2]*RADIANS_TO_DEGREES, 2);
           //Serial.print(":");
           //Serial.print(-ypr[1]*RADIANS_TO_DEGREES, 2);
           //Serial.print(":");
           //Serial.println(ypr[0]*RADIANS_TO_DEGREES, 2);
    
           //Serial.print(q.x, 2);
           //Serial.print(":");
           //Serial.print(q.y, 2);
           //Serial.print(":");
          // Serial.print(q.z, 2);
           //Serial.print(":");
           //Serial.println(q.w, 2);
          
           binaryFloat hi;

           hi.floatingPoint[0] = q.x;
           hi.floatingPoint[1] = q.y;
           hi.floatingPoint[2] = q.z;
           hi.floatingPoint[3] = q.w;
           Serial.write(hi.binary,16);
           
            // blink LED to indicate activity
            //blinkState = !blinkState;
            //digitalWrite(LED_PIN, blinkState);
        }
    }
}
