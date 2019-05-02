/*
  DIY Gimbal - MPU6050 Arduino Tutorial
  by Dejan, www.HowToMechatronics.com
  Code based on the MPU6050_DMP6 example from the i2cdevlib library by Jeff Rowberg:
  https://github.com/jrowberg/i2cdevlib
*/
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <Servo.h>
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu1;
MPU6050 mpu2(0x69); // <-- use for AD0 high

// Define the 3 servo motors
Servo servo0;
Servo servo1;
Servo servo2;
float correct;
int j = 0;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN1 2  // use pin 2 on Arduino Uno & most boards
#define INTERRUPT_PIN2 3
bool blinkState = false;

// MPU control/status vars
bool dmpReady1 = false;  // set true if DMP init was successful
uint8_t mpuIntStatus1;
uint8_t mpuIntStatus2;// holds actual interrupt status byte from MPU
uint8_t devStatus1;// return status after each device operation (0 = success, !0 = error)
uint8_t devStatus2;
uint16_t packetSize1;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount1;     // count of all bytes currently in FIFO
uint8_t fifoBuffer1[64]; // FIFO storage buffer

bool dmpReady2 = false;  // set true if DMP init was successful
uint16_t packetSize2;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount2;     // count of all bytes currently in FIFO
uint8_t fifoBuffer2[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q1;           // [w, x, y, z]         quaternion container
VectorInt16 aa1;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal1;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld1;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity1;    // [x, y, z]            gravity vector
float euler1[3];         // [psi, theta, phi]    Euler angle container
float ypr1[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// packet structure for InvenSense teapot demo
uint8_t teapotPacket1[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

Quaternion q2;           // [w, x, y, z]         quaternion container
VectorInt16 aa2;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal2;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld2;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity2;    // [x, y, z]            gravity vector
float euler2[3];         // [psi, theta, phi]    Euler angle container
float ypr2[3];
// packet structure for InvenSense teapot demo
uint8_t teapotPacket2[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt1 = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady1() {
  mpuInterrupt1 = true;
}

volatile bool mpuInterrupt2 = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady2() {
  mpuInterrupt2 = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================



void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(9600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  //Serial.println(F("Initializing I2C devices..."));
  mpu1.initialize();
  mpu2.initialize();
  pinMode(INTERRUPT_PIN1, INPUT);
  pinMode(INTERRUPT_PIN2, INPUT);
  pinMode(5,INPUT);
  devStatus1 = mpu1.dmpInitialize();
  devStatus2 = mpu2.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu1.setXGyroOffset(17);
  mpu1.setYGyroOffset(-69);
  mpu1.setZGyroOffset(27);
  mpu1.setZAccelOffset(1551); // 1688 factory default for my test chip

  mpu2.setXGyroOffset(17);
  mpu2.setYGyroOffset(-69);
  mpu2.setZGyroOffset(27);
  mpu2.setZAccelOffset(1551);
  // make sure it worked (returns 0 if so)
  if (devStatus1 == 0 && devStatus2 == 0) {
    // turn on the DMP, now that it's ready
    // Serial.println(F("Enabling DMP..."));
    mpu1.setDMPEnabled(true);
    mpu2.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN1), dmpDataReady1, RISING);
    mpuIntStatus1 = mpu1.getIntStatus();
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN2), dmpDataReady2, RISING);
    mpuIntStatus2 = mpu2.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady1 = true;
    dmpReady2 = true;
    // get expected DMP packet size for later comparison
    packetSize1 = mpu1.dmpGetFIFOPacketSize();
    packetSize2 = mpu2.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    // Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }

  // Define the pins to which the 3 servo motors are connected
  servo0.attach(10);
  servo1.attach(9);
  servo2.attach(8);
}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
int cal = -1;
int correction_yaw = 0;
int correction_pitch = 0;
int sum_yaw = 0;
int sum_pitch = 0;
void loop() {
  // if programming failed, don't try to do anything
  if(cal>=0) cal--;
  else {sum_yaw = 0; sum_pitch = 0;}
  if (!dmpReady1 || !dmpReady2) return;

  // wait for MPU interrupt or extra packet(s) available
  while ((!mpuInterrupt1 && fifoCount1 < packetSize1) && (!mpuInterrupt2 && fifoCount2 < packetSize2)) {
    if ((mpuInterrupt1 && fifoCount1 < packetSize1)&&(mpuInterrupt2 && fifoCount2 < packetSize2)) {
      // try to get out of the infinite loop
      fifoCount1 = mpu1.getFIFOCount();
      fifoCount2 = mpu2.getFIFOCount();
    }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt1 = false;
  mpuIntStatus1 = mpu1.getIntStatus();

  mpuInterrupt2 = false;
  mpuIntStatus2 = mpu2.getIntStatus();

  // get current FIFO count
  fifoCount1 = mpu1.getFIFOCount();
  fifoCount2 = mpu2.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus1 & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount1 >= 1024) {
    // reset so we can continue cleanly
    mpu1.resetFIFO();
    fifoCount1 = mpu1.getFIFOCount();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  if ((mpuIntStatus2 & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount2 >= 1024) {
    // reset so we can continue cleanly
    mpu2.resetFIFO();
    fifoCount2 = mpu2.getFIFOCount();
    Serial.println(F("FIFO2 overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if ((mpuIntStatus1 & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))&&(mpuIntStatus1 & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))) {
    // wait for correct available data length, should be a VERY short wait
    if(digitalRead(5) == HIGH){cal=50; Serial.println("callibrating...");}
    while (fifoCount1 < packetSize1) fifoCount1 = mpu1.getFIFOCount();

    // read a packet from FIFO
    mpu1.getFIFOBytes(fifoBuffer1, packetSize1);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount1 -= packetSize1;

    while (fifoCount2 < packetSize2) fifoCount2 = mpu2.getFIFOCount();

    // read a packet from FIFO
    mpu2.getFIFOBytes(fifoBuffer2, packetSize2);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount2 -= packetSize2;
    // Get Yaw, Pitch and Roll values
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu1.dmpGetQuaternion(&q1, fifoBuffer1);
    mpu1.dmpGetGravity(&gravity1, &q1);
    mpu1.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);

    mpu2.dmpGetQuaternion(&q2, fifoBuffer2);
    mpu2.dmpGetGravity(&gravity2, &q2);
    mpu2.dmpGetYawPitchRoll(ypr2, &q2, &gravity2);
    // Yaw, Pitch, Roll values - Radians to degrees
    ypr1[0] = ypr1[0] * 180 / M_PI;
    ypr1[1] = ypr1[1] * 180 / M_PI;
    ypr1[2] = ypr1[2] * 180 / M_PI;

    ypr2[0] = ypr2[0] * 180 / M_PI;
    ypr2[1] = ypr2[1] * 180 / M_PI;
    ypr2[2] = ypr2[2] * 180 / M_PI;
    // Skip 300 readings (self-calibration process)
    int correct1, correct2;
    if (j <= 300) {
      correct1 = ypr1[0]; // Yaw starts at random value, so we capture last value after 300 readings
      correct2 = ypr2[0]; // Yaw starts at random value, so we capture last value after 300 readings
      j++;
    }
    // After 300 readings
    
    else {
      ypr1[0] = ypr1[0] - correct1; // Set the Yaw to 0 deg - subtract  the last random Yaw value from the currrent value to make the Yaw 0 degrees
      ypr2[0] = ypr2[0] - correct2;
      // Map the values of the MPU6050 sensor from -90 to 90 to values suatable for the servo control from 0 to 180
      int yaw1 = map(ypr1[0], -90, 90, 0, 180);
      int roll1 = map(ypr1[1], -90, 90, 0, 180);
      int pitch1 = map(ypr1[2], -90, 90, 180, 0);
      int yaw2 = map(ypr2[0], -90, 90, 0, 180);
      int roll2 = map(ypr2[1], -90, 90, 0, 180);
      int pitch2 = map(ypr2[2], -90, 90, 180, 0);
      int useful_yaw = yaw2-yaw1+correction_yaw;
      int hand_pitch = pitch2-pitch1+correction_pitch;
      // Control the servos according to the MPU6050 orientation
      if(cal > 0){
        sum_yaw += (yaw1-yaw2);
        sum_pitch += (pitch1 - pitch2);
      }
      if(cal = 0){
        correction_yaw = sum_yaw/100;
        correction_pitch = sum_pitch/100;
      }
      //useful things are: pitch1, pitch2, roll2, yaw2.
      
      Serial.print("Useful values: ");Serial.print("pitch1: ");Serial.print(pitch1);Serial.print("hand_pitch: ");Serial.print(hand_pitch);Serial.print("roll2: ");Serial.print(roll2);Serial.print("useful_yaw: ");Serial.println(useful_yaw);
    }
#endif
  }
}
