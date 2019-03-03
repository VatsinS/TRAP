// put your setup code here, to run once:
// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
//sensor 1 is on pin 1, sensor 2 is on pin 2 and sensor 3 is on pin 3
#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t GyX,GyY,GyZ;
void setup(){
  Wire.begin();
  pinMode(1,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  for(int i = 1; i <=3 ; i++){
    if(i == 1){
        digitalWrite(1,LOW);
        digitalWrite(2,HIGH);
        digitalWrite(3,HIGH);
    }
    if(i == 2){
        digitalWrite(2,LOW);
        digitalWrite(1,HIGH);
        digitalWrite(3,HIGH);
    }
    if(i == 3){
        digitalWrite(3,LOW);
        digitalWrite(2,HIGH);
        digitalWrite(1,HIGH);
    }
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
  }
  Serial.begin(9600);
}
void loop(){
  for(int i = 1; i <=3 ; i++){
    if(i == 1){
        digitalWrite(1,LOW);
        digitalWrite(2,HIGH);
        digitalWrite(3,HIGH);
    }
    if(i == 2){
        digitalWrite(2,LOW);
        digitalWrite(1,HIGH);
        digitalWrite(3,HIGH);
    }
    if(i == 3){
        digitalWrite(3,LOW);
        digitalWrite(2,HIGH);
        digitalWrite(1,HIGH);
    }
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    Serial.print(" | GyX = "); Serial.print(GyX); Serial.print(" for sensor: "); Serial.println(i);
    Serial.print(" | GyY = "); Serial.print(GyY); Serial.print(" for sensor: "); Serial.println(i);
    Serial.print(" | GyZ = "); Serial.print(GyZ); Serial.print(" for sensor: "); Serial.println(i);
    delay(333);
  }
}
