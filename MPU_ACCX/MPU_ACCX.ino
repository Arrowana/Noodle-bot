// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>
const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU);
  //Set low pass filter - see register datasheet
  Wire.write(0x1A);
  Wire.write(0x06);
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU);
  //Set GYRO_CONFIG
  Wire.write(0x1B);
  //Write config to bit4 and bit3
  Wire.write(0 << 3);
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU);
  //Set ACCEL_CONFIG
  Wire.write(0x1C);
  //Write config to bit4 and bit3
  Wire.write(0 << 3);
  Wire.endTransmission(true);
  
  Serial.begin(115200);
}
void loop(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.println("data:AcX"); Serial.println(AcX);
  Serial.println("data:AcY"); Serial.println(AcY);
  Serial.println("data:AcZ"); Serial.println(AcZ);
  Serial.println("data:Tmp"); Serial.println(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.println("data:GyX"); Serial.println(GyX);
  Serial.println("data:GyY"); Serial.println(GyY);
  Serial.println("data:GyZ"); Serial.println(GyZ);
  
  Wire.beginTransmission(MPU);
  Wire.write(0x1A);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,1,true);
  Serial.println("Data:Low_pass");
  Serial.println(B00000111 & Wire.read());
  
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,1,true);
  Serial.println("Data:Gyro_config");
  Serial.println((B00011000 & Wire.read())>>3);
  
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,1,true);
  Serial.println("Data:Accel_config");
  Serial.println((B00011000 & Wire.read())>>3);
  
  
  delay(50);
}
