#include<Wire.h>
const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(115200);
}
void loop(){
  Wire.beginTransmission(MPU);
  //Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.write(0x1A);
  Wire.endTransmission(false);
  //Read low pass filter value
  Wire.requestFrom(MPU, 1, true);
  Serial.println("Low pass filter setting :");
  Serial.println(B00000111 & Wire.read());
  //Wire.endTransmission(true);
  
  //Wire.beginTransmission(MPU);
  Wire.write(0x1A);
  Wire.write(0x02);
  //Wire.endTransmission(true);
  
  delay(200);
}
