
// Based on:
// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain

#include<Wire.h>

const int MPU=0x68;  // I2C address of the MPU-6050

int16_t GyZ;
int16_t GyZoffset;
float angle = 0;

unsigned long prev_ts;

uint16_t iter;

void setup() {
  delay(400);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);   // GYRO_CONFIG register
  Wire.write(0x18);   // set sensitivity to +-2000deg/s
  Wire.endTransmission(true);
  
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);   // PWR_MGMT_1 register
  Wire.write(0);      // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(115200);
  delay(1000);
  GyZoffset = 0;
  for(int i=0; i<100; i++) {
    GyZoffset += readGyroRaw();
  }
  GyZoffset /= 100;
  Serial.println("Offset:");
  Serial.println(GyZoffset);

  Serial.println("Gyro config register:");
  
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);  // starting with register 0x47 (GYRO_CONFIG)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,1,true);  // request 1 register
  Serial.println(Wire.read());
  
  delay(10000);

  prev_ts = millis();
  iter = 0;
}

int16_t readGyroRaw() {
  Wire.beginTransmission(MPU);
  Wire.write(0x47);  // starting with register 0x47 (GYRO_ZOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,2,true);  // request a total of 2 registers
  return Wire.read()<<8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}


void loop() {
  unsigned long ts = micros();
  GyZ = (readGyroRaw()-GyZoffset);

  float dt = ((float)(ts-prev_ts))/1000000.;
  
  angle += GyZ*dt;
  if((iter % 100) == 0)
    Serial.println(angle*2000./32768.); // +-2000 deg/s sensitivity; +-2^15 full scale
  //delay(5000);
  prev_ts = ts;
  iter++;
}

