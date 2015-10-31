
// Based on:
// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain

#include<Wire.h>

const int MPU=0x68;  // I2C address of the MPU-6050

int16_t GyZ;
int16_t GyZoffset;
float angle_big = 0;

unsigned long prev_ts;

//uint16_t iter;



#define LED_RED_PIN     13
#define LED_GREEN_PIN   9
#define BUZZER_PIN      8
#define LINE_R_PIN      2
#define LINE_M_PIN      3
#define LINE_L_PIN      4
#define M4A_PIN         5
#define M3A_PIN         6
#define BUTTON_PIN      7
#define DIST_3_PIN      A3
#define DIST_2_PIN      A2
#define DIST_1_PIN      A1
#define BATTERY_V_PIN   A0
#define M1A_PIN         11
#define M2A_PIN         10

void init_motor_pins() {
  pinMode(M1A_PIN,OUTPUT);
  pinMode(M2A_PIN,OUTPUT);
  pinMode(M3A_PIN,OUTPUT);
  pinMode(M4A_PIN,OUTPUT);
}

void LmotorSpeed(int16_t vel) {
  if(vel > 0) {
    analogWrite(M1A_PIN, 0);
    analogWrite(M2A_PIN, vel);
  } else if(vel < 0) {
    analogWrite(M1A_PIN, -vel);
    analogWrite(M2A_PIN, 0);
  } else {
    analogWrite(M1A_PIN, 0);
    analogWrite(M2A_PIN, 0);
  }
}

void RmotorSpeed(int16_t vel) {
  if(vel > 0) {
    analogWrite(M3A_PIN, 0);
    analogWrite(M4A_PIN, vel);
  } else if(vel < 0) {
    analogWrite(M3A_PIN, -vel);
    analogWrite(M4A_PIN, 0);
  } else {
    analogWrite(M3A_PIN, 0);
    analogWrite(M4A_PIN, 0);
  }
}




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
  //Serial.begin(115200);
  delay(1000);
  GyZoffset = 0;
  for(int i=0; i<100; i++) {
    GyZoffset += readGyroRaw();
  }
  GyZoffset /= 100;
  //Serial.println("Offset:");
  //Serial.println(GyZoffset);

  //Serial.println("Gyro config register:");
  
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);  // starting with register 0x47 (GYRO_CONFIG)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,1,true);  // request 1 register
  //Serial.println(Wire.read());

  prev_ts = millis();
  //iter = 0;
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
  
  angle_big += GyZ*dt;
  float angle = angle_big*2000./32768.;
  int error = max(min(round(255.*angle/45.),255),-255);
  //Serial.println(angle);

  LmotorSpeed(50+error);
  RmotorSpeed(50-error);
  
  //if((iter % 100) == 0)
  //  Serial.println(angle*2000./32768.); // +-2000 deg/s sensitivity; +-2^15 full scale
  //delay(5000);
  prev_ts = ts;
  //iter++;
}

