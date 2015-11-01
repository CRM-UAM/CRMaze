

// Based on:
// MPU-6050 Short Example Sketch by Arduino User JohnChi

#include<Wire.h>

const int MPU=0x68;  // I2C address of the MPU-6050

#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG        0x1A

void init_IMU() {
  for(int i=0; i<3; i++) { // Reset the IMU a few times
    IMUwriteReg(MPU6050_PWR_MGMT_1, bit(7) );  // DEVICE_RESET to 1 (D7=1)
    delay(100);
  }

  IMUwriteReg(MPU6050_PWR_MGMT_1, bit(0) | bit(1) ); // set clock source to Z Gyro (D0,D1=1, D2=0) and set SLEEP to zero (D6=0, wakes up the MPU-6050)

  IMUwriteReg(MPU6050_ACCEL_CONFIG, bit(3) | bit(4) ); // set sensitivity to +-16G (D3=1,D4=1) and disable high pass filter (D0,D1,D2=0)

  IMUwriteReg(MPU6050_GYRO_CONFIG, bit(3) | bit(4) ); // set sensitivity to +-2000deg/s (D3=1,D4=1)

  IMUwriteReg(MPU6050_SMPLRT_DIV, 0 ); // set sampling rate to 1khz (1khz / (1 + 0) = 1000 Hz)

  IMUwriteReg(MPU6050_CONFIG, bit(1) | bit(5) ); // set digital low pass filter to 94hz (D0,D2=0, D1=1) and EXT_SYNC to GYRO_ZOUT (D3,D4=0, D5=1)
}


#define LED_RED_PIN     13
#define LED_GREEN_PIN   9

#define BUZZER_PIN      8

#define M1A_PIN         11
#define M2A_PIN         10
#define M4A_PIN         5
#define M3A_PIN         6

#define BUTTON_PIN      7

#define LINE_R_PIN      2
#define LINE_M_PIN      3
#define LINE_L_PIN      4

#define DIST_3_PIN      A3
#define DIST_2_PIN      A2
#define DIST_1_PIN      A1

#define BATTERY_V_PIN   A0

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



// --- BUTTON ---
void init_button_pin() {
  pinMode(BUTTON_PIN,INPUT);
  digitalWrite(BUTTON_PIN, HIGH); // Enable internal pull-up resistor
}
// Returns 1 if the button is being pressed, 0 if it is not
int button_is_pressed() {
  return !digitalRead(BUTTON_PIN);
}




int16_t AcY, GyZ;
int16_t AcYoffset, GyZoffset;
float AcY_integral, GyZ_integral;

unsigned long prev_ts;

void IMUwriteReg(byte reg, byte val) {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void setup() {
  delay(400);
  
  init_motor_pins();
  init_button_pin();
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  
  digitalWrite(LED_RED_PIN, HIGH);

  init_IMU();
  
  Serial.begin(115200);
  delay(1000);
  
  // Measure IMU sensor offsets (robot must remain still)
  AcYoffset = 0;
  GyZoffset = 0;
  for(int i=0; i<10; i++) {
    readIMU(&AcY, &GyZ);
    AcYoffset += AcY;
    GyZoffset += GyZ;
  }
  AcYoffset /= 10;
  GyZoffset /= 10;
  Serial.println("Offset:");
  Serial.println(AcYoffset);
  Serial.println(GyZoffset);
  
  
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, HIGH);
  
  //while(!button_is_pressed());
  
  prev_ts = millis();
  
  AcY_integral = 0;
  GyZ_integral = 0;
}

void readIMU(int16_t *AcY, int16_t *GyZ) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3D);  // starting with register 0x3D (ACCEL_YOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);  // request a total of 2 registers
  *AcY = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  for(int i=0; i<8; i++) Wire.read(); // Discard 0x3F-0x46
  *GyZ = Wire.read()<<8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}


int iter = 0;
float linear_motion_big = 0;

void loop() {
  readIMU(&AcY, &GyZ);
  unsigned long ts = micros();
  float dt = ((float)(ts-prev_ts))/1000000.;
  if(dt > 0) { // Ensures the integration only over sensible sampling intervals
    linear_motion_big += AcY_integral*dt;
    AcY_integral += (AcY-AcYoffset)*dt;
    
    GyZ_integral += (GyZ-GyZoffset)*dt;
  }
  
  float linear_motion = linear_motion_big*16./32768.; // +-16G sensitivity; +-2^15 full scale
  float rotation = GyZ_integral*2000./32768.; // +-2000 deg/s sensitivity; +-2^15 full scale
  
  int rot_error = max(min(round(255.*rotation/90.),255),-255);
  int dist_error = max(min(round(255.*(40-1000*linear_motion)/300.),255),-255);
  
  //Serial.println(AcY);


  
  if((iter % 10) == 0) {
    if(1000*linear_motion > 20) {
      LmotorSpeed(0);
      RmotorSpeed(0);
      digitalWrite(LED_RED_PIN, HIGH);
      while(1);
    } else {
      LmotorSpeed(dist_error+rot_error);
      RmotorSpeed(dist_error-rot_error);
    }
    /*Serial.print(rotation);
    Serial.print("\t");
    Serial.println(linear_motion);*/
  }
  prev_ts = ts;
  iter++;

  /*if(button_is_pressed()) {
    linear_motion = 0;
    AcY_integral = 0;
    GyZ_integral = 0;
    delay(1000);
  }*/
  //delay(10);
}

