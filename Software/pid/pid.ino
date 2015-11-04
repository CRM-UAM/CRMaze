// Code for the CRMaze robot
// By Víctor Uceda & Carlos García
//   Club de Robótica-Mecatrónica,
//   Universidad Autónoma de Madrid
//   http://crm-uam.github.io
// License: CC-BY-SA (http://creativecommons.org/licenses/by-sa/4.0/)
//
// Derived from:
//   GNBot ( https://github.com/carlosgs/GNBot )
//   MPU-6050 Short Example Sketch by Arduino User JohnChi

#define RED_LED_PIN     13
#define GREEN_LED_PIN   9

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




#include<Wire.h>

const int MPU=0x68;  // I2C address of the MPU-6050

#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_GYRO_CONFIG   0x1B

void IMUwriteReg(byte reg, byte val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

int16_t GyZoffset;
float yaw;
unsigned long prev_ts;

void init_IMU() {
  Wire.begin();
  
  // Measure gyro sensor offset (robot must remain still)
  GyZoffset = 0;
  for(int i=0; i<10; i++)
    GyZoffset += readGyro_raw();
  GyZoffset /= 10;
  Serial.println("Gyro offset:");
  Serial.println(GyZoffset);
  
  IMUwriteReg(MPU6050_PWR_MGMT_1, bit(7) );  // DEVICE_RESET to 1 (D7=1)
  delay(100);
  IMUwriteReg(MPU6050_PWR_MGMT_1, 0 ); // set SLEEP to zero (D6=0, wakes up the MPU-6050)
  IMUwriteReg(MPU6050_GYRO_CONFIG, bit(3) | bit(4) ); // set sensitivity to +-2000deg/s (D3=1, D4=1)

  yaw = 0;
}

int16_t readGyro_raw() {
  Wire.beginTransmission(MPU);
  Wire.write(0x47);  // starting with register 0x47 (GYRO_ZOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,2,true);  // request a total of 12 registers
  return Wire.read()<<8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

// Outputs the robot's rotational speed (yaw) in degrees/second
float readGyro() {
  return (readGyro_raw()-GyZoffset)*2000./32768.; // +-2000 deg/s sensitivity; +-2^15 full scale
}

float updateGyro(float rotationalSpeed) {
  unsigned long ts = micros();
  float dt = ((float)(ts-prev_ts))/1000000.;
  if(dt > 0 && dt < 0.5) { // Ensures the integration only over sensible sampling intervals
    yaw += rotationalSpeed*dt;
  }
  prev_ts = ts;
  return dt;
}



// Music notes
#define DO 262
#define RE 294
#define MI 330
#define FA 349
#define SOL 392
#define LA 440
#define SI 494

void playNote(int frequency, int length) {
    if(frequency <= 0) frequency = DO;
    if(length <= 0) length = 100;
    //tone(BUZZER_PIN,frequency);
    delay(length);
    //noTone(BUZZER_PIN);
    delay(100);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



void init_motor_pins() {
  pinMode(M1A_PIN,OUTPUT);
  pinMode(M2A_PIN,OUTPUT);
  pinMode(M3A_PIN,OUTPUT);
  pinMode(M4A_PIN,OUTPUT);
}

// Input: [-255,+255] (positive values move the robot forward)
void Lmotor_PWM(int16_t vel) {
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

// Input: [-255,+255] (positive values move the robot forward)
void Rmotor_PWM(int16_t vel) {
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

// Calibrated motor functions (cm/s)
#define motorDeadZonePWM 25
#define motorK_PWM       100
#define motorK_CMs       30
int16_t map_vel_pwm(float vel) {
  int16_t res = 0;
  if(vel > 0)
    res = mapf(vel, 0,motorK_CMs, motorDeadZonePWM, motorK_PWM);
  if(vel < 0)
    res = mapf(vel, 0,-motorK_CMs, -motorDeadZonePWM, -motorK_PWM);
  return round(max(min(res,255),-255));
}

void set_motor_speed(float velocity, float offset) {
  Lmotor_PWM(map_vel_pwm(velocity+offset));
  Rmotor_PWM(map_vel_pwm(velocity-offset));
}

#define MIN_ROT_SPEED 10
void motor_calibration() {
  int16_t Lmotor_minA, Lmotor_minB, Rmotor_minA, Rmotor_minB;
  Serial.println("Motor calibration. Minimum speeds:");
  float rotSpeed;
  
  Lmotor_PWM(0);
  Rmotor_PWM(0);
  Lmotor_minA = 0;
  rotSpeed = 0;
  while(abs(rotSpeed) < MIN_ROT_SPEED) {
    rotSpeed = readGyro();
    Lmotor_PWM(Lmotor_minA);
    Lmotor_minA++;
    delay(100);
  }
  Lmotor_PWM(0);
  playNote(DO, 100);
  Serial.print("Lmotor_A\t");
  Serial.print(rotSpeed,5);
  Serial.print(" \t--->\t");
  Serial.println(Lmotor_minA);


  Lmotor_PWM(0);
  Rmotor_PWM(0);
  Rmotor_minA = 0;
  rotSpeed = 0;
  while(abs(rotSpeed) < MIN_ROT_SPEED) {
    rotSpeed = readGyro();
    Rmotor_PWM(Rmotor_minA);
    Rmotor_minA++;
    delay(100);
  }
  Rmotor_PWM(0);
  playNote(MI, 100);
  Serial.print("Rmotor_A\t");
  Serial.print(rotSpeed,5);
  Serial.print(" \t--->\t");
  Serial.println(Rmotor_minA);


  Lmotor_PWM(0);
  Rmotor_PWM(0);
  Lmotor_minB = 0;
  rotSpeed = 0;
  while(abs(rotSpeed) < MIN_ROT_SPEED) {
    rotSpeed = readGyro();
    Lmotor_PWM(Lmotor_minB);
    Lmotor_minB--;
    delay(100);
  }
  Lmotor_PWM(0);
  playNote(SOL, 100);
  Serial.print("Lmotor_B\t");
  Serial.print(rotSpeed,5);
  Serial.print(" \t--->\t");
  Serial.println(Lmotor_minB);


  Lmotor_PWM(0);
  Rmotor_PWM(0);
  Rmotor_minB = 0;
  rotSpeed = 0;
  while(abs(rotSpeed) < MIN_ROT_SPEED) {
    rotSpeed = readGyro();
    Rmotor_PWM(Rmotor_minB);
    Rmotor_minB--;
    delay(100);
  }
  Rmotor_PWM(0);
  playNote(DO*2, 100);
  Serial.print("Rmotor_B\t");
  Serial.print(rotSpeed,5);
  Serial.print(" \t--->\t");
  Serial.println(Rmotor_minB);

  Serial.println("Calibration done!");
  delay(10000);
}



// --- BUTTON ---
void init_button_pin() {
  pinMode(BUTTON_PIN,INPUT);
  digitalWrite(BUTTON_PIN, HIGH); // Enable internal pull-up resistor
}

bool button_is_pressed() {
  return !digitalRead(BUTTON_PIN);
}




float getDistanceCM(int pin) {
  float measurement = analogRead(pin);
  float ir_K = 4419.36; // Linearization of the sensor response
  float ir_C = 32.736;
  if(measurement <= ir_C) return 150;
  float res = ir_K/(measurement-ir_C);
  if(res < 0 || res > 150) res = 150;
  return res;
}

float getBatteryVoltage() {
  int val = analogRead(BATTERY_V_PIN);
  return mapf(val,0,1023, 0,21.1765); // Voltage divider with 22k in series with 6k8
}





void setup() {
  delay(400);
  
  init_motor_pins();
  init_button_pin();
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  
  Serial.begin(115200);
  
  
  // Low battery notification (program will stop here if a minimum of 7V are not available)
  if(getBatteryVoltage() < 7) {
    while(1) {
      digitalWrite(RED_LED_PIN, HIGH);
      delay(100);
      digitalWrite(RED_LED_PIN, LOW);
      delay(100);
    }
  }
  
  
  digitalWrite(RED_LED_PIN, HIGH);
  
  // Robot must remain still while the following routine is executed
  init_IMU();
  
  
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, HIGH);
  
  while(!button_is_pressed());
  digitalWrite(GREEN_LED_PIN, LOW);
  delay(1000);


  // Use this code to calculate motorK_PWM and motorK_CMs
  Lmotor_PWM(40);
  Rmotor_PWM(40);
  delay(3000);
  Lmotor_PWM(0);
  Rmotor_PWM(0);
  while(1);

  // Use this to estimate motorDeadZonePWM
  //motor_calibration();
}

float yawGoal;

/*void loop() {
  float rotationalSpeed = readGyro();
  float dt = updateGyro(rotationalSpeed);
  
  set_motor_speed(0, yaw);
  Serial.println(yaw);
}*/

float kp = 0.1;
float kd = 0.01;
float ki = 0.05;

int motorPIDcontroller(float yawGoal, boolean term_yawReached, float c_speed, float IRdistGoal_cm, boolean term_IRdistReached, float yawEnd, float distanceGoal, boolean term_distanceReached) {
  float prev_error = 0;
  boolean saturated = false;
  boolean first_iteration = true;
  float error_integral = 0;
  float error_derivative = 0;
  float distance_integral = 0;
  while(1) {
    float rotationalSpeed = readGyro();
    float dt = updateGyro(rotationalSpeed);

    float targetYaw = yawGoal;
    if(term_distanceReached) targetYaw = mapf(distance_integral,0,distanceGoal,yawGoal,yawEnd);
    float error = (yaw-targetYaw);
    while(error > 180) error -= 360;
    while(error <= -180) error += 360;
    
    if(!first_iteration) {
      if(!saturated) error_integral += prev_error*dt;
      error_derivative = (error-prev_error)/dt;
      distance_integral += c_speed*dt;
      if(term_distanceReached) {
          if(c_speed > 0) {
              if(distance_integral > distanceGoal-c_speed*dt) return 3;
          } else {
              if(distance_integral < distanceGoal+c_speed*dt) return 3;
          }
      }
    } else first_iteration = false;
  
    if(term_yawReached && abs(error)+abs(error_integral)+abs(error_derivative) < 10) return 1;
    
    float v = kp*error + ki*error_integral + kd*error_derivative;
    
    if(abs(v)>100) { // 100cm/s
      saturated = true;
    } else {
      if(saturated) error_integral = 0;
      saturated = false;
    }
    
    set_motor_speed(c_speed, v);
    
    prev_error = error;
  }
}

void pointToAngle(float yawGoal) {
    motorPIDcontroller(yawGoal, true, 0, 0, false, 0, 0, false);
    set_motor_speed(0, 0);
}


void loop() {
  pointToAngle(45);
  delay(1000);
  pointToAngle(-45);
  delay(1000);
}

