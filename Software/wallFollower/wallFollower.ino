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
  //Serial.println("Gyro offset:");
  //Serial.println(GyZoffset);

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
  Wire.requestFrom(MPU,2,true);  // request a total of 2 registers
  return Wire.read()<<8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

// Outputs the robot's rotational speed (yaw) in degrees/second
float readGyro() {
  return (readGyro_raw()-GyZoffset)*2000./32768.; // +-2000 deg/s sensitivity; +-2^15 full scale
}

float updateGyro(float rotationalSpeed) {
  unsigned long ts = micros();
  float dt = ((float)(ts-prev_ts))/1000000.;
  if(dt > 0.001 && dt < 0.5) { // Ensures the integration only over sensible sampling intervals
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
    tone(BUZZER_PIN,frequency);
    delay(length);
    noTone(BUZZER_PIN);
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
#define motorDeadZonePWM 20
#define motorK_PWM       100
#define motorK_CMs       40
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



// --- BUTTON ---
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



bool turnLeft = true;


float kp = 1;
float kd = 0.05;
float ki = 1;

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

    float distR = getDistanceCM(DIST_1_PIN);
    float distL = getDistanceCM(DIST_3_PIN);

    if(button_is_pressed()) {
      set_motor_speed(0, 0);
      while(1);
    }

    float targetYaw = yawGoal;
    if(term_distanceReached) targetYaw = mapf(distance_integral,0,distanceGoal,yawGoal,yawEnd);
    float error = 0;//2*(distR-15);//(yaw-targetYaw);
    if(term_yawReached)
      error = (yaw-targetYaw);
    else {
      if(distR < 20) error = 3*(distR-12);
      if(distL < 20 && distL < distR) error = -3*(distL-12);
    }
    while(error > 180) error -= 360;
    while(error <= -180) error += 360;

    if(!first_iteration) {
      if(!saturated && term_yawReached) error_integral += prev_error*dt;
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
    
    if(!term_yawReached) {
        if(turnLeft) {
          if(distL > 20) return 8;
        } else {
          if(distR > 20) return 8;
        }
    }

    if(term_yawReached && abs(error)+abs(error_integral)+abs(error_derivative) < 10) return 1;
    if(term_yawReached && abs(yaw-targetYaw)<10) return 1;
    if(term_IRdistReached) {
        float dist = getDistanceCM(DIST_2_PIN);
        if(c_speed > 0) {
            if(dist < IRdistGoal_cm) return 2;
        } else {
            if(dist > IRdistGoal_cm) return 2;
        }
    }

    float v = kp*error + ki*error_integral + kd*error_derivative;

    if(abs(v)>400) {
      saturated = true;
    } else {
      if(saturated) error_integral = 0;
      saturated = false;
    }

    set_motor_speed(c_speed, min(max(v,-5),5));

    prev_error = error;
  }
}

void turn(float yawGoal) {
    motorPIDcontroller(yaw+yawGoal, true, 0, 0, false, 0, 0, false);
    /*float rotVel = 10;
    yawGoal += yaw;
    float error = yaw-yawGoal;
    while(error > 180) error -= 360;
    while(error <= -180) error += 360;

    if(error < 0) rotVel *= -1;

    set_motor_speed(1, rotVel);
    while(abs(error) > 10) {
      float rotationalSpeed = readGyro();
      float dt = updateGyro(rotationalSpeed);
      if((yaw-yawGoal)*error < 0) break;
      error = yaw-yawGoal;
    }*/
    set_motor_speed(0, 0);
    delay(500);
}








void setup() {
  delay(400);
  init_motor_pins();
  pinMode(BUTTON_PIN,INPUT_PULLUP);
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
  if(button_is_pressed())
    turnLeft = false;
  digitalWrite(GREEN_LED_PIN, HIGH);
  delay(1000);


  // Use this code to calculate motorK_PWM and motorK_CMs
  /*Lmotor_PWM(100);
  Rmotor_PWM(100);
  delay(3000);//~120cm
  Lmotor_PWM(0);
  Rmotor_PWM(0);*/

  //while(!button_is_pressed());
  //digitalWrite(GREEN_LED_PIN, HIGH);
  //delay(1000);

  /*set_motor_speed(28, 0);
  delay(500);
  set_motor_speed(0, 0);
  while(!button_is_pressed());
  delay(1000);*/
  // Use this to estimate motorDeadZonePWM
  //motor_calibration();
  //turn(90);

  delay(1000);

}


bool invierte = false;
void loop() {
  int ret = motorPIDcontroller(0, false, 10, 12, true, 0, 0, false);
  //delay(500);
  invierte = false;
  if(ret == 8) {
    set_motor_speed(0, 0);
    delay(500);
    set_motor_speed(12, 0);
    delay(700);
    if(getDistanceCM(DIST_2_PIN)<25) invierte = true;
  } else {
    float distR = getDistanceCM(DIST_1_PIN);
    float distL = getDistanceCM(DIST_3_PIN);
  }
  if(turnLeft){
    if(!invierte) turn(90);
    else turn(-90);
  } else {
    if(!invierte) turn(-90);
    else turn(90);
  }
  if(ret == 8) {
    set_motor_speed(12, 0);
    delay(500);
  }
  /*int i=0;
  for(i=0;i<30;i+=5){
    set_motor_speed(i, 0);
    delay(2000);
    set_motor_speed(0, 0);
    delay(500);
    turn(90);
    set_motor_speed(0, 0);
    delay(500);
  }
  delay(5000);*/
}


/*
bool gap = false;
void loop(){
  
  float distR = getDistanceCM(DIST_1_PIN);
  float distM = getDistanceCM(DIST_2_PIN);
  float distL = getDistanceCM(DIST_3_PIN);
  
  if(distR > 12 && gap == false) 
    set_motor_speed(20, 0);
    delay(500);
    gap = true
  else gap = false;
  

  float error = 2*(distR-12);
  if(error > 25) error = 25;
  if(error < -25) error = -25;
  
  float errorF = 20+(distM-12);
  if(errorF > 20) errorF = 20;
  if(errorF < 0) errorF = 0;
  
  set_motor_speed(errorF, error);
  
  if(button_is_pressed()) {
    set_motor_speed(0, 0);
    while(1);
  }
}*/

