// Code for the CRMaze robot
// By Víctor Uceda & Carlos García
//   Club de Robótica-Mecatrónica,
//   Universidad Autónoma de Madrid
//   http://crm-uam.github.io
// License: CC-BY-SA (http://creativecommons.org/licenses/by-sa/4.0/)
//
// Derived from:
//   GNBot ( https://github.com/carlosgs/GNBot )

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


// IMU
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int setupIMU() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
    #endif

    mpu.initialize();
/*    mpu.setZGyroOffset(0);*/
/*    */
/*    delay(500);*/
/*    */
/*    int16_t ax, ay, az;*/
/*    int16_t gx, gy, gz;*/
/*    int gzf = 0; // Filtered Z gyro measurement*/
/*    for(int i=0; i<16; i++) {*/
/*        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);*/
/*        gzf += (int)gz;*/
/*        delay(100);*/
/*    }*/
/*    gzf = round(((float)gzf)/16.);*/

    uint8_t devStatus = mpu.dmpInitialize(); //(0 = success, !0 = error)

    if (devStatus == 0) {
        // Apply the newly-calibrated offset
        //mpu.setZGyroOffset(-gzf);

        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        // Get the expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
    }
    return devStatus;
}

void readIMU_YawPitchRoll(float *data) {
    mpu.resetFIFO();

    // Wait for correct available data length
    fifoCount = mpu.getFIFOCount();
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // Read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
}


// --- BUTTON ---

//void init_button_pin() {
//    pinMode(BUTTON_PIN,INPUT);
//    digitalWrite(BUTTON_PIN, HIGH); // Enable internal pull-up resistor
//}
//
//// Returns 1 if the button is being pressed, 0 if it is not
//int button_is_pressed() {
//    return !digitalRead(BUTTON_PIN);
//}






//float getDistanceCM(int pin) {
//    float measurement = analogRead(pin);
//    float ir_K = 4419.36; // Linearization of the sensor response
//    float ir_C = 32.736;
//    if(measurement <= ir_C) return 150;
//    float res = ir_K/(measurement-ir_C);
//    if(res < 0 || res > 150) res = 150;
//    return res;
//}



//float mapf(float x, float in_min, float in_max, float out_min, float out_max)
//{
//    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//}
//
//float getBatteryVoltage() {
//    int val = analogRead(BATTERY_V_PIN);
//    return mapf(val,0,1023, 0,21.1765); // Voltage divider with 22k in series with 6k8
//}


void setup() {
    delay(1000);
    
    int ret = setupIMU();
    
    Serial.begin(115200);
    
    //init_button_pin();
    
    // To-Do: decide on a cut-off/warning voltage value
    // Low battery notification (program will stop here if 16.5V are not available)
    //if(getBatteryVoltage() < 16.5)
    //  unrecoverable_error("Battery voltage is too low");
    
    if(ret == 0) {
      //Serial.println("IMU calibration...");
      delay(20000);
      //Serial.println("IMU successfully initialized :-)");
    }// else Serial.println("Error while initializing IMU :-(");
}

void loop() {
    readIMU_YawPitchRoll(ypr);
    Serial.println(ypr[0]);
    delay(100);
}

