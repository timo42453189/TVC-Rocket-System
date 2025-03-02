#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

#define SX_CENTER 89
#define SX_HIGH 104
#define SX_LOW 76
#define SX_PIN 10

#define SY_CENTER 85
#define SY_HIGH 120
#define SY_LOW 50
#define SY_PIN 7

#define ALPHA 0.98

#define MAX_YAW_DIFF 30.0
#define MAX_PITCH_DIFF 30.0

struct Rotation {
    double yaw;
    double pitch;
};


int minVal=-100;
int maxVal=100;

Servo sx;
Servo sy;


// PID variable Setup
double setPoint = 0;

double inputX;
double KpX = 0.6;
double KiX = 0.005;
double KdX = 0.2;
double driverOutX;
PID PID_X(&inputX, &driverOutX, &setPoint,KpX,KiX,KdX, DIRECT);

double inputY;
double KpY = 1.3;
double KiY = 0.005;
double KdY = 0.2;
double driverOutY;
PID PID_Y(&inputY, &driverOutY, &setPoint,KpY,KiY,KdY, DIRECT);

Adafruit_BNO055 bno = Adafruit_BNO055(55);

float correction_angle = 90.0;
bool calibrated = false;

int init_pid(){
  // PID X
  PID_X.SetMode(AUTOMATIC);
  PID_X.SetOutputLimits(-(SX_HIGH-SX_CENTER), (SX_HIGH-SX_CENTER));
  // PID y
  PID_Y.SetMode(AUTOMATIC);
  PID_Y.SetOutputLimits(-(SY_HIGH-SY_CENTER),  (SY_HIGH-SY_CENTER));
  if (PID_X.GetMode() == AUTOMATIC && PID_Y.GetMode() == AUTOMATIC){
    return 0;
  } else {
    return 1;
  }
}

void init_servo(){
  sx.attach(SX_PIN);
  sy.attach(SY_PIN);
}

void displayCalStatus(void) {
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("\t");
  if (!system) {
    Serial.print("! ");
  }
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
  if (mag == 3 && system == 3 && gyro == 3 && accel == 3){calibrated = true;}
}

int init_imu(){
  int status = bno.begin();
  Serial.println(status);
  bno.setExtCrystalUse(true);
  //calibrateIMU();
  return status;
}

void test_servo_X(){
  sx.write(SX_HIGH);
  delay(500);
  sx.write(SX_CENTER);
  delay(500);
  sx.write(SX_LOW);
  delay(500);
  sx.write(SX_CENTER);
}

void test_servo_Y(){
  sy.write(SY_HIGH);
  delay(500);
  sy.write(SY_CENTER);
  delay(500);
  sy.write(SY_LOW);
}



void setup(void) {
  Serial.begin(115200);
  int status_pid = init_pid();
  if (status_pid == 1){Serial.println("PID setup not succesfull"); while (1){}};
  init_servo();
  int status_imu = init_imu();
  if (status_imu == 0){Serial.println("IMU setup not succesfull"); while (1){}};
  //test_servo_X();
  //test_servo_Y();
}


void computeX(double inputXx){
  inputX = inputXx;
  PID_X.Compute();
}

void computeY(double inputYy){
  inputY = inputYy;
  PID_Y.Compute();
}

float last_yaw = 0;
float last_pitch = 0;


Rotation getRotation() {
    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> eul = quat.toEuler();
    float raw_yaw = eul.z() * (180.0 / 3.14159) - correction_angle;
    float raw_pitch = eul.y() * (180.0 / 3.14159);
    if (isnan(raw_pitch)) {
        raw_pitch = last_pitch;
    } else {
        if (abs(raw_pitch - last_pitch) > MAX_PITCH_DIFF) {
            raw_pitch = last_pitch;
        } else {
            last_pitch = raw_pitch;
        }
    }
    if (isnan(raw_yaw)) {
        raw_yaw = last_yaw;
    } else {
        if (abs(raw_yaw - last_yaw) > MAX_YAW_DIFF) {
            raw_yaw = last_yaw;
        } else {
            last_yaw = raw_yaw;
        }
    }
    Rotation currentRotation;
    currentRotation.yaw = raw_yaw;
    currentRotation.pitch = raw_pitch;
    return currentRotation;
}


void loop() {
  Rotation rot = getRotation();
  computeY(rot.yaw);
  computeX(rot.pitch);
  //displayCalStatus();
  if (calibrated or !calibrated){
    computeX(rot.yaw);
    computeY(rot.pitch);
    Serial.print("Yaw: ");
    Serial.print(rot.yaw);
    Serial.print(", Pitch: ");
    Serial.println(rot.pitch);
    sx.write(SX_CENTER-driverOutX);
    sy.write(SY_CENTER+driverOutY);
  }

}
