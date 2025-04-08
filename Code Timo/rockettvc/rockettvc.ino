#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WebServer.h>

#define SX_CENTER 89
#define SX_HIGH 104
#define SX_LOW 76
#define SX_PIN 14

#define SY_CENTER 85
#define SY_HIGH 120
#define SY_LOW 50
#define SY_PIN 27

#define MAX_YAW_DIFF 30.0
#define MAX_PITCH_DIFF 30.0

// Globale Variablen für den Puffer
String buffer = "";                // Puffer für die Daten
const size_t bufferLimit = 512;    // Maximale Puffergröße in Bytes

struct Rotation {
  double yaw;
  double pitch;
};

Servo sx;
Servo sy;

unsigned long lastFlush = 0;
const unsigned long flushInterval = 5000;

double setPoint = 0;
double inputX, inputY;
double KpX = 0.6, KiX = 0.05, KdX = 0.01;
double KpY = 0.6, KiY = 0.05, KdY = 0.01;
double driverOutX, driverOutY;

double OffsetX = 0.0, OffsetY = 0.0;

PID PID_X(&inputX, &driverOutX, &setPoint, KpX, KiX, KdX, DIRECT);
PID PID_Y(&inputY, &driverOutY, &setPoint, KpY, KiY, KdY, DIRECT);

Adafruit_BNO055 bno = Adafruit_BNO055(55);
float correction_angle = 90.0;
bool calibrated = false;

const char* ap_ssid = "Rocket-Calibration";
const char* ap_password = "12345678";
WebServer server(80);
bool ap_mode = false;
bool corrected = false;

void startAccessPoint() {
  WiFi.softAP(ap_ssid, ap_password);
  Serial.println("Access Point gestartet!");
  Serial.print("IP Adresse: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", []() {
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    String html = "<h1>Kalibrierung</h1>";
    html += "System: " + String(sys) + "<br>";
    html += "Gyro: " + String(gyro) + "<br>";
    html += "Accel: " + String(accel) + "<br>";
    html += "Mag: " + String(mag) + "<br>";
    server.send(200, "text/html", html);
  });

  server.begin();
  ap_mode = true;
}

void saveOffsets(){
  Rotation r = getRotation();
  OffsetX = -r.pitch;
  OffsetY = -r.yaw;
  corrected = true;
}

void correctionRequest(){
  WiFi.softAP(ap_ssid, ap_password);
  Serial.println("Access Point gestartet!");
  Serial.print("IP Adresse: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", []() {
    String html = "<h1>Offset</h1>";
    html += "<button onclick=\"location.href='/buttonPressed'\">Die Rakete ist nun gerade</button>";
    server.send(200, "text/html", html);
  });

  server.on("/buttonPressed", []() {
    saveOffsets();
    server.send(200, "text/plain", "Success!");
  });

  server.begin();
  ap_mode = true;
}


void stopAccessPoint() {
  server.stop();
  WiFi.softAPdisconnect(true);
  Serial.println("Access Point deaktiviert.");
  ap_mode = false;
}

int init_pid() {
  PID_X.SetMode(AUTOMATIC);
  PID_X.SetOutputLimits(-(SX_HIGH - SX_CENTER), (SX_HIGH - SX_CENTER));
  PID_Y.SetMode(AUTOMATIC);
  PID_Y.SetOutputLimits(-(SY_HIGH - SY_CENTER), (SY_HIGH - SY_CENTER));
  return (PID_X.GetMode() == AUTOMATIC && PID_Y.GetMode() == AUTOMATIC) ? 0 : 1;
}

void init_servo() {
  sx.attach(SX_PIN);
  sy.attach(SY_PIN);
}

void displayCalStatus() {
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  if (mag == 3 && sys == 3 && gyro == 3 && accel == 3) {
    calibrated = true;
    stopAccessPoint();
  }
}

int init_imu() {
  int status = bno.begin();
  Serial.println(status);
  bno.setExtCrystalUse(true);
  return status;
}

void init_SPIFFS() {
  SPIFFS.begin(true);
  File file = SPIFFS.open("/data.csv", FILE_APPEND);
  file.println("Time,Yaw,Pitch,DriverOutX,DriverOutY,AccelZ");
  file.close();
}

Rotation getRotation() {
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> eul = quat.toEuler();
  float raw_yaw = eul.z() * (180.0 / PI) - correction_angle;
  float raw_pitch = eul.y() * (180.0 / PI);

  static float last_yaw = 0;
  static float last_pitch = 0;

  if (isnan(raw_pitch) || abs(raw_pitch - last_pitch) > MAX_PITCH_DIFF) {
    raw_pitch = last_pitch;
  } else {
    last_pitch = raw_pitch;
  }

  if (isnan(raw_yaw) || abs(raw_yaw - last_yaw) > MAX_YAW_DIFF) {
    raw_yaw = last_yaw;
  } else {
    last_yaw = raw_yaw;
  }

  return {raw_yaw + OffsetY, raw_pitch + OffsetX};
}

float getAccelY() {
  imu::Vector<3> lin_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  return lin_accel.y();
}

void setup() {
  Serial.begin(115200);
  Serial.println("INIT SETUP");

  if (init_pid()) {
    Serial.println("PID setup fehlgeschlagen");
    while (true);
  }

  init_servo();

  if (init_imu() == 0) {
    Serial.println("IMU setup fehlgeschlagen");
    while (true);
  }

  init_SPIFFS();

}

bool running_main = false;

void loop() {
  Rotation rot = getRotation();

  if (!calibrated && !ap_mode && !running_main) {
    startAccessPoint();
    Serial.println("START AP");
  }
  if (calibrated && !ap_mode && !corrected && !running_main){
    Serial.println("OFFSET AP");
    correctionRequest();
  }
  if (calibrated && ap_mode && corrected && !running_main) {
    stopAccessPoint();
  }
  if (ap_mode) {
    server.handleClient();
  }

  if (calibrated && corrected) {
    running_main = true;
  // Datenaufzeichnung auf 8-mal pro Sekunde begrenzen
    static unsigned long lastImageTime = 0;
    if (millis() - lastImageTime >= 125) { // 125 ms = 1/8 Sekunde
      lastImageTime = millis();

      // Daten in den Puffer schreiben
      buffer += String(millis()) + "," + String(rot.yaw) + "," + String(rot.pitch) + ",";
      buffer += String(driverOutX) + "," + String(driverOutY) + "," + String(getAccelY()) + "\n";
    }

    if (millis() - lastFlush >= flushInterval || buffer.length() >= bufferLimit) {
      File dataFile = SPIFFS.open("/data.csv", FILE_APPEND);
      if (dataFile) {
        dataFile.print(buffer); // Puffer in die Datei schreiben
        dataFile.close();
        buffer = ""; // Puffer leeren
      }
      lastFlush = millis();
    }

    // Daten in den Puffer schreiben
    inputX = rot.pitch;
    inputY = rot.yaw;
    PID_X.Compute();
    PID_Y.Compute();

    Serial.print("Yaw: ");
    Serial.print(rot.yaw);
    Serial.print(", Pitch: ");
    Serial.println(rot.pitch);
    // Motorwerte einstellen
    sx.write(SX_CENTER - driverOutX);
    sy.write(SY_CENTER + driverOutY);

    // Nicht-blockierendes "Delay"
    static unsigned long lastAction = 0;
    if (millis() - lastAction >= 100) {
      lastAction = millis();
    }
  }
}
