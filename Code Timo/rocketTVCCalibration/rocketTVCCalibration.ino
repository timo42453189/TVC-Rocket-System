#include <Wire.h>
#include <Servo.h>


#define SX_CENTER 89
#define SX_HIGH 104
#define SX_LOW 76
#define SX_PIN 10

#define SY_CENTER 85
#define SY_HIGH 120
#define SY_LOW 50
#define SY_PIN 8

Servo sx;
Servo sy;

void init_servo(){
  sx.attach(SX_PIN);
  sy.attach(SY_PIN);
}

void setup(void) {
  Serial.begin(115200);
  init_servo();
  sy.write(SY_HIGH);
  sx.write(SX_CENTER);
}

void loop(){}