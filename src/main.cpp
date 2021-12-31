#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_MMA8451.h>


Servo servo_turning;
Adafruit_MMA8451 accelerometer = Adafruit_MMA8451();


void setup() {
// write your initialization code here

servo_turning.attach(SERVOPIN);
accelerometer.begin()
accelerometer.setRange(MMA8451_RANGE_2_G);

}

void loop() {
// write your code here
}