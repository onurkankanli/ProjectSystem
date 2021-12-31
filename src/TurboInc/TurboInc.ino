/*
 *  Date: 29.05.2021
 *  Name: Racing Car by Turbo Inc.
 *  Author: Onurkan Kanli
*/

/******************************   Including All Libraries   *****************************/
#include <Servo.h>

/*******************************   Enums and Structures   ******************************/
enum CarStateName {
 Stop,
 Forward,
 SlightRT,
 SlightLT,
 SharpRT,
 SharpLT
};

struct CarState {
 CarStateName stateName;
 int motorRotationDirection;
 int motorSpeed;
 int servoPosition;
 int sharpCycleCount;
};

enum ObstaclePosition {
 NoObstacle,
 ObstacleFar,
 ObstacleNear
};

struct SensorsState {
 bool leftLineSensorOnLine;
 bool rightLineSensorOnLine;
 ObstaclePosition obstacleDetectorResult;
};

/*******************************   Variables and Constants Declaration   ******************************/
#define echoPin 3               // Echo pin in the Ultrasonic sensor
#define trigPin 10               // Trig pin in the Ultrasonic sensor

#define BRAKE 0
#define CW    1
#define CCW   2
#define CS_THRESHOLD 15

//MOTOR 1
#define MOTOR_A1_PIN 7
#define MOTOR_B1_PIN 8

#define PWM_MOTOR_1 5

#define CURRENT_SEN_1 A2

#define EN_PIN_1 A0

/* Ultrasonic Sensor */
const int ObstacleFarDistance  = 30;
const int ObstacleNearDistance = 5;


/* TCRT5000 Sensors */
const int tcrtLeftPin  = 11;     // The variable for pin of the TCRT5000 left sensor
const int tcrtRightPin = 12;     // The variable for pin of the TCRT5000 right sensor

/* Servo */
Servo servo;                     // The variable for servo
const int servoPin = 13;         // The variable for pin of the servo

const int servoNeutral   = 90;
const int servoLowRight  = servoNeutral + 10;
const int servoHighRight = servoNeutral + 45;
const int servoLowLeft   = servoNeutral - 10;
const int servoHighLeft  = servoNeutral - 45;

/* DC Motor */
const int lowSpeed             = 10;
const int maxSpeed             = 100;
const int maxSpeedWithObstacle = 30;

const int ObstacleAroundSpeed  = 7;

int highSpeed = maxSpeed;

/* The condition to determine that the line was lost */
const int maxSharpT = 999;

/* Global objects */
SensorsState sensorsState = {true, true, NoObstacle};
CarState     carState     = {Forward, CW, 0, 0, 0};

/**********************************   Setup Function   **********************************/
void setup() {
  /* Arduino setup */
  Serial.begin(9600);           // Start the serial connection with Arduino

  /* Ultrasonic sensor setup */
  pinMode(trigPin, OUTPUT);     // The Trig pin for the Ultrasonic sensor
  pinMode(echoPin, INPUT);      // The Echo pin for the Ultrasonic sensor

  /* TCRT5000 setup */
  pinMode(tcrtLeftPin,  INPUT);  // The input pin fro the TCRT5000 left sensor
  pinMode(tcrtRightPin, INPUT); // The input pin fro the TCRT5000 right sensor

  /* Servo */
  servo.attach(servoPin);       // The output of the servo

  /* DC Motor */
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);

  pinMode(PWM_MOTOR_1, OUTPUT);

  pinMode(CURRENT_SEN_1, OUTPUT);

  pinMode(EN_PIN_1, OUTPUT);
}

/**********************************   Main Loop Function   ***********************************/
void loop() {
 GetSensorData();
 CalculateNextState();
 ChangeCarState();
}

/*******************************   GetSensorData Function   *******************************/
void GetSensorData(){
 sensorsState.leftLineSensorOnLine  = IsSensorOnWhiteLine(tcrtLeftPin);  // left  TCRT5000 sensor
 sensorsState.rightLineSensorOnLine = IsSensorOnWhiteLine(tcrtRightPin); // right TCRT5000 sensor
 
 long distance = GetDistanceToObstacle();
 
 if(distance > ObstacleFarDistance){ // Check the distance to find the obstacle
  sensorsState.obstacleDetectorResult = NoObstacle;
 } else {
  if(distance > ObstacleNearDistance && distance <= ObstacleFarDistance){
   sensorsState.obstacleDetectorResult = ObstacleFar;
  } else {
   sensorsState.obstacleDetectorResult = ObstacleNear;
  }
 }
};

/*******************************   CalculateNextState Function   *******************************/
void CalculateNextState(){

 if(carState.stateName == Stop){
  return;
 };

 if(sensorsState.obstacleDetectorResult != NoObstacle){
  highSpeed = maxSpeedWithObstacle;
  
  if(sensorsState.obstacleDetectorResult != ObstacleNear){
   GoAroundObstacle();
  }
 } else {
  highSpeed = maxSpeed;
 }
 
 if(sensorsState.leftLineSensorOnLine && sensorsState.rightLineSensorOnLine){
  PrepareCarState(Forward, CW, highSpeed, servoNeutral, true);
 };

 if(!sensorsState.leftLineSensorOnLine && sensorsState.rightLineSensorOnLine){
  PrepareCarState(SlightRT, CW, highSpeed, servoLowRight, true);
 };

 if(sensorsState.leftLineSensorOnLine && !sensorsState.rightLineSensorOnLine){
  PrepareCarState(SlightLT, CW, highSpeed, servoLowLeft, true);
 };
 
 if(!sensorsState.leftLineSensorOnLine && !sensorsState.rightLineSensorOnLine){
  if(carState.stateName == SlightRT){
   PrepareCarState(SharpRT, CW, lowSpeed, servoHighRight, false);
  };

  if(carState.stateName == SlightLT){
   PrepareCarState(SharpLT, CW, lowSpeed, servoHighLeft, false);
  };

  if(carState.sharpCycleCount > maxSharpT){
   FindLine();
  };
 };
};

/*******************************   PrepareCarState Function   *******************************/
void PrepareCarState(CarStateName stateName, int motorRotationDirection, int motorSpeed, int servoPosition, bool resetCycleCount){
 carState.stateName = stateName;
 carState.motorRotationDirection = motorRotationDirection;
 carState.motorSpeed = motorSpeed;
 carState.servoPosition = servoPosition;
 if(resetCycleCount){
  carState.sharpCycleCount = 0;
 } else {
  carState.sharpCycleCount++;
 };
};

/*******************************   ChangeCarState Function   *******************************/
void ChangeCarState(){
  servo.write(carState.servoPosition); // Turn the servo
  delay(10);                           // Delay for turning the servo

  motorGo(carState.motorRotationDirection, carState.motorSpeed);
}

/*******************************   GoAroundObstacle Function   *******************************/
void GoAroundObstacle(){
  servo.write(servoHighLeft); // Turn left and Back
  delay(10);
  motorGo(CCW, ObstacleAroundSpeed);
  delay(50);
  
  servo.write(servoNeutral); // Forward
  delay(10);
  motorGo(CW, ObstacleAroundSpeed);
  delay(100);

  servo.write(servoHighLeft); // Turn left and Forward
  delay(10);
  motorGo(CW, ObstacleAroundSpeed);
  delay(100);
}

/*******************************   FindLine Function   *******************************/
void FindLine(){

 PrepareCarState(Stop, BRAKE, 0, servoNeutral, true);
 
}

/****************************   Ultrasonic Sensor Function   ****************************/
long GetDistanceToObstacle(){
 digitalWrite(trigPin, LOW);               // Clears the trigPin condition
 delayMicroseconds(2);

 digitalWrite(trigPin, HIGH);              // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
 delayMicroseconds(10);
 digitalWrite(trigPin, LOW);

 long duration = pulseIn(echoPin, HIGH);        // Reads the echoPin, returns the sound wave travel time in microseconds

 long distance = duration * 0.034 / 2;          // Calculating the distance
 
 return(distance);
}


/******************************   TCRT5000 sensor Function   ******************************/
bool IsSensorOnWhiteLine(int TCRT5000Pin){
  boolean val = digitalRead(TCRT5000Pin);   // Read the value of TCRT5000 sensor on Pin TCRT5000Pin
  
  if(val == HIGH){                          // If it is HiGH
    return(false);                           // Return false if on the Black area
  }
  else{                                     // If it is LOW
    return(true);                           // Return true if on the White line
  }
}

/**********************************   Motor Go Function   *******************************/
void motorGo(uint8_t direct, uint8_t pwm){
    if(direct == CW)
    {
      digitalWrite(MOTOR_A1_PIN, LOW); 
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if(direct == CCW)
    {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);      
    }
    else
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);            
    }
    
    analogWrite(PWM_MOTOR_1, pwm); 
}
