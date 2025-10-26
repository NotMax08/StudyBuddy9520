#include <Servo.h>
#include <Encoder.h>
#include "ServoController.h"
#include <math.h>
#include <Wire.h>
#include <Arduino.h>  



const int CENTER_THRESHOLD = 200; 

// FSM States
enum State {
    STATE_WAITING,
    STATE_TRACKING
};

State currentState = STATE_WAITING;
bool isTargetLost = false; 


bool canShoot = false; 


ServoController yaw(3,11,12); 
Servo pitch;
double KpYaw = 0.2;
double KdYaw = 30;


double targetX = 0;
double targetY = 0;


const int DIR_A = 11; 
const int DIR_B = 10; 
const int LWPM = 9;


String inputString = "";         
bool stringComplete = false;     
const char TERMINATOR_CHAR = '\n'; 

unsigned long sequence_period = 2000; // 2 seconds
unsigned long previousMillis = 0; 
bool isYawAt90 = false;

void readAndParseCoordinates_NonBlocking();
void printStatus();
void setMotorSpeed(int speed);
void firstPart();
void secondPart();

void setup() {
  Wire.begin();
  yaw.initialize();
  yaw.reverseDirection();
  yaw.setKp(KpYaw);
  yaw.setKd(KdYaw);
  pitch.attach(2);

  Serial.begin(9600);
  Serial.setTimeout(0); 

  // Motor Pin Setup
  //pinMode(RPWM, OUTPUT);
  //pinMode(LWPM, OUTPUT);

  //digitalWrite(DIR_A, HIGH);
  //digitalWrite(DIR_B, HIGH);
}

void firstPart(){
 pitch.write(70);
  readAndParseCoordinates_NonBlocking();
  if (targetX == 0){
    yaw.turnOffServo();
    Serial.println("bye"); 
    // currentState = STATE_WAITING; 
  } else {
    yaw.turnOnServo();
    if (abs(targetX) > CENTER_THRESHOLD){
      setMotorSpeed(255);
      
    }
  }
  yaw.lockOn(targetX);

  printStatus();
}

void secondPart(){
  unsigned long currentMillis = millis();

  // Check if 2000 milliseconds (2 seconds) have passed
  if (currentMillis - previousMillis >= sequence_period) {
    previousMillis = currentMillis;
    isYawAt90 = !isYawAt90;
  }

  if (isYawAt90){
    yaw.rotateContinuous(120);
  } else {
    yaw.rotateContinuous(60);
  }

  int pitchTarget = isYawAt90 ? 90 : 120;
  pitch.write(pitchTarget);

}

void loop() {
  //secondPart();
  firstPart();
}


void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read(); 
    if (inChar == TERMINATOR_CHAR) {
      stringComplete = true; 
    } else {
      inputString += inChar; 
    }
  }
}

void readAndParseCoordinates_NonBlocking() {
  if (stringComplete) {
    int commaIndex = inputString.indexOf(',');
    if (commaIndex != -1) {
        String xStr = inputString.substring(0, commaIndex);
        String yStr = inputString.substring(commaIndex + 1);
        targetX = xStr.toDouble(); 
        targetY = yStr.toDouble();
    }
    inputString = "";
    stringComplete = false;
  }
}



void printStatus() {
  Serial.print("STATE: ");
  Serial.print(currentState == STATE_WAITING ? "WAITING" : "TRACKING");
  Serial.print(" | TargetX: ");
  Serial.print(targetX); 
  Serial.print(" | Output: ");
  Serial.print(yaw.getControllerOutput()); 
  Serial.print(" | cy:");
  Serial.println(90 + (targetY * (30/400)));
}

void setMotorSpeed(int speed) {
  // Constrain speed to a safe range
  speed = constrain(speed, -255, 255);
  // Magnitude is the absolute value of the speed
  int magnitude = abs(speed);
  
  if (speed > 0) {
    // Forward: DIR_A HIGH, DIR_B LOW
    digitalWrite(DIR_A, HIGH);
    digitalWrite(DIR_B, LOW);
  } else if (speed < 0) {
    // Reverse: DIR_A LOW, DIR_B HIGH
    digitalWrite(DIR_A, LOW);
    digitalWrite(DIR_B, HIGH); 
  } else {
    // Stop/Brake: Both pins LOW (or both HIGH for quick brake, but LOW is safer/slower)
    digitalWrite(DIR_A, LOW);
    digitalWrite(DIR_B, LOW);
  }

  // Set the magnitude (speed) on the PWM pin
  analogWrite(LWPM, magnitude);
}