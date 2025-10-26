#include <Servo.h>
#include <Encoder.h>
#include "ServoController.h" // Assuming this library is installed
#include <math.h>
#include <Wire.h>
#include <Arduino.h>  

// --- FSM Configuration ---
// CRITICAL: Must match the LOST_FLAG in the Python script
const double LOST_FLAG = 9999.0; 
// How close to center (0 error) before the robot stops moving (in pixels)
const int CENTER_THRESHOLD = 200; 

// FSM States
enum State {
    STATE_WAITING, // Motor stopped, target centered or lost
    STATE_TRACKING // Motor active, using PID to follow targetX
};

State currentState = STATE_WAITING;
bool isTargetLost = false; // Flag set when LOST_FLAG is received

// Flag to control one-time shooting action
bool canShoot = false; 

// --- PID Configuration ---
ServoController yaw(3,11,12); 
Servo pitch;
double KpYaw = 0.2;
double KdYaw = 30;

// The target coordinates (setpoints) coming from Python
double targetX = 0;
double targetY = 0;

// --- Motor Pins ---
const int DIR_A = 7; 
const int DIR_B = 8; 
const int RPWM = 6; 
const int LWPM = 5; 

// --- Serial Communication Variables ---
String inputString = "";         
bool stringComplete = false;     
const char TERMINATOR_CHAR = '\n'; 

// Function Prototypes
void readAndParseCoordinates_NonBlocking();
void printStatus();
void setMotorSpeed(int speed);
void clearIntegralTerm(); // Placeholder for clearing PID windup

void setup() {
  Wire.begin();
  yaw.initialize();
  yaw.reverseDirection();
  yaw.setKp(KpYaw);
  yaw.setKd(KdYaw);
  pitch.attach(2);

  Serial.begin(9600);
  Serial.setTimeout(0); // CRITICAL: Non-blocking serial read

  // Motor Pin Setup
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LWPM, OUTPUT);

  digitalWrite(DIR_A, HIGH);
  digitalWrite(DIR_B, HIGH);
}

void loop() {
  // 1. Check for incoming data
  pitch.write(70);
  readAndParseCoordinates_NonBlocking();
  if (targetX == 0){
    yaw.turnOffServo();
    Serial.println("bye"); 
    // currentState = STATE_WAITING; 
  } else {
    yaw.turnOnServo();
    if (abs(targetX) > CENTER_THRESHOLD){
      Serial.println("SHOOOOOOOOOOT");
    }
  }
  yaw.lockOn(targetX);
  
  // // 2. FSM State Transition Logic
  // switch (currentState) {
    
  //   case STATE_WAITING:
  //     //setMotorSpeed(0); // Motor is explicitly stopped in WAITING state
      
  //     // *** SHOOTING ACTION: Executes once when target is centered ***
  //     if (canShoot) {
  //         //put shooting balls
  //         canShoot = false; // reset the flag after shooting
  //     }

  //     if (!isTargetLost && abs(targetX) > CENTER_THRESHOLD) {
  //       yaw.turnOnServo();
  //       currentState = STATE_TRACKING;
  //     }

  //   break; 
      
  //   case STATE_TRACKING:

  //     if (isTargetLost) {
  //       currentState = STATE_WAITING; 
  //       yaw.turnOffServo();
  //       break; 
  //     }
      
  //     if (abs(targetX) <= CENTER_THRESHOLD) {
  //       canShoot = true;
        
  //       currentState = STATE_WAITING;
  //       break; 
  //     }
      
  //     // If still tracking, perform PID calculation and apply motor speed
  //     if (targetX != 0){
  //       yaw.turnOffServo();
  //       yaw.lockOn(targetX);
  //       currentState = STATE_WAITING; 
  //     }
  //     //setMotorSpeed(yaw.getControllerOutput()); 
  //     break;
  // }
  
  printStatus();
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
  if (speed > 0) {
    // Forward (adjust pins for your motor driver setup)
    analogWrite(LWPM, speed);
    analogWrite(RPWM, 0);
  } else if (speed < 0) {
    // Reverse (adjust pins for your motor driver setup)
    analogWrite(LWPM, 0);
    analogWrite(RPWM, -speed); 
  } else {
    // Stop
    analogWrite(LWPM, 0);
    analogWrite(RPWM, 0);
  }
}
