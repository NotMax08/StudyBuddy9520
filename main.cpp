#include <Servo.h>
#include <Encoder.h>
#include "ServoController.h"
#include <math.h>
#include <Wire.h>
#include <Arduino.h>

//red is ground, white is 5volt, blue is A, purple is B
ServoController yaw(3,2,4);
double KpYaw = 7;

double targetYaw = 0;


bool started = false;

String inputString = "";

void setup() {
  Wire.begin();

  yaw.initialize();

  Serial.begin(9600);
  Serial.println("Initializing");

  yaw.reverseDirection(); 
  yaw.setKp(KpYaw);

  delay(2000);
}

void loop() {
  if (Serial.available() > 0) {
    String message = Serial.readString();

    inputString = Serial.readStringUntil('\n');
    inputString.trim();

    Serial.println("90");
    targetYaw = 90;
    // if (inputString.equalsIgnoreCase("start")) {
    //   Serial.println("Program started!");
    //   started = true;
    // } else if (inputString.equalsIgnoreCase("1")) {
    //   Serial.println("90");
    //   targetYaw = 90;
    // }


  }

  if (started){
    yaw.track();
    yaw.lockOn(targetYaw);

    Serial.print("Yaw Angle: ");
    Serial.print(yaw.getAngle());
    Serial.print(" | Yaw Derivative: ");
    Serial.print(yaw.getDerivativeOutput());
    Serial.print(" | Yaw Target: ");
    Serial.println(targetYaw);

  }

}
