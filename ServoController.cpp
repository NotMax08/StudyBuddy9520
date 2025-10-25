#include "ServoController.h"
#include <Arduino.h> 

ServoController::ServoController(int servoPin, int encoderPin1, int encoderPin2)
    : myEnc(encoderPin1, encoderPin2) {
    this->servoPin = servoPin;
}

void ServoController::initialize() {
    servo.attach(servoPin);
}

double ServoController::track() {
    if (reverse){
        ticks = myEnc.read();
    } else {
        ticks = -myEnc.read();
    }
    angle = ((float)ticks / (float)COUNTS_PER_REV) * 360.0;
    return angle;
}

void ServoController::reverseDirection(){
    reverse = true; 
}

void ServoController::setKp(double newKp){
    Kp = newKp;
}

double ServoController::outputPositional(double targetPosition, double currentPosition) {
    unsigned long now = millis();
    double deltaTime = (now - lastTime);
    double error = targetPosition - currentPosition;
    double derivative = (error - lastError) / deltaTime;
    derivativeOutput = derivative;

    lastError = error;
    lastTime = now;
    return (error * Kp) + (derivative * Kd);
}

double ServoController::normalizePower(double pidOutput) {
    if (!reverse){
        normalizedOutput = 90 - (pidOutput * 30.0 / 270.0);
    } else {
        normalizedOutput = 90 + (pidOutput * 30.0 / 270.0);
    }
    if (normalizedOutput <0){
        normalizedOutput = 0;
    } else if (normalizedOutput > 180){
        normalizedOutput = 180;
    }
    return normalizedOutput;
}

void ServoController::lockOn(double targetAngle) {
    servo.write(normalizePower(outputPositional(targetAngle, angle)));
}

void ServoController::rotateContinuous(double speed) {
    servo.write(speed);
}

double ServoController::getControllerOutput(){
    return normalizedOutput;
}

double ServoController::getAngle() {
    return angle;
}

double ServoController::getDerivativeOutput() {
    return derivativeOutput;
}

double ServoController::getTicks() {
    return ticks;
}