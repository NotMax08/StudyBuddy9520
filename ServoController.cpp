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

void ServoController::turnOffServo(){
    on = false;
}

void ServoController::turnOnServo(){
    on = true;
}

void ServoController::reverseDirection(){
    reverse = true; 
}

void ServoController::setKp(double newKp){
    Kp = newKp;
}

void ServoController::setKd(double newKd){
    Kd = newKd;
}


double ServoController::outputPositional(double error) {
    unsigned long now = millis();
    double deltaTime = (now - lastTime);
    double derivative = (error - lastError) / deltaTime;
    derivativeOutput = derivative * Kd;

    lastError = error;
    lastTime = now;
    return (error * Kp) + (derivative * Kd);
}

double ServoController::normalizePower(double pidOutput) {
    if (!reverse){
        normalizedOutput = 90 - (pidOutput * 30.0 / 500);
    } else {
        normalizedOutput = 90 + (pidOutput * 30.0 / 500);
    }

    if (normalizedOutput < 80){
        normalizedOutput = 81;
    }

    if (normalizedOutput > 110){
        normalizedOutput = 100;
    }
    return normalizedOutput;
}

void ServoController::lockOn(double error) {
    if (on){
        power = normalizePower(outputPositional(error));
    
    } else {
        power = 90;
    }
    servo.write(power);
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

double ServoController::getError() {
    return lastError;
}

double ServoController::getDerivativeOutput() {
    return derivativeOutput;
}

double ServoController::getTicks() {
    return ticks;
}