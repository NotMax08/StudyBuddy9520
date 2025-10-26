#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Servo.h>
#include <Encoder.h>

class ServoController {
public:
    ServoController(int servoPin, int encoderPin1, int encoderPin2);

    void initialize();
    double track();
    double outputPositional(double error);
    double normalizePower(double pidOutput);
    void lockOn(double targetAngle);
    void rotateContinuous(double speed);
    double getError();

    double getAngle();
    double getDerivativeOutput();
    double getTicks();
    double getControllerOutput();
    void reverseDirection();

    void setKp(double newKp);
    void setKd(double newKd);

    void turnOffServo();
    void turnOnServo();

private:
    int servoPin;
    Servo servo;
    Encoder myEnc;
    double power = 0;
    bool reverse = false;

    const long COUNTS_PER_REV = 8192;
    double ticks = 0;
    double angle = 0;

    double Kp = 3.5;
    double Kd = 0;

    bool on = true;

    double derivativeOutput = 0;
    double normalizedOutput = 0;
    double lastError = 0;
    double lastTime = 0;
  
};

#endif