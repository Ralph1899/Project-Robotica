#ifndef MOTOR_H
#define MOTOR_H

#include <pigpio.h>
#include <iostream>
#include "../inc/clock.h"

class Motor
{
private:
    int mGPIO;
    int mAngle;
    void setupServo(int gpioPin);
    void calibration();
    void updateServo();
public:
    Motor();
    Motor(int gpioPin);
    ~Motor() { };

    void setGpioPin(int gpioPin);
    void setAngle(int angle);
};

#endif // !MOTOR_H