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
    int mMinimum = 0, mMaximum = 0;

    void setupServo(int gpioPin);
    void updateServo();
public:
    Motor();
    Motor(int gpioPin);
    ~Motor() { setAngle(((mMinimum + mMaximum) / 2)); };

    void setRange(int minimum, int maximum);
    void setGpioPin(int gpioPin);
    void setAngle(int newAngle);

    int getAngle();
    int getMidpoint();
};

#endif // !MOTOR_H