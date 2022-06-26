#include "../inc/motor.h"

Motor::Motor()
{
    std::cout << "Default ";
    setupServo(4);
}

Motor::Motor(int gpioPin)
{
    setupServo(gpioPin);
}

void Motor::setupServo(int gpioPin)
{
    std::cout << "GPIO pin is set to " << gpioPin << "!\n";
    mGPIO = gpioPin;
}

void Motor::updateServo()
{
    gpioServo(mGPIO, mAngle);
}

void Motor::setRange(int minimum, int maximum)
{
    mMinimum = minimum;
    mMaximum = maximum;

    // Setting the servo exactly in the middle of the given range
    setAngle(((mMinimum + mMaximum) / 2));
}

void Motor::setGpioPin(int gpioPin)
{
    mGPIO = gpioPin;
}

void Motor::setAngle(int angle)
{
    mAngle = angle;
    updateServo();
}