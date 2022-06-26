#include "../inc/motor.h"

Motor::Motor()
    : mAngle(500)
{
    std::cout << "Default ";
    setupServo(4);
}

Motor::Motor(int gpioPin)
    : mAngle(500)
{
    setupServo(gpioPin);
}

void Motor::setupServo(int gpioPin)
{
    std::cout << "GPIO pin is set to " << gpioPin << "!\n";
    mGPIO = gpioPin;
    std::cout << "Setting servo to starting angle " << mAngle << "!\n";
    updateServo();
    clock::sleep_milliseconds(10);
}

void Motor::updateServo()
{
    gpioServo(mGPIO, mAngle);
}

void Motor::setRange(int minimum, int maximum)
{
    mMinimum = minimum;
    mMaximum = maximum;

    // Visualizing the values
    gpioServo(mGPIO, mMinimum);
    clock::sleep_milliseconds(100);
    gpioServo(mGPIO, mMaximum);
    clock::sleep_milliseconds(100);
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