#include "../inc/motor.h"

Motor::Motor()
    : mAngle(1000)
{
    std::cout << "Default ";
    setupServo(4);
}

Motor::Motor(int gpioPin)
    : mAngle(1000)
{
    setupServo(gpioPin);
}

void Motor::setupServo(int gpioPin)
{
    std::cout << "GPIO pin is set to " << gpioPin << "!\n";
    mGPIO = gpioPin;
    std::cout << "Setting servo to starting angle " << mAngle << "!\n";
    updateServo();
}

void Motor::updateServo()
{
    gpioServo(mGPIO, mAngle);
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