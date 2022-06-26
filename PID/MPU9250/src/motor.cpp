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
    calibration();
}

void Motor::calibration()
{
    for (int i = 1700; i < 1725; i++)
    {
        gpioServo(mGPIO, i);
        std::cout << "Servo value: " << i << "\n";
        clock::sleep_milliseconds(1000);
    }

    clock::sleep_milliseconds(1000);
    std::cout << "\n\nMax servo value, returning to start\n\n"; 
    clock::sleep_milliseconds(1000);

    for (int i = 510; i > 490; i--)
    {
        gpioServo(mGPIO, i);
        std::cout << "Servo value: " << i << "\n";
        clock::sleep_milliseconds(1000);
    }
}

void Motor::updateServo()
{
    gpioServo(mGPIO, mAngle);
}

void Motor::setRange(int minimum, int maximum)
{
    mMinimum = minimum;
    mMaximum = maximum;
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