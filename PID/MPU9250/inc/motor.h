#ifndef MOTOR_H
#define MOTOR_H

class Motor
{
private:
    int mGPIO;
public:
    Motor();
    Motor(int GpioPin);
    ~Motor() { };
};

#endif // !MOTOR_H