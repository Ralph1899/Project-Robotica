#ifndef MOTOR_H
#define MOTOR_H

class Motor
{
private:
    int mGPIO;
    int mAngle;
    void setupServo(int gpioPin);
    void updateServo();
public:
    Motor();
    Motor(int gpioPin);
    ~Motor() { };

    void setGpioPin(int gpioPin);
    void setAngle(int angle);
};

#endif // !MOTOR_H