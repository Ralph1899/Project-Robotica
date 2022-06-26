#ifndef PID_H
#define PID_H

#include <math.h>

class PID
{
private:
    double mKp, mKi, mKd;
    float mError;
    int mStep, mMotorDelay;
public:
    PID();
    PID(double Kp, double Ki, double Kd);
    ~PID() { };

    void setP(double Kp);
    void setI(double Ki);
    void setD(double Kd);
    void setPID(double Kp, double Ki, double Kd);
    int getStep();
    int getMotorDelay();

    float runPID(double filteredInput, double desiredAngle, double dT);
};

#endif // !PID_H