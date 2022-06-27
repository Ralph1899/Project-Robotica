#ifndef PID_H
#define PID_H

#include <iostream>
#include <string>
#include <math.h>

class PID
{
private:
    int mMaxDegrees = 10; // Depending on machine config and physical surroundings
    double mKp, mKi, mKd, mDesiredAngle, mThreshold;
    float mLastError = 0, mStoredPID_i = 0;
public:
    PID();
    PID(double Kp, double Ki, double Kd, double desiredAngle, double KiThreshold);
    ~PID() { };

    void setP(double Kp);
    void setI(double Ki);
    void setD(double Kd);
    void setPID(double Kp, double Ki, double Kd);
    void setDesiredAngle(double desiredAngle);
    void setThreshold(double KiThreshold);

    float runPID(double filteredInput, double dT);
};

#endif // !PID_H