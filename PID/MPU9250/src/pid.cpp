#include "../inc/pid.h"

PID::PID()
    : mKp(0), mKi(0), mKd(0), mError(0) { }

PID::PID(double Kp, double Ki, double Kd)
    : mKp(Kp), mKi(Ki), mKd(Kd), mError(0) { }

void PID::setP(double Kp)
{
    mKp = Kp;
}

void PID::setI(double Ki)
{
    mKi = Ki;
}

void PID::setD(double Kd)
{
    mKd = Kd;
}

void PID::setPID(double Kp, double Ki, double Kd)
{
    mKp = Kp;
    mKi = Ki;
    mKd = Kd;
}

int PID::getStep()
{
    return mStep;
}

int PID::getMotorDelay()
{
    return mMotorDelay;
}

void PID::runPID(double filteredInput, double desiredAngle, double dT)
{
    float kp, ki, kd, k_pid;

    float error = filteredInput - desiredAngle;
    kp = mKp * error;

    if((error < 5) && (error > -5))
        ki += (mKi * error);
    else
        ki = 0;

    kd = mKd * ((error - mError) / dT);
    mError = error;

    k_pid = kp + ki + kd;

    if(error > 0)
        mStep = -1;
    else
        mStep = 1;

    if((k_pid < 5) && (k_pid > -5))
        mMotorDelay = 100000;
    else
        mMotorDelay = abs(mMotorDelay - abs(k_pid));
    
    if(mMotorDelay < 1000)
        mMotorDelay = 1000;
}