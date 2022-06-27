#include "../inc/pid.h"

PID::PID()
{ 
    setPID(0, 0, 0);
    setDesiredAngle(0);
    setThreshold(1);
}

PID::PID(double Kp, double Ki, double Kd, double desiredAngle, double KiThreshold)
{
    setPID(Kp, Ki, Kd);
    setDesiredAngle(desiredAngle);
    setThreshold(KiThreshold);
}

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
    setP(Kp);
    setI(Ki);
    setD(Kd);
}

void PID::setDesiredAngle(double desiredAngle)
{
    try
    {
        if((desiredAngle < -mMaxDegrees) || (desiredAngle > mMaxDegrees))
        {
            std::string errorString = "Desired angle cannot exceed " + (std::string)mMaxDegrees + " degrees!";
            throw std::invalid_argument(errorString);
        }
        mDesiredAngle = desiredAngle;
    }
    catch(const std::exception& e)
    {
        std::cerr << "PID : " << e.what() << '\n';
    }
}

void PID::setThreshold(double KiThreshold)
{
    try
    {
        if(KiThreshold == 0)
            throw std::invalid_argument("Threshold cannot be zero!");
        mThreshold = KiThreshold;
    }
    catch(const std::exception& e)
    {
        std::cerr << "PID : " << e.what() << '\n';
    }
}

float PID::runPID(double filteredInput, double dT)
{
    // PID = PID_p + PID_i + PID_d
    // PID_p = Kp * error
    // PID_i = PID_i + (Ki * error)
    // PID_d = Kd * (error - previousError) / deltaTime

    float PID, PID_p, PID_i, PID_d;
    float currentError = filteredInput - mDesiredAngle;

    // PID_p = Kp * error
    PID_p = mKp * currentError;

    // Ki part is only used if the error reading is between a given threshold.
    // When error is not between the threshold, Ki part is zero.
    PID_i = 0;
    // PID_i = PID_i + (Ki * error) -> Only if error is between a given threshold
    if((currentError < mThreshold) && (currentError > -mThreshold))
        // Overwrite the 0 value
        PID_i = mStoredPID_i + (mKi * currentError);
    
    // PID_d = Kd * (error - previousError) / deltaTime
    PID_d = mKd * (currentError - mLastError) / dT;

    // Saving the values of currentError and PID_i so they can be used in next call
    mLastError = currentError;
    mStoredPID_i = PID_i;
    
    // PID = PID_p + PID_i + PID_d    
    PID = PID_p + PID_i + PID_d;

    return PID;
}