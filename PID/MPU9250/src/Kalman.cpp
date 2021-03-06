#include "../inc/Kalman.h"

Kalman::Kalman()
{
    // Default setup
    mQ_angle = 0.001f;
    mQ_bias = 0.003f;
    mR_measure = 0.03f;

    mAngle = 0.0f;
    mBias = 0.0f;

    mP[0][0] = 0.0f;
    mP[0][1] = 0.0f;
    mP[1][0] = 0.0f;
    mP[1][1] = 0.0f;
}

void Kalman::setAngle(float angle)
{
    mAngle = angle;
}

void Kalman::setQ_angle(float Q_angle)
{
    mQ_angle = Q_angle;
}

void Kalman::setQ_bias(float Q_bias)
{
    mQ_bias = Q_bias;
}

void Kalman::setR_measure(float R_measure)
{
    mR_measure = R_measure;
}

float Kalman::getAngle(float newAngle, float newRate, float deltaTime)
{
    // Step 1:
    mRate = newRate - mBias;
    mAngle += deltaTime * mRate;

    // Step 2:
    mP[0][0] += deltaTime * (deltaTime * mP[1][1] - mP[0][1] - mP[1][0] + mQ_angle); 
    mP[0][1] -= deltaTime * mP[1][1];
    mP[1][0] -= deltaTime * mP[1][1]; 
    mP[1][1] += deltaTime * mQ_bias;

    // Step 3: Innovation
    float y = newAngle - mAngle;

    // Step 4: Innovation covariance
    float S = mP[0][0] + mR_measure;
    
    // Step 5: Kalman Gain
    float K[2];
    K[0] = mP[0][0] / S;
    K[1] = mP[1][0] / S;

    // Step 6: Update the Angle
    mAngle = K[0] * y;
    mBias = K[1] * y;

    // Step 7: Calculate estimation error covariance - Update the error covariance
    float P00_temp = mP[0][0];
    float P01_temp = mP[0][1];

    mP[0][0] -= K[0] * P00_temp;
    mP[0][1] -= K[0] * P01_temp;
    mP[1][0] -= K[1] * P00_temp;
    mP[1][1] -= K[1] * P01_temp;

    // Step 8: Return filtered value
    return mAngle;
}

float Kalman::getQ_angle()
{
    return mQ_angle;
}

float Kalman::getQ_bias()
{
    return mQ_bias;
}

float Kalman::getR_measure()
{
    return mR_measure;
}

float Kalman::getRate()
{
    return mRate;
}
