#ifndef KALMAN_H
#define KALMAN_H

class Kalman
{
private:
    float mQ_angle;  // Process noise variance for the accelermeter
    float mQ_bias;   // Process noise variance for the gyro bias
    float mR_measure;// Measurement noise variance

    float mAngle;// The angle calculated by the Kalman filter - part of 2x1 state vector
    float mBias; // The gyro  calculated by the Kalman filter - part of 2x1 state vector
    float mRate; // Unbiased rate calculated from the rate and the calculated bias

    float P[2][2]; // Error covariance matrix
public:
    Kalman();
    ~Kalman() { };

    void setQ_angle(float Q_angle);
    void setQ_bias(float Q_bias);
    void setR_measure(float R_measure);
    void setAngle(float angle);

    float getQ_angle();
    float getQ_bias();
    float getR_measure();
    float getAngle();
    float getRate();
};

#endif // !KALMAN_H