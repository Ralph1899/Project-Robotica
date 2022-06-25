/*
    @author: Ralph Stortenbeker
    @date: 13.06.22
*/

#include <iostream>
#include <math.h>
#include <sys/time.h>

#include "../inc/sensor.h"
#include "../inc/clock.h"
#include "../inc/Kalman.h"

#define M_PI 3.14159265358979323846  /* pi */
#define RAD_TO_DEG 57.29577951308233

#define X 0
#define Y 1
#define Z 2

int motorXDelayActual, motorYDelayActual;
long lastMotorXDelayTime, lastMotorYDelayTime;
int currentStepXA, currentStepXB, currentStepXC;
int currentStepYA, currentStepYB, currentStepYC;
int stepX, stepY;

float pidX_p, pidX_i, pidX_d;
float errorX, prev_errorX;
float PIDX;

float pidY_p, pidY_i, pidY_d;
float errorY, prev_errorY;
float PIDY;

int table[] = {
    128, 130, 132, 134, 136, 138, 139, 141,
    143, 145, 147, 149, 151, 153, 155, 157,
    159, 161, 163, 165, 167, 169, 171, 173,
    174, 176, 178, 180, 182, 184, 185, 187,
    189, 191, 192, 194, 196, 198, 199, 201,
    202, 204, 206, 207, 209, 210, 212, 213,
    215, 216, 218, 219, 220, 222, 223, 224,
    226, 227, 228, 229, 231, 232, 233, 234,
    235, 236, 237, 238, 239, 240, 241, 242,
    243, 244, 245, 245, 246, 247, 247, 248,
    249, 249, 250, 250, 251, 251, 252, 252,
    253, 253, 253, 254, 254, 254, 254, 255,
    255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 254, 254, 254, 254, 253, 253,
    253, 252, 252, 251, 251, 250, 250, 249,
    249, 248, 247, 247, 246, 245, 245, 244,
    243, 242, 241, 240, 239, 238, 237, 236,
    235, 234, 233, 232, 231, 229, 228, 227,
    226, 224, 223, 222, 220, 219, 218, 216,
    215, 213, 212, 210, 209, 207, 206, 204,
    202, 201, 199, 198, 196, 194, 192, 191,
    189, 187, 185, 184, 182, 180, 178, 176,
    174, 173, 171, 169, 167, 165, 163, 161,
    159, 157, 155, 153, 151, 149, 147, 145,
    143, 141, 139, 138, 136, 134, 132, 130,
    128, 125, 123, 121, 119, 117, 116, 114,
    112, 110, 108, 106, 104, 102, 100, 98,
    96, 94, 92, 90, 88, 86, 84, 82,
    81, 79, 77, 75, 73, 71, 70, 68,
    66, 64, 63, 61, 59, 57, 56, 54,
    53, 51, 49, 48, 46, 45, 43, 42,
    40, 39, 37, 36, 35, 33, 32, 31,
    29, 28, 27, 26, 24, 23, 22, 21,
    20, 19, 18, 17, 16, 15, 14, 13,
    12, 11, 10, 10, 9, 8, 8, 7,
    6, 6, 5, 5, 4, 4, 3, 3,
    2, 2, 2, 1, 1, 1, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 1, 1, 1, 2, 2,
    2, 3, 3, 4, 4, 5, 5, 6,
    6, 7, 8, 8, 9, 10, 10, 11,
    12, 13, 14, 15, 16, 17, 18, 19,
    20, 21, 22, 23, 24, 26, 27, 28,
    29, 31, 32, 33, 35, 36, 37, 39,
    40, 42, 43, 45, 46, 48, 49, 51,
    53, 54, 56, 57, 59, 61, 63, 64,
    66, 68, 70, 71, 73, 75, 77, 79,
    81, 82, 84, 86, 88, 90, 92, 94,
    96, 98, 100, 102, 104, 106, 108, 110,
    112, 114, 116, 117, 119, 121, 123, 125
};
int sineArraySize;

double micros()
{
   struct timeval tv;
   double t;

   gettimeofday(&tv, 0);

   t = (double)tv.tv_sec + ((double)tv.tv_usec / 1E6);

   return t;
}

void runPIDX(double kalRoll, double desired_angleX, double dT)
{
    double kxp = 45, kxi = 0.002, kxd = 2;

    errorX = kalRoll - desired_angleX;
    pidX_p = kxp * errorX;

    if (errorX < 5 && errorX > -5)
        pidX_i = pidX_i + (kxi * errorX);
    else
        pidX_i = 0;

    pidX_d = kxd * ((errorX - prev_errorX) / dT);
    PIDX = pidX_p + pidX_i + pidX_d;
    prev_errorX = errorX;

    if (errorX > 0)
        stepX = -1;
    else
        stepX = 1;

    if (PIDX < 5 && PIDX > -5)
        motorXDelayActual = 100000;
    else
        motorXDelayActual = abs(motorXDelayActual - abs(PIDX));

    if (motorXDelayActual < 1000)
        motorXDelayActual = 1000;
}

void runPIDY(double kalPitch, double desired_angleY, double dT)
{
    double kyp = 35, kyi = 0.002, kyd = 1.5;

    errorY = kalPitch - desired_angleY;
    pidY_p = kyp * errorY;

    if (-5 < errorY && errorY < 5)
        pidY_i = pidY_i + (kyi * errorY);
    else
        pidY_i = 0;

    pidY_d = kyd * ((errorY - prev_errorY) / dT);
    PIDY = pidY_p + pidY_i + pidY_d;
    prev_errorY = errorY;

    if (errorY > 0)
        stepY = -1;
    else
        stepY = 1;

    if (PIDY < 5 && PIDY > -5)
        motorYDelayActual = 100000;
    else
        motorYDelayActual = abs(motorYDelayActual - abs(PIDY));

    if (motorYDelayActual < 1000)
        motorYDelayActual = 1000;
}

void runMotorX()
{
    currentStepXA = currentStepXA + stepX;

    if (currentStepXA > sineArraySize)
        currentStepXA = 0;

    if (currentStepXA < 0) 
        currentStepXA = sineArraySize; 
        
    currentStepXB = currentStepXB + stepX; 
    
    if (currentStepXB > sineArraySize) 
        currentStepXB = 0; 
        
    if (currentStepXB < 0) 
        currentStepXB = sineArraySize; 
    
    currentStepXC = currentStepXC + stepX; 
    
    if (currentStepXC > sineArraySize) 
        currentStepXC = 0; 
        
    if (currentStepXC < 0) 
        currentStepXC = sineArraySize; 
        
    std::cout << "curStepXA: " << currentStepXA << "\n";
    std::cout << "curStepXB: " << currentStepXB << "\n";
    std::cout << "curStepXC: " << currentStepXC << "\n";
}

void runMotorY()
{
    currentStepYA = currentStepYA + stepY;

    if(currentStepYB > sineArraySize)
     currentStepYA = 0; 
        
    if (currentStepYA < 0) 
        currentStepYA = sineArraySize; 
        
    currentStepYB = currentStepYB + stepY; 
    
    if (currentStepYB > sineArraySize) 
        currentStepYB = 0; 
        
    if (currentStepYB < 0) 
        currentStepYB = sineArraySize; 
    
    currentStepYC = currentStepYC + stepY; 
    
    if (currentStepYC > sineArraySize) 
        currentStepYC = 0; 
        
    if (currentStepYC < 0) 
        currentStepYC = sineArraySize; 
        
    std::cout << "curStepYA: " << currentStepYA << "\n";
    std::cout << "curStepYB: " << currentStepYB << "\n";
    std::cout << "curStepYC: " << currentStepYC << "\n";
}

int main()
{
    Kalman *pKalmanX = new Kalman(), *pKalmanY = new Kalman();
    MPU_9250 *sensor = new MPU_9250();
    double *readings = new double[3];

    sensor->getAccelReading(readings);
    float accX = readings[X];
    float accY = readings[Y];
    float accZ = readings[Z];

    sensor->getGyrosReading(readings);
    float gyroX = readings[X];
    float gyroY = readings[Y];
    float gyroZ = readings[Z];

    double roll = (180 / M_PI) * atan(accX / sqrt(pow(accY, 2) + pow(accZ, 2)));
    double pitch = (180 / M_PI) * atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2)));

    pKalmanX->setAngle(roll);
    pKalmanY->setAngle(pitch);

    //double gyroXangle = roll;
    //double gyroYangle = pitch;
    //double compAngleX = roll;
    //double compAngleY = pitch;

    double timer = micros(), compRoll, compPitch, gyroYaw;
    double gyroXrate = 0;
    double gyroYrate = 0;

    sineArraySize = sizeof(table) / sizeof(int); 
    int phaseShift = sineArraySize / 3;

    currentStepXA = 0; 
    currentStepXB = currentStepXA + phaseShift; 
    currentStepXC = currentStepXB + phaseShift; 
    
    currentStepYA = 0; 
    currentStepYB = currentStepYA + phaseShift; 
    currentStepYC = currentStepYB + phaseShift; 
    
    sineArraySize--;

    while (true)
    {
	    std::cout << "New loop\n";
        sensor->getAccelReading(readings);
        accX = readings[X];
        accY = readings[Y];
        accZ = readings[Z];

        sensor->getGyrosReading(readings);
        gyroX = readings[X];
        gyroY = readings[Y];
        gyroZ = readings[Z];

        double dT = (double)(micros() - timer) / 1000000;
        timer = micros();

        //Angle from accelorometer
        double roll = (180 / M_PI) * atan(accX / sqrt(pow(accY, 2) + pow(accZ, 2)));
        double pitch = (180 / M_PI) * atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2)));

        // Angle from gyro
        gyroXrate += (gyroX * RAD_TO_DEG) * dT;
        gyroYrate += (gyroY * RAD_TO_DEG) * dT;

        // Angle from Kalman
        double kalRoll = pKalmanX->getAngle(roll, gyroXrate, dT);
        double kalPitch = pKalmanY->getAngle(pitch, gyroYrate, dT);

        //Angle from comp.
        compRoll = (double)0.96 * (compRoll + gyroY * dT) + 0.04 * roll;
        compPitch = (double)0.96 * (compPitch + gyroX * dT) + 0.04 * pitch;
        gyroYaw = (double)(gyroYaw + (gyroZ * dT));

        runPIDX(kalRoll, 0, dT);
        runPIDY(kalPitch, 0, dT);

        std::cout << micros() << "\n";
        std::cout << lastMotorXDelayTime << "\n";
        std::cout << microc() - lastMotorXDelayTime << "\n\n";
        std::cout << motorXDelayActual << "\n";
        if ((micros() - lastMotorXDelayTime) > motorXDelayActual) 
        { 
            runMotorX(); 
            lastMotorXDelayTime = micros();
        }

        if ((micros() - lastMotorYDelayTime) > motorYDelayActual) 
        { 
            runMotorY(); 
            lastMotorYDelayTime = micros(); 
        }

        clock::sleep_milliseconds(100);
    }

    return 0;
}
