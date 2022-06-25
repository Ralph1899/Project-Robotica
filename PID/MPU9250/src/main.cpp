/*
    @author: Ralph Stortenbeker
    @date: 13.06.22
*/

#include <iostream>
#include <math.h>

#include "../inc/sensor.h"
#include "../inc/clock.h"
#include "../inc/Kalman.h"

#define M_PI 3.14159265358979323846  /* pi */
#define RAD_TO_DEG 57.29577951308233 /* 180 / pi */

int main()
{
    // Reserving static memory for IMU sensor to write data to
    double *pSensorBuffer = new double[6];

    // Creating Kalman filter objects for X- and Y-axis
    Kalman *pKalmanX = new Kalman(), *pKalmanY = new Kalman();
    
    // Setting up the MPU9250 sensor class
    MPU_9250 *pSensor = new MPU_9250();
    pSensor->setAccelSensitivity(ACCEL_SENSITIVITY_2G); // Setting sensitivity to 2G (2/4/8/16G is possible)
    pSensor->setGyrosSensitivity(GYRO_SENSITIVITY_250); // Setting sensitivity to 250 (250/500/1000/2000 is possible)
    pSensor->setMemoryBuffer(pSensorBuffer); // Setting the memory location where the sensor can write to

    // Setting pointers to each byte locations the IMU points to. 
    // By updating the memory, these pointers point to the updated value!
    double *pAccelX = &pSensorBuffer[0]; // Memory location [0-2] is used for Acceleration Sensor
    double *pAccelY = &pSensorBuffer[1]; // Memory location [0-2] is used for Acceleration Sensor
    double *pAccelZ = &pSensorBuffer[2]; // Memory location [0-2] is used for Acceleration Sensor
    double *pGyrosX = &pSensorBuffer[3]; // Memory location [3-5] is used for Gyroscope Sensor
    double *pGyrosY = &pSensorBuffer[4]; // Memory location [3-5] is used for Gyroscope Sensor
    double *pGyrosZ = &pSensorBuffer[5]; // Memory location [3-5] is used for Gyroscope Sensor

    // The roll can be calculated with acceleration variables
    // The ptich can be calculated with acceleration variables
    // Source -> http://robo.sntiitk.in/2017/12/21/Beginners-Guide-to-IMU.html
    double roll = (180 / M_PI) * atan2(*pAccelX, sqrt(pow(*pAccelY, 2) + pow(*pAccelZ, 2)));
    double pitch = (180 / M_PI) * atan2(*pAccelY, sqrt(pow(*pAccelX, 2) + pow(*pAccelZ, 2)));

    // Setting the initial Kalman angles for the X- and Y-axis
    pKalmanX->setAngle(roll);
    pKalmanY->setAngle(pitch);

    double gyroXrate = 0;
    double gyroYrate = 0;

    // After initalizing, setting timer to current time
    double timer = clock::micros();

    while (true)
    {
        pSensor->getSensorReading();
        
        double dT = (double)(clock::micros() - timer) / 1000000; 
        timer = clock::micros();
        
        //Angle from accelorometer
        double roll = (180 / M_PI) * atan2(*pAccelX, sqrt(pow(*pAccelY, 2) + pow(*pAccelZ, 2)));
        double pitch = (180 / M_PI) * atan2(*pAccelY, sqrt(pow(*pAccelX, 2) + pow(*pAccelZ, 2)));

        // Angle from gyro
        gyroXrate += (*pGyrosX * RAD_TO_DEG) * dT;
        gyroYrate += (*pGyrosY * RAD_TO_DEG) * dT;

        // Angle from Kalman
        double kalRoll = pKalmanX->getAngle(roll, gyroXrate, dT);
        double kalPitch = pKalmanY->getAngle(pitch, gyroYrate, dT);

        //Angle from comp.
        //compRoll = (double)0.96 * (compRoll + pGyrosY * dT) + 0.04 * roll;
        //compPitch = (double)0.96 * (compPitch + pGyrosX * dT) + 0.04 * pitch;
        //gyroYaw = (double)(gyroYaw + (pGyrosZ * dT));

        std::cout << "Original Roll: " << roll << "\n";
        std::cout << "Filtered Roll: " << kalRoll << "\n\n";

        //std::cout << "Original Pitch: " << pitch << "\n";
        //std::cout << "Filtered Pitch: " << kalPitch << "\n\n";


        clock::sleep_milliseconds(1000);
    }

    return 0;
}
