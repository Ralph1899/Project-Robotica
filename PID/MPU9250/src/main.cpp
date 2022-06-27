/*
    @author: Ralph Stortenbeker
    @date: 13.06.22
*/

#include <iostream>
#include <pigpio.h>
#include <signal.h>

#include "../inc/sensor.h"
#include "../inc/clock.h"
#include "../inc/Kalman.h"
#include "../inc/pid.h"
#include "../inc/motor.h"

#define M_PI 3.14159265358979323846  /* pi */
#define RAD_TO_DEG 57.29577951308233 /* 180 / pi */

bool isRunning = true;

void stopProgram(int signum)
{
    isRunning = false;
}

double calculate_roll(double accX, double accY, double accZ)
{
    return ((180 / M_PI) * atan2(accX, sqrt(pow(accY, 2) + pow(accZ, 2))));
}

double calculate_pitch(double accX, double accY, double accZ)
{
    return calculate_roll(accY, accX, accZ);
}

/*
    // Magnetometer not implemented yet...
    double calculate_yaw(magX, magY)
    {
        return ((180 / M_PI) * atan2(magX, magY));
    }
*/

int main()
{
    try
    {
        if(gpioInitialise() < 0)
            throw std::invalid_argument("pigpio.h failed");
        gpioSetSignalFunc(SIGINT, stopProgram); // Setting the handler when CTRL+C is being pressed
    }
    catch(const std::exception& e)
    {
        std::cerr << "Main : " << e.what() << '\n';
    }

    // Reserving static memory for IMU sensor to write data to
    double *pSensorBuffer = new double[6]; // When only accelorometer is used
    //double *pSensorBuffer = new double[9]; // When magnetometer is implemented

    // Creating Servo motor objects for X- and Y-axis
    Motor *pServoX = new Motor(4), *pServoY = new Motor(17);
    pServoX->setRange(500, 1675);
    pServoY->setRange(500, 1675);

    // Creating Kalman filter objects for X-, Y- and Z-axis
    Kalman *pKalmanX = new Kalman(), *pKalmanY = new Kalman()/*, *pKalmanZ = new Kalman()*/;

    // Creating PID objects for X-, Y- and Z-axis
    PID *pPIDX = new PID(), *pPIDY = new PID()/*, *pPIDZ = new PID()*/;

    //pPIDX->setP(2);
    //pPIDX->setI(0.002);
    //pPIDX->setD(2);
    pPIDX->setPID(2, 0.002, 0.01);
    pPIDX->setDesiredAngle(1);
    pPIDX->setThreshold(3);

    //pPIDY->setP(2);
    //pPIDY->setI(0.002);
    //pPIDY->setD(1.5);
    pPIDY->setPID(2, 0.002, 0.01);
    pPIDY->setDesiredAngle(0);
    pPIDY->setThreshold(3);

    //pPIDZ->setP(10);
    //pPIDZ->setI(0.00);
    //pPIDZ->setD(10);
    //pPIDZ->setPID(10, 0.00, 10);
    //pPIDZ->setDesiredAngle(0);
    //pPIDZ->setThreshold(5);
    
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
    //double *pGyrosZ = &pSensorBuffer[5]; // Memory location [3-5] is used for Gyroscope Sensor
    //double *pMagneX = &pSensorBuffer[6]; // Memory location [6-8] is used for Magnetometer
    //double *pMagneY = &pSensorBuffer[7]; // Memory location [6-8] is used for Magnetometer
    //double *pMagneZ = &pSensorBuffer[8]; // Memory location [6-8] is used for Magnetometer


    // The roll can be calculated with acceleration variables
    // Source -> http://robo.sntiitk.in/2017/12/21/Beginners-Guide-to-IMU.html
    double roll = calculate_roll(*pAccelX, *pAccelY, *pAccelZ);

    // The pitch can be calculated with acceleration variables
    // Source -> http://robo.sntiitk.in/2017/12/21/Beginners-Guide-to-IMU.html
    double pitch = calculate_pitch(*pAccelX, *pAccelY, *pAccelZ);

    // The yaw can be calculated with magnitude measurements in X- and Y-direction
    // Source ->
    //double yaw = calculate_yaw(*pMagneX, *pMagneY);

    pKalmanX->setAngle(roll); // Setting the initial Kalman angle for the X-axis
    pKalmanY->setAngle(pitch); // Setting the initial Kalman angle for the Y-axis
    //pKalmanZ->setAngle(yaw); // Setting the initial Kalman angle for the Z-axis

    // After initalizing, setting timer to current time
    std::chrono::steady_clock::time_point timer = clock::current_time_ms();
    double dT;

    //double compRoll, compPitch/*, gyroYaw*/;
    
    clock::sleep_milliseconds(2000);
    while (isRunning)
    {
        // Send trigger to sensor to update values in memory
        pSensor->getSensorReading();

        // Know the passed time since previous measurement (dT in ms)
        dT = clock::time_difference_ms(timer) / 1000; // deviding by 1000 for value in seconds (needed in kalman)
        timer = clock::current_time_ms(); // Updating the current time

        //Angle from accelorometer
        roll = calculate_roll(*pAccelX, *pAccelY, *pAccelZ);
        pitch = calculate_pitch(*pAccelX, *pAccelY, *pAccelZ);
    
        // Angle from magnetometer
        //yaw = calculate_yaw(*pMagneX, *pMagneY);

        // Angle from gyro
        double gyroXrate = (((*pGyrosX) * RAD_TO_DEG) * dT);
        double gyroYrate = (((*pGyrosY) * RAD_TO_DEG) * dT);
        //double gyroZrate = (((*pGyrosZ) * RAD_TO_DEG) * dT);

        // Angle from Kalman
        double kalRoll = pKalmanX->getAngle(roll, gyroXrate, dT);
        double kalPitch = pKalmanY->getAngle(pitch, gyroYrate, dT);
        //double kalYaw = pKalmanZ->getAngle(yaw, gyroZrate, dT);

        // Angle from comp.
        //compRoll = (0.96 * (compRoll + *pGyrosY * dT) + 0.04 * roll);
        //compPitch = (0.96 * (compPitch + *pGyrosX * dT) + 0.04 * pitch);
        //gyroYaw = (gyroYaw + (pGyrosZ * dT));

        // Calculate PID values for each axis
        float x = pPIDX->runPID(kalRoll, dT);
        float y = pPIDY->runPID(kalPitch, dT);
        //float z = pPIDZ->runPID(gyroYaw, dT);


        // Run the motors based on PID results
        pServoX->setAngle((pServoX->getAngle() + x));
        pServoY->setAngle((pServoY->getAngle() - y));
        //pServoZ->setAngle((pServoZ->getAngle() + z));
        clock::sleep_milliseconds(10);

    }

    std::cout << "\n\n#####################################\n";
    std::cout << "Setting Servos back to middle of range!\n";
    std::cout << pServoX->getMidpoint() << std::endl;
    pServoX->setAngle(pServoX->getMidpoint());
    pServoY->setAngle(pServoY->getMidpoint());
    std::cout << "Tidying up used GPIO pins!\n";
    gpioTerminate();
    std::cout << "GPIO is terminated!\n";
    std::cout << "#####################################\n\n";
    return 0;
}
