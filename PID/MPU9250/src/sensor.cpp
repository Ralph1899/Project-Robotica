#include "../inc/sensor.h"

MPU_9250::MPU_9250()
{
    try
    {
        if(gpioInitialise() < 0)
            throw std::invalid_argument("pigpio.h failed");

        accel_gyro_handler = i2cOpen(I2C_DEVICE, I2C_DEVICE_ADDR, I2C_FLAGS);
        if(accel_gyro_handler < 0)
            throw std::invalid_argument("Failed communication to IMU");
        
        mag_handler = i2cOpen(I2C_DEVICE, I2C_MAGDEVICE_ADDR, I2C_FLAGS);
        if(mag_handler < 0)
            throw std::invalid_argument("Failed communication to MAG");
        
        wake_handler = i2cWriteByteData(accel_gyro_handler, PWR_MGMT_1_ADDR, PWR_MGMT_1_VAL);
        if(wake_handler < 0)
            throw std::invalid_argument("Failed waking up IMU");

        i2cWriteByteData(accel_gyro_handler, USR_CTRL, 0x20);
/*
        mag_wake_handler = i2cReadByteData(mag_handler, MAG_DEVICE_ADDR);
        if(mag_wake_handler != 72)
            throw std::invalid_argument("MAG not found");
*/
        clock::sleep_milliseconds(100);

        std::cout << "Initialised Sensor, please consider setting sensitivity!\n";
    }
    catch(const std::exception& e)
    {
        std::cerr << "MPU_9250 : " << e.what() << '\n';
    }
}

void MPU_9250::setAccelSensitivity(accel_sensitivity sensitivity)
{
    pAccel = new Accelerometer(sensitivity);
    std::cout << "Acceleration sensitivity set: " << sensitivity << "\n";
}

void MPU_9250::setGyrosSensitivity(gyros_sensitivity sensitivity)
{
    pGyros = new Gyroscope(sensitivity);
    std::cout << "Gyroscope sensitivity set: " << sensitivity << "\n";
}

void MPU_9250::setMemoryBuffer(double *pMemoryLocation)
{
    pMemoryBuffer = pMemoryLocation;
}

void MPU_9250::getSensorReading()
{
    pMemoryBuffer[0] = pAccel->getAcceleration(accel_gyro_handler, ACCEL_X);
    pMemoryBuffer[1] = pAccel->getAcceleration(accel_gyro_handler, ACCEL_Y);
    pMemoryBuffer[2] = pAccel->getAcceleration(accel_gyro_handler, ACCEL_Z);
    pMemoryBuffer[3] = pGyros->getGyroscope(accel_gyro_handler, GYRO_X);
    pMemoryBuffer[4] = pGyros->getGyroscope(accel_gyro_handler, GYRO_Y);
    pMemoryBuffer[5] = pGyros->getGyroscope(accel_gyro_handler, GYRO_Z);
}