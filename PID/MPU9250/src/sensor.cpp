#include "../inc/sensor.h"

MPU_9250::MPU_9250()
{
    try
    {
        mAccGyroHandler = i2cOpen(I2C_DEVICE, I2C_DEVICE_ADDR, I2C_FLAGS);
        if(mAccGyroHandler < 0)
            throw std::invalid_argument("Failed communication to IMU");
        
        mMagHandler = i2cOpen(I2C_DEVICE, I2C_MAGDEVICE_ADDR, I2C_FLAGS);
        if(mMagHandler < 0)
            throw std::invalid_argument("Failed communication to MAG");
        
        mAccGyroWakeHandler = i2cWriteByteData(mAccGyroHandler, PWR_MGMT_1_ADDR, PWR_MGMT_1_VAL);
        if(mAccGyroWakeHandler < 0)
            throw std::invalid_argument("Failed waking up IMU");

        i2cWriteByteData(mAccGyroHandler, USR_CTRL, 0x20);
/*
        mMagWakeHandler = i2cReadByteData(mMagHandler, MAG_DEVICE_ADDR);
        if(mMagWakeHandler != 72)
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
    std::cout << "Acceleration sensitivity set!\n";
}

void MPU_9250::setGyrosSensitivity(gyros_sensitivity sensitivity)
{
    pGyros = new Gyroscope(sensitivity);
    std::cout << "Gyroscope sensitivity set!\n";
}

void MPU_9250::setMemoryBuffer(double *pMemoryLocation)
{
    pMemoryBuffer = pMemoryLocation;
}

void MPU_9250::getSensorReading()
{
    pMemoryBuffer[0] = pAccel->getAcceleration(mAccGyroHandler, ACCEL_X);
    pMemoryBuffer[1] = pAccel->getAcceleration(mAccGyroHandler, ACCEL_Y);
    pMemoryBuffer[2] = pAccel->getAcceleration(mAccGyroHandler, ACCEL_Z);
    pMemoryBuffer[3] = pGyros->getGyroscope(mAccGyroHandler, GYRO_X);
    pMemoryBuffer[4] = pGyros->getGyroscope(mAccGyroHandler, GYRO_Y);
    pMemoryBuffer[5] = pGyros->getGyroscope(mAccGyroHandler, GYRO_Z);
}