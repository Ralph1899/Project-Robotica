#ifndef I2C_H
#define I2C_H

#include <pigpio.h>

#define I2C_DEVICE 1
#define I2C_DEVICE_ADDR 0x68
#define I2C_MAGDEVICE_ADDR 0X0C
#define I2C_FLAGS 0

#define USR_CTRL 0X6A

#define PWR_MGMT_1_ADDR 0x68
#define PWR_MGMT_1_VAL 0x00

class i2c
{
public:
    static int readDevice(int deviceHandle, int address);
};

#endif // !I2C_H