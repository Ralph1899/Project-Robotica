#include "../inc/i2c.h"

int i2c::readDevice(int deviceHandle, int address)
{
    int value = i2cReadByteData(deviceHandle, address);
    value <<= 8;
    value += i2cReadByteData(deviceHandle, address + 1);
    if (value >= 0x8000)
        value = -(65536 - value);

    return value;
}