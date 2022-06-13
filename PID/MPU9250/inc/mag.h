#ifndef MAG_H
#define MAG_H

#include "../inc/i2c.h"

#define MAG_DEVICE_ADDR 0x00
#define MAG_DEVICE_INFO_ADDR 0x01
#define MAG_DEVICE_STATUS_1_ADDR 0x02
#define MAG_DEVICE_CNTRL_ADDR 0x0A

#define MAG_XOUT_L_ADDR 0x03
#define MAG_YOUT_L_ADDR 0x04
#define MAG_ZOUT_L_ADDR 0x05



#define MAG_XSENSITIVITY_ADDR 0x10
#define MAG_YSENSITIVITY_ADDR 0x11
#define MAG_ZSENSITIVITY_ADDR 0x11

enum mag_axis
{
    MAG_X = 0x04,
    MAG_Y = 0x06,
    MAG_Z = 0x08
};

class Magnetometer
{
private:

public:
    Magnetometer();
    ~Magnetometer() { };

    double getMagnetometer(int sensor, mag_axis axis);
};

#endif // !MAG_H