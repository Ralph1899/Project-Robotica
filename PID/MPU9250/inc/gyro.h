#ifndef GYRO_H
#define GYRO_H

#include "../inc/i2c.h"

//defines self test addresses for gyro
#define SELF_TEST_X_GYRO_ADDR 0x00
#define SELF_TEST_Y_GYRO_ADDR 0x01
#define SELF_TEST_Z_GYRO_ADDR 0x02

//defines memory address for register responsible for configuring gyroscope
#define GYRO_CONFIG_ADDR 0X1B

//defines memory addresses for gyroscope output HIGH registers

//define scales for gyro
#define GYRO_FSCALE_250 0x00
#define GYRO_FSCALE_500 0x01
#define GYRO_FSCALE_1000 0x02
#define GYRO_FSCALE_2000 0x03

enum gyros_sensitivity
{
    GYRO_SENSITIVITY_250 = 1310,
    GYRO_SENSITIVITY_500 = 655,
    GYRO_SENSITIVITY_1000 = 328,
    GYRO_SENSITIVITY_2000 = 164
};

enum gyros_axis
{
    GYRO_X = 0x43,
    GYRO_Y = 0x45,
    GYRO_Z = 0x47
};

class Gyroscope
{
private:
    double scaleFactor;
    double scaleGyro(int raw);
public:
    Gyroscope(gyros_sensitivity sensitivity);
    ~Gyroscope() { };

    double getGyroscope(int sensor, gyros_axis axis);

};

#endif // !GYRO_H