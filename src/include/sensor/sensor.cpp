/*
 * This file constains the implementation of the functions defined
 * in sensors.h
 */

#include "sensor.h"


void sensor::reset()
{
    for (unsigned i = 0; i < 3; i++) {
        acc[i] = 0;
        gyro[i] = 0;
        mag[i] = 0;
    }
}

void sensor::get_acc_data(float* data)
{   
    if (acc_on)
        for (unsigned i = 0; i < 3; i++)
            data[i] = acc[i];
}

void sensor::get_gyro_data(float* data)
{
    if (gyro_on)
        for (unsigned i = 0; i < 3; i++)
            data[i] = gyro[i];
}

void sensor::get_mag_data(float* data)
{
    if (mag_on)
        for (unsigned i = 0; i < 3; i++)
            data[i] = mag[i];
}
