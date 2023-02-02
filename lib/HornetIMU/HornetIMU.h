/**
 * @file      HornetIMU.h
 * @brief
 * @details
 * @author    Ian Sodersjerna
 * @version   0.0.1
 * @date      2022
 * @pre
 * @bug
 * @warning
 * @copyright GNU Public License.
 */
#ifndef FLIGHT_CONTROLLER_HORNET_IMU_H
#define FLIGHT_CONTROLLER_HORNET_IMU_H
#include <Arduino.h>
#include <config.h>
#include <HornetMODULE.h>
#include <HornetLOGGER.h>

#ifdef IMU_JY901
#include "JY901.h"
#endif // IMU_JY901


class HornetIMU : public HornetModule
{
    enum axis
    {
        X,
        Y,
        Z
    };

public:
    HornetIMU(HornetLOGGER *logger);
    void begin(float *GyroX, float *GyroY, float *GyroZ, float *accX, float *accY, float *accZ, float *magX, float *magY, float *magZ, float *angleX, float *angleY, float *angleZ);
    void update();
    void print();

private:
#ifdef IMU_JY901
    CJY901 *imu;
#endif // IMU_JY901

    float *gyroX, *gyroY, *gyroZ;
    float *accX, *accY, *accZ;
    float *magX, *magY, *magZ;
    float *angleX, *angleY, *angleZ;
};

#endif // FLIGHT_CONTROLLER_HORNET_IMU_H
