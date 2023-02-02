/**
 * @file      HornetIMU.cpp
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
#include <Arduino.h>
#include "config.h"
#include "HornetIMU.h"

HornetIMU::HornetIMU(HornetLOGGER *logger) : HornetModule(logger)
{
    /**
     * @brief Initialize IMU objects.
     * 
     */
    this->logger->debug("HornetIMU init!");

#ifdef IMU_JY901
    this->imu = new CJY901();
#endif // IMU_JY901

    this->logger->debug("HornetIMU init complete!");
}

void HornetIMU::begin(float *gyroX, float *gyroY, float *gyroZ, float *accX, float *accY, float *accZ, float *magX, float *magY, float *magZ, float *angleX, float *angleY, float *angleZ)
{
    /**
     * @brief Begin communications with IMU.
     * 
     */
    this->logger->debug("HornetIMU begin!");

    this->accX = accX;
    this->accY = accY;
    this->accZ = accZ;

    this->gyroX = gyroX;
    this->gyroY = gyroY;
    this->gyroZ = gyroZ;

    this->magX = magX;
    this->magY = magY;
    this->magZ = magZ;

    this->angleX = angleX;
    this->angleY = angleY;
    this->angleZ = angleZ;

#ifdef IMU_JY901
    this->imu->StartIIC();
#endif // IMU_JY901

    this->logger->debug("HornetIMU begin complete!");
}

void HornetIMU::update()
{
    /**
     * @brief Update IMU readings
     * 
     */
#ifdef IMU_JY901
    this->imu->GetAcc();
    *this->accX = (float)this->imu->stcAcc.a[X] / 32768 * 16;
    *this->accY = (float)this->imu->stcAcc.a[Y] / 32768 * 16;
    *this->accZ = (float)this->imu->stcAcc.a[Z] / 32768 * 16;

    this->imu->GetGyro();
    *this->gyroX = (float)this->imu->stcGyro.w[X] / 32768 * 2000;
    *this->gyroY = (float)this->imu->stcGyro.w[Y] / 32768 * 2000;
    *this->gyroZ = (float)this->imu->stcGyro.w[Z] / 32768 * 2000;

    this->imu->GetAngle();
    *this->angleX = (float)this->imu->stcAngle.Angle[X] / 32768 * 180;
    *this->angleY = (float)this->imu->stcAngle.Angle[Y] / 32768 * 180;
    *this->angleZ = (float)this->imu->stcAngle.Angle[Z] / 32768 * 180;

    this->imu->GetMag();
    *this->magX = this->imu->stcMag.h[X];
    *this->magY = this->imu->stcMag.h[Y];
    *this->magZ = this->imu->stcMag.h[Z];
#endif // IMU_JY901
}

void HornetIMU::print()
{
    /**
     * @brief Print IMU data to logger
     * 
     */
    String acc = "Accelerometer:";
    acc += *this->accX;
    acc += " ";
    acc += *this->accY;
    acc += " ";
    acc += *this->accZ;
    this->logger->info(acc.c_str());

    String gyr = "Gyroscope:";
    gyr += *this->gyroX;
    gyr += " ";
    gyr += *this->gyroY;
    gyr += " ";
    gyr += *this->gyroZ;
    this->logger->info(gyr.c_str());

    String ang = "Angle:";
    ang += *this->angleX;
    ang += " ";
    ang += *this->angleY;
    ang += " ";
    ang += *this->angleZ;
    this->logger->info(ang.c_str());

    String mag = "Magnetometer:";
    mag += *this->magX;
    mag += " ";
    mag += *this->magY;
    mag += " ";
    mag += *this->magZ;
    this->logger->info(mag.c_str());
}