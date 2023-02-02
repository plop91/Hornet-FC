/**
 * @file      HornetBarometer.h
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
#ifndef HORNET_BAROMETER_H
#define HORNET_BAROMETER_H
#include <Arduino.h>
#include <HornetMODULE.h>
#include <HornetLOGGER.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define SEALEVELPRESSURE_HPA (1013.25)

class HornetBarometer : public HornetModule
{
public:
    HornetBarometer(HornetLOGGER *logger);
    void begin();
    void update();
    void print();

private:
    Adafruit_BMP3XX *bmp;
    float temperature;
    float pressure;
    float altitude;
};

#endif // HORNET_BAROMETER_H
