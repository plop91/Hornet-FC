/**
 * @file      HornetBarometer.cpp
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
#include "HornetBarometer.h"

HornetBarometer::HornetBarometer(HornetLOGGER *logger):HornetModule(logger)
{
    logger->debug("HornetBuzzer init!");
    this->bmp = new Adafruit_BMP3XX();
    logger->debug("HornetBuzzer init complete!");
}

void HornetBarometer::begin()
{

    logger->debug("HornetBuzzer begin!");
    if (!this->bmp->begin_I2C())
    {
        while (true)
        {
            logger->warn("BARO initialization failed!");
        }
    }

    // Set up oversampling and filter initialization
    this->bmp->setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    this->bmp->setPressureOversampling(BMP3_OVERSAMPLING_4X);
    this->bmp->setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    this->bmp->setOutputDataRate(BMP3_ODR_50_HZ);

    logger->debug("HornetBuzzer begin complete!");
}

void HornetBarometer::update()
{
    this->temperature = this->bmp->readTemperature();
    this->pressure = this->bmp->readPressure();
    this->altitude = this->bmp->readAltitude(SEALEVELPRESSURE_HPA);
}

void HornetBarometer::print()
{
    const String temp = String("Temperature:" + String(this->temperature) + " *C");
    logger->info(temp.c_str());

    const String press = String("Pressure:" + String(this->pressure) + " hPa");
    logger->info(press.c_str());

    const String alt = String("Altitude:" + String(this->altitude) + " m");
    logger->info(alt.c_str());
}
