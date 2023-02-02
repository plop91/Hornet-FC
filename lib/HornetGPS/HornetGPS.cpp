/**
 * @file      HornetGPS.cpp
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
#include "HornetGPS.h"

HornetGPS::HornetGPS(HornetLOGGER *logger) : HornetModule(logger)
{
    logger->debug("hornetGPS init!");
#ifdef RTC_ENBL
    if (this->logger->is_time_set() == timeSet)
    {
        this->system_time_set = true;
    }
#endif // RTC_ENBL
    this->gps = new TinyGPSPlus();
    logger->debug("hornetGPS init success!");
}

void HornetGPS::begin()
{
    /**
     * @brief Connect with GPS device
     *
     */
    logger->debug("hornetGPS begin!");
#if (defined TEENSY40 || defined TEENSY41)
    Serial4.begin(this->GPSBaud);
#elif (defined ESPCAM || defined ESP32)
    Serial1.begin(this->GPSBaud);
#endif
    this->read(1000);
    logger->debug("hornetGPS begin success!");
}

void HornetGPS::update()
{
    /**
     * @brief Update GPS module
     *
     */
    this->read(10);

#ifdef RTC_ENBL
    if (!this->system_time_set)
    {
        if (this->gps->date.isValid() && this->gps->time.isValid())
        {
        }
    }
#endif // RTC_ENBL
}

void HornetGPS::read(unsigned long t)
{
    unsigned long t1 = millis();
    do
    {

#if (defined TEENSY40 || defined TEENSY41)
        while (Serial4.available())
        {
            this->gps->encode(Serial4.read());
        }
#elif (defined ESPCAM || defined ESP32)
        while (Serial1.available())
        {
            this->gps->encode(Serial1.read());
        }
#endif
    } while (millis() - t1 < t);
}

void HornetGPS::print()
{
    if (this->gps->charsProcessed() > 10)
    {
        String satellites = "Satellites: ";
        if (this->gps->satellites.isValid())
        {
            satellites += this->gps->satellites.value();
        }
        else
        {
            satellites += "INVALID";
        }
        this->logger->info(satellites.c_str());

        String location = "Location: ";
        if (this->gps->location.isValid())
        {
            location += this->gps->location.lat();
            location += ",";
            location += this->gps->location.lng();
        }
        else
        {
            location += "INVALID";
        }
        this->logger->info(location.c_str());

        String date = "GPS Date: ";
        if (this->gps->date.isValid())
        {
            date += this->gps->date.month();
            date += "/";
            date += this->gps->date.day();
            date += "/";
            date += this->gps->date.year();
        }
        else
        {
            date += "INVALID";
        }
        this->logger->info(date.c_str());

        String time = "GPS Time: ";
        if (this->gps->time.isValid())
        {
            if (this->gps->time.hour() < 10)
                time += "0";
            time += this->gps->time.hour();
            time += ":";
            if (this->gps->time.minute() < 10)
                time += "0";
            time += this->gps->time.minute();
            time += ":";
            if (this->gps->time.second() < 10)
                time += "0";
            time += this->gps->time.second();
            time += ".";
            if (this->gps->time.centisecond() < 10)
                time += "0";
            time += this->gps->time.centisecond();
        }
        else
        {
            time += "INVALID";
        }
        this->logger->info(time.c_str());

        String age = "GPS Age: ";
        if (this->gps->time.isValid())
        {
            age += this->gps->location.age();
        }
        else
        {
            age += "INVALID";
        }
        this->logger->info(age.c_str());
    }
    else
    {
        logger->warn("GPS: disconnected or malfunctioning: no signal received");
    }
}
