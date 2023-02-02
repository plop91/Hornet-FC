/**
 * @file      HornetLOGGER.h
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
#ifndef HORNET_LOGGER_H
#define HORNET_LOGGER_H
#include <Arduino.h>
#include "config.h"

#ifdef EEPROM_ENBL
#include <EEPROM.h>
#endif // EEPROM_ENBL

#ifdef SD_ENBL
#include <SD.h>
#endif // SD_ENBL

#ifdef RTC_ENBL
#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#endif // RTC_ENBL

struct SensorFrame
{
    float temperature; // deg
    float pressure;    //
    float altitude;    //
    float latitude;    // deg
    float longitude;   // deg
    float speed;       // kmph
    float acc[3];      //
    float gyro[3];     //
    float angle[3];    //
    int16_t sbus[16];  //
};

class HornetLOGGER
{
public:
    HornetLOGGER();

    void begin_step();
    void end_step(unsigned long loop_time);

    void debug();
    void debug(const char *s);
    void debug(const __FlashStringHelper *s);
    void debug(float s);

    void info();
    void info(const char *s);
    void info(const __FlashStringHelper *s);
    void info(float s);

    void warn();
    void warn(const char *s);
    void warn(const __FlashStringHelper *s);
    void warn(float s);

    void error();
    void error(const char *s);
    void error(const __FlashStringHelper *s);
    void error(float s);

    // void print();
    // void print(const __FlashStringHelper *s);
    // void print(const char *s);
    // void print(float s);
    // void print(float f, int i);

    // void println();
    // void println(const __FlashStringHelper *s);
    // void println(const char *s);
    // void println(float s);
    // void println(float f, int i);

    unsigned long get_begin_time() { return this->begin_time; };
    unsigned long get_step_counter() { return this->step_counter; };

#ifdef RTC_ENBL
    void digitalClockDisplay();
    void printDigits(int digits);

    timeStatus_t is_time_set();
    void set_time(time_t t);
#endif // RTC_ENBL

#ifdef SD_ENBL
    void init_sd_info();
#endif // SD_ENBL

#ifdef EEPROM_ENBL
    void eeprom_put();
    void eeprom_get();
#endif // EEPROM_ENBL

private:
    unsigned long begin_time = 0;
    unsigned long end_time = 0;
    unsigned long step_counter = 0;

#ifdef EEPROM_ENBL
    unsigned int eeAddress = 0;
#endif // EEPROM_ENBL

#ifdef SD_ENBL
    Sd2Card card;
    SdVolume volume;
    SdFile root;
    int chipSelect = 254;
#endif // SD_ENBL
};

#endif // HORNET_LOGGER_H
