/**
 * @file      HornetLOGGER.cpp
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
#include "HornetLOGGER.h"

HornetLOGGER::HornetLOGGER()
{
    Serial.begin(9600);

#ifdef DEBUG
    // while (!Serial)
    // {
    // }
#endif

#ifdef EEPROM_ENBL
    Serial.print("EEPROM FRAMES: ");
    Serial.println(EEPROM.length() / sizeof(SensorFrame));
#endif // EEPROM_ENBL

#ifdef RTC_ENBL
    setSyncProvider(RTC.get); // the function to get the time from the RTC
    if (timeStatus() != timeSet)
        Serial.println("Unable to sync with the RTC");
    else
        Serial.println("RTC has set the system time");
#endif // RTC_ENBL
}

void HornetLOGGER::begin_step()
{
#ifdef RTC_ENBL
    if (this->step_counter % LOG_FREQ == 0)
    {
        if (timeStatus() == timeSet)
        {
            this->digitalClockDisplay();
        }
        else
        {
            Serial.println("The time has not been set.");
        }
    }
#endif // RTC_ENBL
}

void HornetLOGGER::end_step(unsigned long loop_time)
{
    this->step_counter++;
#ifdef DEBUG
    String time = "loop time: ";
    time += loop_time;
    this->debug(time.c_str());

    String counter = "loop_counter: ";
    counter += this->step_counter;
    this->debug(counter.c_str());

    String temp = "==========================================================================================";
    Serial.println(temp.c_str());
#endif // DEBUG
}

void HornetLOGGER::debug(const char *s)
{
    Serial.print("<DEBUG:");
    Serial.print(s);
    Serial.println(">");
}

void HornetLOGGER::debug(const __FlashStringHelper *s)
{
    Serial.print("<DEBUG:");
    Serial.print(s);
    Serial.println(">");
}

void HornetLOGGER::debug(float s)
{
    Serial.print("<DEBUG:");
    Serial.print(s);
    Serial.println(">");
}

void HornetLOGGER::info(const char *s)
{
    Serial.print("<INFO:");
    Serial.print(s);
    Serial.println(">");
}

void HornetLOGGER::info(const __FlashStringHelper *s)
{
    Serial.print("<INFO:");
    Serial.print(s);
    Serial.println(">");
}

void HornetLOGGER::info(float s)
{
    Serial.print("<INFO:");
    Serial.print(s);
    Serial.println(">");
}

void HornetLOGGER::warn(const char *s)
{
    Serial.print("<WARN:");
    Serial.print(s);
    Serial.println(">");
}

void HornetLOGGER::warn(const __FlashStringHelper *s)
{
    Serial.print("<WARN:");
    Serial.print(s);
    Serial.println(">");
}

void HornetLOGGER::warn(float s)
{
    Serial.print("<WARN:");
    Serial.print(s);
    Serial.println(">");
}

void HornetLOGGER::error(const char *s)
{
    Serial.print("<ERROR:");
    Serial.print(s);
    Serial.println(">");
}

void HornetLOGGER::error(const __FlashStringHelper *s)
{
    Serial.print("<ERROR:");
    Serial.print(s);
    Serial.println(">");
}

void HornetLOGGER::error(float s)
{
    Serial.print("<ERROR:");
    Serial.print(s);
    Serial.println(">");
}

#ifdef RTC_ENBL

void HornetLOGGER::digitalClockDisplay()
{
    // digital clock display of the time
    Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.print(" ");
    Serial.print(day());
    Serial.print(" ");
    Serial.print(month());
    Serial.print(" ");
    Serial.print(year());
    Serial.println();
}

void HornetLOGGER::printDigits(int digits)
{
    // utility function for digital clock display: prints preceding colon and leading 0
    Serial.print(":");
    if (digits < 10)
        Serial.print('0');
    Serial.print(digits);
}

timeStatus_t HornetLOGGER::is_time_set()
{
    return timeStatus();
}

void HornetLOGGER::set_time(time_t t)
{
    RTC.set(t);
    setTime(t);
}

#endif // RTC_ENBL

#ifdef SD_ENBL

void HornetLOGGER::init_sd_info()
{
    Serial.print("\nInitializing SD card...");

    // we'll use the initialization code from the utility libraries
    // since we're just testing if the card is working!
    if (!this->card.init(SPI_HALF_SPEED, this->chipSelect))
    {
        Serial.println("SD initialization failed.");
        return;
    }
    else
    {
        Serial.println("Wiring is correct and a card is present.");
    }

    // print the type of card
    Serial.print("\nCard type: ");
    switch (this->card.type())
    {
    case SD_CARD_TYPE_SD1:
        Serial.println("SD1");
        break;
    case SD_CARD_TYPE_SD2:
        Serial.println("SD2");
        break;
    case SD_CARD_TYPE_SDHC:
        Serial.println("SDHC");
        break;
    default:
        Serial.println("Unknown");
    }
}

#endif // SD_ENBL

#ifdef EEPROM_ENBL

void HornetLOGGER::eeprom_put()
{
    // eeAddress = 0;
    // MyObject customVar = {
    //     3.14f,
    //     65,
    //     "Working!"};

    // EEPROM.put(eeAddress, customVar);
    // Serial.println("Written custom data type!");
}

void HornetLOGGER::eeprom_get()
{
    // eeAddress = 0;

    // MyObject customVar;
    // EEPROM.get(eeAddress, customVar);

    // Serial.println("Read custom object from EEPROM: ");
    // Serial.println(customVar.field1);
    // Serial.println(customVar.field2);
    // Serial.println(customVar.name);
}

#endif // EEPROM_ENBL