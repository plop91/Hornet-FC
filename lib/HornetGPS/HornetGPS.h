/**
 * @file      HornetGPS.h
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
#ifndef HORNET_GPS_H
#define HORNET_GPS_H
#include <Arduino.h>
#include <HornetMODULE.h>
#include <HornetLOGGER.h>
#include <TinyGPSPlus.h>

class HornetGPS : public HornetModule
{
public:
    // Functions
    HornetGPS(HornetLOGGER *logger); //!
    void begin();                                      //!
    void update();                                     //!
    void print();                                      //!

private:
    // Variables
    TinyGPSPlus *gps;                     //!
    static const uint32_t GPSBaud = 9600; //!
    bool system_time_set = false;         //!
    float latitude, longitude;            //!
    float speed;                          //!
    time_t time;                          //!

    // Functions
    void read(unsigned long t); //!
};

#endif // HORNET_GPS_H
