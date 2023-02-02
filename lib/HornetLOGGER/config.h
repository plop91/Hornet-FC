/**
 * @file      config.h
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
#ifndef HORNET_LOGGER_CONFIG
#define HORNET_LOGGER_CONFIG

#define LOG_FREQ 50

#define DEBUG

#if (defined TEENSY40 || defined TEENSY41)
// #define SD_ENBL     // Enable SD-card
// #define EEPROM_ENBL // Enable EEPROM
// #define RTC_ENBL    // Enable real time clock
#endif

#endif // HORNET_LOGGER_CONFIG