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
#ifndef HORNET_FC_CONFIG
#define HORNET_FC_CONFIG

// Board selection (Only choose one)
#if !(defined TEENSY40 || defined TEENSY41 || defined ESPCAM)
// #define TEENSY40
// #define TEENSY41
// #define ESPCAM
#endif

// DEBUG settings
#define DEBUG           // Enable debug outputs
#define LOOP_DELAY 1000 // Time delay between loops

// Onboard
#define BLINK_ENBL // Enable blinking led
//#define BUZZER_ENBL    // Enable buzzer (needs to be updated to work properly)

// Receiver settings (this will effect how many channels are processed by the flight controller so use the smallest number possible)
#define SBUS_8
//#define SBUS_16
//#define WIFI-RECV

// Enable Optional Sub-Systems
// #define BARO_ENBL    // Enable barometer
// #define GPS_ENBL     // Enable GPS

// Autonomous
// #define AUTONAV_ENBL // Enable automatic navigation via gps

// Enable Outputs
#define ONBOARD_PWM_OUTPUTS // Enable PWM output from Hornet (should be left enabled for most use cases)
// #define ONBOARD_PWM_MOTORS  // Enable PWM based Motor Control
// #define PCA9685

// ESP32 Variables

#endif // HORNET_FC_CONFIG
