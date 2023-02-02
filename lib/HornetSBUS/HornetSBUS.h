/**
 * @file      HornetSBUS.h
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
#ifndef HORNET_SBUS_H
#define HORNET_SBUS_H
#include "Arduino.h"
#include <HornetMODULE.h>
#include <HornetLOGGER.h>
#include "sbus.h"

class HornetSBUS : public HornetModule
{
    bfs::SbusRx *sbus_rx;
    bfs::SbusTx *sbus_tx;
    std::array<int16_t, bfs::SbusRx::NUM_CH()> sbus_data;

public:
    HornetSBUS(HornetLOGGER *logger);
    void begin(unsigned long *channel_1_pwm, unsigned long *channel_2_pwm, unsigned long *channel_3_pwm, unsigned long *channel_4_pwm, unsigned long *channel_5_pwm, unsigned long *channel_6_pwm, unsigned long *channel_7_pwm, unsigned long *channel_8_pwm);
    #ifdef SBUS_16
    void begin(unsigned long *channel_1_pwm, unsigned long *channel_2_pwm, unsigned long *channel_3_pwm, unsigned long *channel_4_pwm, unsigned long *channel_5_pwm, unsigned long *channel_6_pwm, unsigned long *channel_7_pwm, unsigned long *channel_8_pwm, unsigned long *channel_9_pwm, unsigned long *channel_10_pwm, unsigned long *channel_11_pwm, unsigned long *channel_12_pwm, unsigned long *channel_13_pwm, unsigned long *channel_14_pwm, unsigned long *channel_15_pwm, unsigned long *channel_16_pwm);
    #endif // SBUS_16
    void update();
    void print();

private:
    unsigned long *channel_1_pwm, *channel_2_pwm, *channel_3_pwm, *channel_4_pwm, *channel_5_pwm, *channel_6_pwm, *channel_7_pwm, *channel_8_pwm;

#ifdef SBUS_16
    unsigned long *channel_9_pwm, *channel_10_pwm, *channel_11_pwm, *channel_12_pwm, *channel_13_pwm, *channel_14_pwm, *channel_15_pwm, *channel_16_pwm;
#endif // SBUS_16
};

#endif // HORNET_SBUS_H
