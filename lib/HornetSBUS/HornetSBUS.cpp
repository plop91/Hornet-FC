/**********************************
 * @file      HornetSBUS.cpp
 * @brief
 * @details
 * @author    Ian Sodersjerna
 * @version   0.0.1
 * @date      2022
 * @pre
 * @bug
 * @warning
 * @copyright GNU Public License.
 *********************************/
#include "HornetSBUS.h"

HornetSBUS::HornetSBUS(HornetLOGGER *logger) : HornetModule(logger)
{
    this->logger->debug("HornetSBUS init!");
#if (defined TEENSY40 || defined TEENSY41)
    this->sbus_rx = new bfs::SbusRx(&Serial5);
    this->sbus_tx = new bfs::SbusTx(&Serial5);
#elif defined ESPCAM
    this->sbus_rx = new bfs::SbusRx(&Serial1);
    this->sbus_tx = new bfs::SbusTx(&Serial1);

#else
#error sbus ports not defined for this board
#endif

    this->logger->debug("HornetSBUS init complete!");
}

void HornetSBUS::begin(unsigned long *channel_1_pwm, unsigned long *channel_2_pwm, unsigned long *channel_3_pwm, unsigned long *channel_4_pwm, unsigned long *channel_5_pwm, unsigned long *channel_6_pwm, unsigned long *channel_7_pwm, unsigned long *channel_8_pwm)
{
    this->logger->debug("HornetSBUS begin!");
    this->channel_1_pwm = channel_1_pwm;
    this->channel_2_pwm = channel_2_pwm;
    this->channel_3_pwm = channel_3_pwm;
    this->channel_4_pwm = channel_4_pwm;
    this->channel_5_pwm = channel_5_pwm;
    this->channel_6_pwm = channel_6_pwm;
    this->channel_7_pwm = channel_7_pwm;
    this->channel_8_pwm = channel_8_pwm;
    this->sbus_rx->Begin();
    this->sbus_tx->Begin();
    this->logger->debug("HornetSBUS begin complete!");
}

#ifdef SBUS_16
void HornetSBUS::begin(unsigned long *channel_1_pwm, unsigned long *channel_2_pwm, unsigned long *channel_3_pwm, unsigned long *channel_4_pwm, unsigned long *channel_5_pwm, unsigned long *channel_6_pwm, unsigned long *channel_7_pwm, unsigned long *channel_8_pwm, unsigned long *channel_9_pwm, unsigned long *channel_10_pwm, unsigned long *channel_11_pwm, unsigned long *channel_12_pwm, unsigned long *channel_13_pwm, unsigned long *channel_14_pwm, unsigned long *channel_15_pwm, unsigned long *channel_16_pwm)
{
    this->logger->debug("HornetSBUS begin!");
    this->sbus_rx->Begin();
    this->channel_1_pwm = channel_1_pwm;
    this->channel_2_pwm = channel_2_pwm;
    this->channel_3_pwm = channel_3_pwm;
    this->channel_4_pwm = channel_4_pwm;
    this->channel_5_pwm = channel_5_pwm;
    this->channel_6_pwm = channel_6_pwm;
    this->channel_7_pwm = channel_7_pwm;
    this->channel_8_pwm = channel_8_pwm;
    this->channel_9_pwm = channel_9_pwm;
    this->channel_10_pwm = channel_10_pwm;
    this->channel_11_pwm = channel_11_pwm;
    this->channel_12_pwm = channel_12_pwm;
    this->channel_13_pwm = channel_13_pwm;
    this->channel_14_pwm = channel_14_pwm;
    this->channel_15_pwm = channel_15_pwm;
    this->channel_16_pwm = channel_16_pwm;
    this->sbus_rx->Begin();
    this->sbus_tx->Begin();
    this->logger->debug("HornetSBUS begin complete!");
}
#endif // SBUS_16

void HornetSBUS::update()
{
    if (this->sbus_rx->Read())
    {
        this->sbus_data = this->sbus_rx->ch();

        *this->channel_1_pwm = this->sbus_data[0];
        *this->channel_2_pwm = this->sbus_data[1];
        *this->channel_3_pwm = this->sbus_data[2];
        *this->channel_4_pwm = this->sbus_data[3];
        *this->channel_5_pwm = this->sbus_data[4];
        *this->channel_6_pwm = this->sbus_data[5];
        *this->channel_7_pwm = this->sbus_data[6];
        *this->channel_8_pwm = this->sbus_data[7];

#ifdef SBUS_16
        *this->channel_9_pwm = this->sbus_data[8];
        *this->channel_10_pwm = this->sbus_data[9];
        *this->channel_11_pwm = this->sbus_data[10];
        *this->channel_12_pwm = this->sbus_data[11];
        *this->channel_13_pwm = this->sbus_data[12];
        *this->channel_14_pwm = this->sbus_data[13];
        *this->channel_15_pwm = this->sbus_data[14];
        *this->channel_16_pwm = this->sbus_data[15];
#endif // SBUS_16
    }
}

void HornetSBUS::print()
{
    String text = "RC Receiver: ";
    for (int8_t i = 0; i < bfs::SbusRx::NUM_CH(); i++)
    {
        text += this->sbus_data[i];
        text += "  ";
    }
    text += this->sbus_rx->lost_frame();
    text += "  ";
    text += this->sbus_rx->failsafe();
    logger->info(text.c_str());
}
