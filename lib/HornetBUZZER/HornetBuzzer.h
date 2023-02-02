/** 
 * @file      HornetBuzzer.h
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
#ifndef FLIGHT_CONTROLLER_HORNETBUZZER_H
#define FLIGHT_CONTROLLER_HORNETBUZZER_H
#include <Arduino.h>
#include <HornetMODULE.h>
#include <HornetLOGGER.h>
#include "notes.h"

#define asabranca            0
#define babyelephantwalk     0
#define badinerie            0
#define bloodytears          0
#define brahmslullaby        0
#define cannonind            0
#define cantinaband          0
#define doom                 0
#define furelise             0
#define gameofthrones        0
#define greenhill            0
#define greensleeves         0
#define happybirthday        0
#define harrypotter          1
#define imperialmarch        0
#define jigglypuffsong       0
#define keyboardcat          0
#define merrychristmas       0
#define miichannel           0
#define minuetg              0
#define nevergonnagiveyouup  0
#define nokia                0
#define odetojoy             0
#define pacman               0
#define pinkpanther          0
#define princeigor           0
#define professorlayton      0
#define pulodagaita          0
#define silentnight          0
#define songofstorms         0
#define startrekintro        0
#define starwars             0
#define supermariobros       0
#define takeonme             0
#define tetris               0
#define thegodfather         0
#define thelick              0
#define thelionsleepstonight 0
#define vampirekiller        0
#define zeldaslullaby        0
#define zeldatheme           0


class HornetBuzzer : public HornetModule {
public:
    HornetBuzzer(HornetLOGGER *logger);
    void buzzer_play_hp();
private:
    int pin;
};




#endif //FLIGHT_CONTROLLER_HORNETBUZZER_H
