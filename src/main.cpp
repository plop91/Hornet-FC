/** 
 * @file      main.cpp
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
#include <HornetFC.h>

HornetFC* fc;

void setup() {
    fc = new HornetFC();
}

void loop() {
    fc->step();
}
