/** 
 * @file      HornetModule.h
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
#ifndef HORNET_MODULE_H
#define HORNET_MODULE_H
#include <Arduino.h>
#include <HornetLOGGER.h>

class HornetModule
{
protected:
    HornetModule(HornetLOGGER* logger){
        this->logger = logger;
    }
    HornetLOGGER *logger;
};

#endif // HORNET_MODULE_H