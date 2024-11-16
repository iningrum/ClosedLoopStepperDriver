/*
 * engine.h
 *
 *  Created on: Nov 16, 2024
 *      Author: naidrok
 */

#ifndef INC_DRV8711_ENGINE_H_
#define INC_DRV8711_ENGINE_H_
#include "error.h"
#include "stdint.h"
inline uint16_t getStepSize(uint16_t i){
    uint16_t result = (1U << i); 
    return (result > 256U)? 256U: result;
}
typedef struct{
    float angle;
    float velocity;
#pragma region FLAGS /**/
    unsigned int direction: 1;
    unsigned int engine_enabled: 1;
#pragma endregion FLAGS /**/
    DRV8711_ERROR_t error;


}DRV8711_ENGINE_t;


#endif /* INC_DRV8711_ENGINE_H_ */
