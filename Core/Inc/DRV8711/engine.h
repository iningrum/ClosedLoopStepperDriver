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
typedef enum
{
  Slow                = 0b000,
  SlowIncMixedDec     = 0b001,
  Fast                = 0b010,
  Mixed               = 0b011,
  SlowIncAutoMixedDec = 0b100,
  AutoMixed           = 0b101,
}DRV8711_DECAY_MODE_t;

typedef struct{
    float angle;
    float velocity;
    DRV8711_ERROR_t error;
    DRV8711_DECAY_MODE_t decay;
    uint32_t ctrl, torque, off, blank, decay, stall, drive;


}DRV8711_ENGINE_t;
void applySettings(DRV8711_ENGINE_t);
/*_____________DEFAULT_ENGINE_SETTINGS____________*/
void DRV8711_init(DRV8711_ENGINE_t* obj){
    if(nullptr== obj) return;
    obj->ctrl   = 0xC10;
    obj->torque = 0x1FF;
    obj->off    = 0x030;
    obj->blank  = 0x080;
    obj->decay  = 0x110;
    obj->stall  = 0x040;
    obj->drive  = 0xA59;
}
void DRV8711_reset(DRV8711_ENGINE_t* obj){
    if(nullptr == obj) return;
    obj->ctrl   = 0xC10;
    obj->torque = 0x1FF;
    obj->off    = 0x030;
    obj->blank  = 0x080;
    obj->decay  = 0x110;
    obj->stall  = 0x040;
    obj->drive  = 0xA59;

}
#endif /* INC_DRV8711_ENGINE_H_ */
