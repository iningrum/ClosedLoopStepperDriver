#ifndef DRV8711_LED_H
#define DRV8711_LED_H
#include "CONFIG.h"
#include "stm32g4xx_hal.h"
typedef struct{
	GPIO_PinState LED_OK;
	GPIO_PinState LED_FAULT;
	GPIO_PinState LED_WS;
}DRV8711_LED_t;
void applyLed(DRV8711_LED_t* obj){
#if defined(STATUS_LED_ENABLE)
    if(NULL == obj) return;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, (GPIO_PinState) obj->LED_FAULT);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, (GPIO_PinState) obj->LED_OK);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (GPIO_PinState) obj->LED_WS);
#endif
}
#endif // DRV8711_LED_H
