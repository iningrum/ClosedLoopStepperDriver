#ifndef DRV8711_ENGINE_H
#define DRV8711_ENGINE_H
typedef struct{
    unsigned int dir: 1;
    unsigned int step: 4;
    unsigned int chip_enable: 1;


}DRV8711_ENGINE_t;
void applyEngine(DRV8711_ENGINE_t* obj){
#if defined(ENGINE_CONTROL_ENABLE)
    if (NULL == obj) return;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, (GPIO_PinState) obj->dir);
    if(obj->step > 8U){ 
        obj->step = 8U; // max val
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (GPIO_PinState) (1U << obj->step));
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (GPIO_PinState) obj->chip_enable );
#endif // ENGINE_CONTROL_ENABLE
}
#endif // DRV8711_ENGINE_H