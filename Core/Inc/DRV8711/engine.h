#ifndef DRV8711_ENGINE_H
#define DRV8711_ENGINE_H
typedef struct{
    unsigned int dir: 1;
    unsigned int step: 4;
    unsigned int chip_enable: 1;


}DRV8711_ENGINE_t;
typedef enum
{
  CTRL   = 0x00,
  TORQUE = 0x01,
  OFF    = 0x02,
  BLANK  = 0x03,
  DECAY  = 0x04,
  STALL  = 0x05,
  DRIVE  = 0x06,
  STATUS = 0x07,
}DRV8711_REGISTERS_t;
void applyEngine(DRV8711_ENGINE_t* obj);

uint16_t readRegister(uint8_t address, DRV8711_ENGINE_t* obj, SPI_HandleTypeDef* spiHandle)
{
    if(NULL==obj) return;
// Read/write bit and register address are the first 4 bits of the first
// byte; data is in the remaining 4 bits of the first byte combined with
// the second byte (12 bits total).

obj->chip_enable = 1U;
applyEngine(obj);
// ((0x8 | (address & 0b111)) << 12);
uint8_t buffer[] = {
		address,
		0x00
};
HAL_SPI_Transmit_IT(&spiHandle, ((0x8 | (address & 0b111)) << 12), 12);
obj->chip_enable = 0U;
applyEngine(obj);
return dataOut & 0xFFF;
}

void applyEngine(DRV8711_ENGINE_t* obj){
#if defined(ENGINE_CONTROL_ENABLE)
    if (NULL == obj) return;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, (GPIO_PinState) obj->dir);
    if(obj->step > 8U){ 
        obj->step = 8U; // max val
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (GPIO_PinState) (1U << obj->step));
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (GPIO_PinState) obj->chip_enable );
#elif defined(ENGINE_SPI_ENABLE)
    if (NULL == obj) return;
#endif // ENGINE_CONTROL_ENABLE
}
#endif // DRV8711_ENGINE_H
