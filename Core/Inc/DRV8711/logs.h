#ifndef DRV8711_DEBUG_LOG_H
#define DRV8711_DEBUG_LOG_H
#include "CONFIG.h"
#include <stdio.h>
#include "stm32g4xx_hal.h"
int _write(unsigned int file, char *ptr, unsigned int len){
#if defined(DEBUG_LOG_ENABLE)
    unsigned int i = 0U;
    for (i = 0U; i < len; i++)
        ITM_SendChar((*ptr++));
    return len;
#endif // DEBUG_LOG_ENABLE
}
#endif // DRV8711_DEBUG_LOG_H