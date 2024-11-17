#ifndef DRV8711_CONFIG_H
#define DRV8711_CONFIG_H
/**
 * This file solely exists because i had no way to test my code.
 * Using ON/OFF enable implementations of features
 * Hopefully this minimizes amount of hard faults.
 */

#define STATUS_LED_ENABLE 1U
#define ENGINE_SPI_ENABLE 1U
/*********************************PARAMETERS*********************************/
#define SPI_BUFFER_SIZE 4U
#endif //DRV8711_CONFIG_H