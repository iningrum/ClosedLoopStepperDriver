/**
 * Error handling for DRV8711 driver
 */

#ifndef INC_DRV8711_ERROR_H_
#define INC_DRV8711_ERROR_H_
typedef enum{
	NO_ERROR = 0x0000,
	ENGINE_STALL = 0xFFFF,
	TIMEOUT = 0xDEAF,
	UNKNOWN = 0xDEAD,
}DRV8711_ERROR_t;

DRV8711_ERROR_t DRV8711_error_handler();
#endif /* INC_DRV8711_ERROR_H_ */
