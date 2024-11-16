/**
 * Error handling for DRV8711 driver
 */

#ifndef INC_DRV8711_ERROR_H_
#define INC_DRV8711_ERROR_H_
typedef enum{
	NO_ERROR = 0xFFFF,
	TIMEOUT = 0xDEAF,
	UNKNOWN = 0xDEAD,
/*_______________HARDWARE FAULTS_______________*/
  OTS = 0,  	// Overtemperature shutdown
  AOCP = 1, 	// Overcurrent shutdown
  BOCP = 2, 	// Channel B overcurrent shutdown
  APDF = 3,		// Channel A predriver fault
  BPDF = 4,		// Channel B predriver fault
  UVLO = 5,		// Undervoltage lockout
  STD = 6,		// Stall detected
  STDLAT = 7,	// Latched stall detect
}DRV8711_ERROR_t;

DRV8711_ERROR_t DRV8711_error_handler();
#endif /* INC_DRV8711_ERROR_H_ */
