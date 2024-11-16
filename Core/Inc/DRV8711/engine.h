#ifndef INC_DRV8711_ENGINE_H_
#define INC_DRV8711_ENGINE_H_
// THIS IS A PORT OF FOLLOWING ARDUINO LIBRARY BY POLULU
// from arduino to stm32
// from C++ to C
// https://github.com/pololu/high-power-stepper-driver-arduino/blob/master/HighPowerStepperDriver.h#L248
#include "CONFIG.h"
#include "stm32g4xx_hal.h"


/// Addresses of control and status registers.
typedef enum
{
    CTRL = 0x00,
    TORQUE = 0x01,
    OFF = 0x02,
    BLANK = 0x03,
    DECAY = 0x04,
    STALL = 0x05,
    DRIVE = 0x06,
    STATUS = 0x07,
} DRV8711_REGISTER_ADDRESS_t;
/// Possible arguments to setStepMode().
typedef enum
{
    MicroStep256 = 256,
    MicroStep128 = 128,
    MicroStep64 = 64,
    MicroStep32 = 32,
    MicroStep16 = 16,
    MicroStep8 = 8,
    MicroStep4 = 4,
    MicroStep2 = 2,
    MicroStep1 = 1,
} DRV8711_STEP_MODE_t;

/// Possible arguments to setDecayMode().
typedef enum
{
    Slow = 0b000,
    SlowIncMixedDec = 0b001,
    Fast = 0b010,
    Mixed = 0b011,
    SlowIncAutoMixedDec = 0b100,
    AutoMixed = 0b101,
} DRV8711_DECAY_MODE_t;

/// Bits that are set in the return value of readStatus() to indicate status
/// conditions.
///
/// See the DRV8711 datasheet for detailed descriptions of these status
/// conditions.
typedef enum
{
    OTS = 0,    /// Overtemperature shutdown
    AOCP = 1,   /// Channel A overcurrent shutdown
    BOCP = 2,   /// Channel B overcurrent shutdown
    APDF = 3,   /// Channel A predriver fault
    BPDF = 4,   /// Channel B predriver fault
    UVLO = 5,   /// Undervoltage lockout
    STD = 6,    /// Stall detected
    STDLAT = 7, /// Latched stall detect
} DRV8711_STATUS_t;

typedef struct
{
    uint16_t ctrl, torque, off, blank, decay, stall, drive;
    GPIO_TypeDef *gpio_base_address;
    uint16_t gpio_pin;
    SPI_HandleTypeDef* hspi1;
} DRV8711SPI_t;
/**                                         FUNCTION DECLARATIONS                                      **/
void DRV8711SPI_setChipSelectPin(DRV8711SPI_t *obj, GPIO_TypeDef *gpio_base_address, uint16_t gpio_pin);
uint16_t readReg(DRV8711SPI_t *obj, DRV8711_REGISTER_ADDRESS_t address);
void writeReg(DRV8711SPI_t *obj, uint8_t address, uint16_t value);
void selectChip(DRV8711SPI_t *obj);
void deselectChip(DRV8711SPI_t *obj);
void DRV8711_init(DRV8711SPI_t *obj);
void setChipSelectPin(DRV8711SPI_t *obj, GPIO_TypeDef *gpio_base_address, uint16_t gpio_pin);
void resetSettings(DRV8711SPI_t *obj);
unsigned int verifySettings(DRV8711SPI_t *obj);
void applySettings(DRV8711SPI_t *obj);
void enableDriver(DRV8711SPI_t *obj);
void disableDriver(DRV8711SPI_t *obj);
void setDirection(DRV8711SPI_t *obj, uint8_t right);
uint16_t getDirection(DRV8711SPI_t *obj);
void step(DRV8711SPI_t *obj);
void setStepMode(DRV8711SPI_t *obj, DRV8711_STEP_MODE_t step);
void setCurrentMilliamps36v4(DRV8711SPI_t *obj, uint16_t current);
void setDecayMode(DRV8711SPI_t *obj, DRV8711_DECAY_MODE_t mode);
uint8_t readStatus(DRV8711SPI_t *obj);
void clearStatus(DRV8711SPI_t *obj);
uint8_t readFaults(DRV8711SPI_t *obj);
void clearFaults(DRV8711SPI_t *obj);
void writeCTRL(DRV8711SPI_t *obj);
void writeTORQUE(DRV8711SPI_t *obj);
void writeOFF(DRV8711SPI_t *obj);
void writeBLANK(DRV8711SPI_t *obj);
void writeDECAY(DRV8711SPI_t *obj);
void writeSTALL(DRV8711SPI_t *obj);
void writeDRIVE(DRV8711SPI_t *obj);

/**                                         END     DECLARATIONS                                      **/
// #include <Arduino.h>
// #include <SPI.h>



/// This class provides low-level functions for reading and writing from the SPI
/// interface of a DRV8711 stepper motor controller IC.
///
/// Most users should use the HighPowerStepperDriver class, which provides a
/// higher-level interface, instead of this class.


/// Configures this object to use the specified pin as a chip select pin.
///
/// You must use a chip select pin; the DRV8711 requires it.
void DRV8711SPI_setChipSelectPin(DRV8711SPI_t *obj, GPIO_TypeDef *gpio_base_address, uint16_t gpio_pin)
{ // function redundant but leave for now
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    setChipSelectPin(obj, gpio_base_address, gpio_pin);
#endif // ENGINE_SPI_ENABLE
}

/// Reads the register at the given address and returns its raw value.
uint16_t readReg(DRV8711SPI_t *obj, DRV8711_REGISTER_ADDRESS_t address)
{
    // Read/write bit and register address are the first 4 bits of the first
    // byte; data is in the remaining 4 bits of the first byte combined with
    // the second byte (12 bits total).
    // TODO
    selectChip(obj);
    //uint16_t dataOut = transfer((0x8 | (address & 0b111)) << 12);
    HAL_SPI_Transmit(obj->hspi1, (uint8_t*)address, 1, 100);
    uint8_t result;
    HAL_SPI_Receive(obj->hspi1, &result, 2, 100);
    deselectChip(obj);
    return result;
};

/// Writes the specified value to a register.
void writeReg(DRV8711SPI_t *obj, uint8_t address, uint16_t value)
{
    // Read/write bit and register address are the first 4 bits of the first
    // byte; data is in the remaining 4 bits of the first byte combined with
    // the second byte (12 bits total).

    selectChip(obj);
    HAL_SPI_Transmit(obj->hspi1, (uint8_t*)address, 8, 100);
    //transfer(((address & 0b111) << 12) | (value & 0xFFF));

    // The CS line must go low after writing for the value to actually take
    // effect.
    deselectChip(obj);
}



void selectChip(DRV8711SPI_t *obj)
{
    //digitalWrite(csPin, HIGH);
    HAL_GPIO_WritePin(obj->gpio_base_address, obj->gpio_pin, (GPIO_PinState) 1U);
    //SPI.beginTransaction(settings);
}

void deselectChip(DRV8711SPI_t* obj)
{
    //SPI.endTransaction();
    //digitalWrite(csPin, LOW);
    HAL_GPIO_WritePin(obj->gpio_base_address, obj->gpio_pin, (GPIO_PinState) 0U);
}


///////////////////////////////////////////////////////////////////////////////////////

void DRV8711_init(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    obj->ctrl = 0xC10;
    obj->torque = 0x1FF;
    obj->off = 0x030;
    obj->blank = 0x080;
    obj->decay = 0x110;
    obj->stall = 0x040;
    obj->drive = 0xA59;
#endif // ENGINE_SPI_ENABLE
}

/// Configures this object to use the specified pin as a chip select pin.
/// You must use a chip select pin; the DRV8711 requires it.
void setChipSelectPin(DRV8711SPI_t *obj, GPIO_TypeDef *gpio_base_address, uint16_t gpio_pin)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    obj->gpio_base_address = gpio_base_address;
    obj->gpio_pin = gpio_pin;
#endif // ENGINE_SPI_ENABLE
}

/// Changes all of the driver's settings back to their default values.
///
/// It is good to call this near the beginning of your program to ensure that
/// there are no settings left over from an earlier time that might affect the
/// operation of the driver.
void resetSettings(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    obj->ctrl = 0xC10;
    obj->torque = 0x1FF;
    obj->off = 0x030;
    obj->blank = 0x080;
    obj->decay = 0x110;
    obj->stall = 0x040;
    obj->drive = 0xA59;
    applySettings(obj);
#endif // ENGINE_SPI_ENABLE
}

/// Reads back the SPI configuration registers from the device and verifies
/// that they are equal to the cached copies stored in this class.
///
/// This can be used to verify that the driver is powered on and has not lost
/// them due to a power failure.  The STATUS register is not verified because
/// it does not contain any driver settings.
///
/// @return 1 if the settings from the device match the cached copies, 0 if
/// they do not.
unsigned int verifySettings(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return 0U;
    // TODO
    // Bit 10 in TORQUE is write-only and will always read as 0.
    return readReg(obj, CTRL) == obj->ctrl &&
           readReg(obj, TORQUE) == (obj->torque & ~(1 << 10)) &&
           readReg(obj, OFF) == obj->off &&
           readReg(obj, BLANK) == obj->blank &&
           readReg(obj, DECAY) == obj->decay &&
           readReg(obj, STALL) == obj->stall &&
           readReg(obj, DRIVE) == obj->drive;
#endif // ENGINE_SPI_ENABLE
}

/// Re-writes the cached settings stored in this class to the device.
///
/// You should not normally need to call this function because settings are
/// written to the device whenever they are changed.  However, if
/// verifySettings() returns false (due to a power interruption, for
/// instance), then you could use applySettings() to get the device's settings
/// back into the desired state.
void applySettings(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    writeTORQUE(obj);
    writeOFF(obj);
    writeBLANK(obj);
    writeDECAY(obj);
    writeDRIVE(obj);
    writeSTALL(obj);
#endif // ENIGNE_SPI_ENABLE
    // CTRL is written last because it contains the ENBL bit, and we want to try
    // to have all the other settings correct first.  (For example, TORQUE
    // defaults to 0xFF (the maximum value), so it would be better to set a more
    // appropriate value if necessary before enabling the motor.)
    writeCTRL(obj);
}

/// Enables the driver (ENBL = 1).
void enableDriver(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    obj->ctrl |= (1 << 0);
    writeCTRL(obj);
#endif // ENGINE_SPI_ENABLE
}

/// Disables the driver (ENBL = 0).
void disableDriver(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    obj->ctrl &= ~(1 << 0);
    writeCTRL(obj);
#endif // ENGINE_SPI_ENABLE
}

/// Sets the motor direction (RDIR).
///
/// Allowed values are 0 or 1.
///
/// You can use this command to control the direction of the stepper motor and
/// leave the DIR pin disconnected.
void setDirection(DRV8711SPI_t *obj, uint8_t right)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    obj->ctrl = right; // if not right (0U), engine moves left
    writeCTRL(obj);
#endif // ENGINE_SPI_ENABLE
}

/// Returns the cached value of the motor direction (RDIR).
///
/// This does not perform any SPI communication with the driver.
uint16_t getDirection(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return 0xDEAD;
    return obj->ctrl;
#endif // ENGINE_SPI_ENABLE
}

/// Advances the indexer by one step (RSTEP = 1).
///
/// You can use this command to step the stepper motor and leave the STEP pin
/// disconnected.
///
/// The driver automatically clears the RSTEP bit after it is written.
void step(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    writeReg(obj, CTRL, obj->ctrl | (1 << 2));
#endif
}

/// Sets the driver's stepping mode (MODE).
///
/// This affects many things about the performance of the motor, including how
/// much the output moves for each step taken and how much current flows
/// through the coils in each stepping position.
///
/// If an invalid stepping mode is passed to this function, then it selects
/// 1/4 micro-step, which is the driver's default.
///
/// Example usage:
/// ~~~{.cpp}
/// sd.setStepMode(HPSDStepMode::MicroStep32);
/// ~~~
void setStepMode(DRV8711SPI_t *obj, DRV8711_STEP_MODE_t step)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    uint8_t stepMode = (uint8_t)step;
    // if stepMode value is correct, only one bit from 8 should be set.
    // by negating stepMode we should get value with only one bit unset,
    // smallest possible value obtained by this method is 177U (0x7F),
    // THEREFORE if stepMode is greater than 176U and not equal to 256U, value is incorrect.
    if ((~stepMode > 177U) && stepMode != 0xFF)
    { // stepMode vlue overflow, set to 256U
        stepMode = 255U;
    }
    obj->ctrl = (obj->ctrl & 0b111110000111) | (stepMode << 3);
    writeCTRL(obj);
#endif // ENGINE_SPI_ENABLE
}

/// Sets the current limit for a High-Power Stepper Motor Driver 36v4.
///
/// The argument to this function should be the desired current limit in
/// milliamps.
///
/// WARNING: The 36v4 can supply up to about 4 A per coil continuously;
/// higher currents might be sustainable for short periods, but can eventually
/// cause the MOSFETs to overheat, which could damage them.  See the driver's
/// product page for more information.
///
/// This function allows you to set a current limit of up to 8 A (8000 mA),
/// but we strongly recommend against using a current limit higher than 4 A
/// (4000 mA) unless you are careful to monitor the MOSFETs' temperatures
/// and/or restrict how long the driver uses the higher current limit.
///
/// This function takes care of setting appropriate values for ISGAIN and
/// TORQUE to get the desired current limit.
void setCurrentMilliamps36v4(DRV8711SPI_t *obj, uint16_t current)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    if (current > 8000)
    {
        current = 8000;
    }
    // From the DRV8711 datasheet, section 7.3.4, equation 2:
    //
    //   Ifs = (2.75 V * TORQUE) / (256 * ISGAIN * Risense)
    //
    // Rearranged:
    //
    //   TORQUE = (256 * ISGAIN * Risense * Ifs) / 2.75 V
    //
    // The 36v4 has an Risense of 30 milliohms, and "current" is in milliamps,
    // so:
    //
    //   TORQUE = (256 * ISGAIN * (30/1000) ohms * (current/1000) A) / 2.75 V
    //          = (7680 * ISGAIN * current) / 2750000
    //
    // We want to pick the highest gain (5, 10, 20, or 40) that will not
    // overflow TORQUE (8 bits, 0xFF max), so we start with a gain of 40 and
    // calculate the TORQUE value needed.
    uint8_t isgainBits = 0b11;
    uint16_t torqueBits = ((uint32_t)768 * current) / 6875;

    // Halve the gain and TORQUE until the TORQUE value fits in 8 bits.
    while (torqueBits > 0xFF)
    {
        isgainBits--;
        torqueBits >>= 1;
    }

    obj->ctrl = (obj->ctrl & 0b110011111111) | (isgainBits << 8);
    writeCTRL(obj);
    obj->torque = (obj->torque & 0b111100000000) | torqueBits;
    writeTORQUE(obj);
#endif // ENGINE_SPI_ENABLE
}

/// Sets the driver's decay mode (DECMOD).
///
/// Example usage:
/// ~~~{.cpp}
/// sd.setDecayMode(HPSDDecayMode::AutoMixed);
/// ~~~
void setDecayMode(DRV8711SPI_t *obj, DRV8711_DECAY_MODE_t mode)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    obj->decay = (obj->decay & 0b00011111111) | (((uint8_t)mode & 0b111) << 8);
    writeDECAY(obj);
#endif // ENGINE_SPI_ENABLE
}

/// Reads the status of the driver (STATUS register).
///
/// The return value is an 8-bit unsigned integer that has one bit for each
/// status condition (the upper 4 bits of the 12-bit STATUS register are not
/// used).  You can simply compare the return value to 0 to see if any of the
/// status bits are set, or you can use the logical AND operator (`&`) and the
/// #HPSDStatusBit enum to check individual bits.
///
/// Example usage:
/// ~~~{.cpp}
/// if (sd.readStatus() & (1 << (uint8_t)HPSDStatusBit::UVLO))
/// {
///   // Undervoltage lockout is active.
/// }
/// ~~~
uint8_t readStatus(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return 0xDE;
    return readReg(obj, STATUS);
#endif // ENGINE_SPI_ENABLE
}

/// Clears any status conditions that are currently latched in the driver.
///
/// WARNING: Calling this function clears latched faults, which might allow
/// the motor driver outputs to reactivate.  If you do this repeatedly without
/// fixing an abnormal condition (like a short circuit), you might damage the
/// driver.
void clearStatus(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    writeReg(obj, STATUS, 0);
#endif // ENGINE_SPI_ENABLE
}

/// Reads fault conditions indicated by the driver.
///
/// The return value is the same as that which would be returned by
/// readStatus(), except it only contains bits that indicate faults (STATUS
/// bits 5:0).
uint8_t readFaults(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return 0xF;
    return readStatus(obj) & 0b00111111;
#endif // ENGINE_SPI_ENABLE
}

/// Clears any fault conditions that are currently latched in the driver.
///
/// This function behaves the same as clearStatus(), except it only clears
/// bits that indicate faults (STATUS bits 5:0).
///
/// WARNING: Calling this function clears latched faults, which might allow
/// the motor driver outputs to reactivate.  If you do this repeatedly without
/// fixing an abnormal condition (like a short circuit), you might damage the
/// driver.
void clearFaults(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    writeReg(obj, STATUS, ~0b00111111);
#endif // ENGINE_SPI_ENABLE
}

void writeCTRL(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    writeReg(obj, CTRL, obj->ctrl);
#endif // ENGINE_SPI_ENABLE
}

/// Writes the cached value of the TORQUE register to the device.
void writeTORQUE(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    writeReg(obj, TORQUE, obj->torque);
#endif // ENGINE_SPI_ENABLE
}

/// Writes the cached value of the OFF register to the device.
void writeOFF(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    writeReg(obj, OFF, obj->off);
#endif // ENGINE_SPI_ENABLE
}

/// Writes the cached value of the BLANK register to the device.
void writeBLANK(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    writeReg(obj, BLANK, obj->blank);
#endif // ENGINE_SPI_ENABLE
}

/// Writes the cached value of the DECAY register to the device.
void writeDECAY(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    writeReg(obj, DECAY, obj->decay);
#endif // ENGINE_SPI_ENABLE
}

/// Writes the cached value of the STALL register to the device.
void writeSTALL(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    writeReg(obj, STALL, obj->stall);
#endif // ENGINE_SPI_ENABLE
}

/// Writes the cached value of the DRIVE register to the device.
void writeDRIVE(DRV8711SPI_t *obj)
{
#if defined(ENGINE_SPI_ENABLE)
    if (NULL == obj)
        return;
    writeReg(obj, DRIVE, obj->drive);
#endif // ENGINE_SPI_ENABLE
}

#endif // INC_DRV8711_ENGINE_H_
