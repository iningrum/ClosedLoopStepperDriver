# ClosedLoopStepperDriver
- Driver documentation: [DRV8711](https://www.ti.com/lit/ds/symlink/drv8711.pdf?ts=1715386038928)
- MCU documentation: [stm32g431c6](https://www.st.com/resource/en/datasheet/stm32g431c6.pdf)

# Solution
> [!WARNING]  
> solution has not been fully tested wonky spi communication.

> [!NOTE] 
> ### Branching strategy
> - `feature/...` - new features like led support, debug logs or spi
> - `fix/...` - bug fixes
> - `master` - treated as develop, forgot to protect it from [direct pushes](https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/managing-protected-branches/managing-a-branch-protection-rule).
> I've generally avoided pushing directly to master with few exceptions.
> ### Contributors
> Both are my accounts, I was logged onto a different one on PC than laptop and didn't notice.
## CONFIG.h
Similar idea to `FreeRTOSConfig.h` - configure program behavior at compile time using preprocessor macros.

Originally I wanted to introduce logs via [SVV/SWO](https://stackoverflow.com/a/65661978) and status displayed on leds so this approach made sense at the moment.
## Polulu library port
I've ported polulu DRV8711 header library which was originally written in C++ and targeted for Arduino framework. The idea was to get correct code which only requires slight fixes in data transmission.

Changes relative to original:
- Removed two layers of abstraction in favour of a simple struct and auxillary functions.
- adjusted documentation so it's correct in C language context.
- forward-declared every function to remove "implicit declaration" warnings, with future intention of splitting declaration and implementation between *.h and *.c files.

## main.c
- step controlled by pwm (`TIM2`)
- leveraged previously ported library to control driver
- status leds should roughly... do something?
- theoretically logs should be printed at the console provided correct clock speed but i did not have the time to test it fully.