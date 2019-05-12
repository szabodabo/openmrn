
#include "Stm32Gpio.hxx"
#include "utils/GpioInitializer.hxx"
#include "BlinkerGPIO.hxx"
#include "stm32f0xx_hal_dma.h"
#include "stm32f0xx_hal_spi.h"


GPIO_PIN(LED_BLUE_RAW, LedPin, B, 2);  // STATUS_LED

typedef GpioInitializer<LED_BLUE_RAW_Pin> GpioInit;

typedef LED_BLUE_RAW_Pin BLINKER_RAW_Pin;
typedef BLINKER_Pin LED_BLUE_Pin;

