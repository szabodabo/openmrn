#include "BlinkerGPIO.hxx"
#include "Stm32Gpio.hxx"
#include "utils/GpioInitializer.hxx"
#include "stm32f3xx_hal_conf.h"

GPIO_PIN(LED_GREEN_RAW, LedPin, A, 5);

GPIO_PIN(SW_USER, GpioInputPU, C, 13);

// Crossbuck lamps on A8-9.
GPIO_PIN(CROSSBUCK_LAMP_A, GpioOutputSafeHigh, A, 8);
GPIO_PIN(CROSSBUCK_LAMP_B, GpioOutputSafeHigh, A, 9);

typedef GpioInitializer<LED_GREEN_RAW_Pin, SW_USER_Pin>
    GpioInit;

typedef LED_GREEN_RAW_Pin BLINKER_RAW_Pin;
typedef BLINKER_Pin LED_GREEN_Pin;

extern TIM_HandleTypeDef tim1_handle;
