
#include "Stm32Gpio.hxx"
#include "utils/GpioInitializer.hxx"
#include "BlinkerGPIO.hxx"

GPIO_PIN(LED_GREEN_RAW, LedPin, A, 5);

GPIO_PIN(SW_USER, GpioInputPU, C, 13);

// Crossbuck lamps on A8-9.
GPIO_PIN(CROSSBUCK_LAMP_A, GpioOutputSafeLow, A, 8);
GPIO_PIN(CROSSBUCK_LAMP_B, GpioOutputSafeLow, A, 9);

typedef GpioInitializer<LED_GREEN_RAW_Pin, SW_USER_Pin,
                        CROSSBUCK_LAMP_A_Pin, CROSSBUCK_LAMP_A_Pin> GpioInit;

typedef LED_GREEN_RAW_Pin BLINKER_RAW_Pin;
typedef BLINKER_Pin LED_GREEN_Pin;
