
#include "Stm32Gpio.hxx"
#include "utils/GpioInitializer.hxx"
#include "BlinkerGPIO.hxx"
#include "stm32f0xx_hal_dma.h"
#include "stm32f0xx_hal_spi.h"


GPIO_PIN(LED_BLUE_RAW, LedPin, B, 2);  // STATUS_LED

GPIO_PIN(ADJ_BUTTON, GpioInputPU, C, 6);  // Adjustment button
// Adjustment knob (ADC)

GPIO_PIN(LedDriver_BLANK, GpioOutputSafeHigh, A, 8);
GPIO_PIN(LedDriver_XLAT, GpioOutputSafeLow, A, 9);

// Input Port
GPIO_PIN(Plug_Input1, GpioInputPU, B, 4);
GPIO_PIN(Plug_Input2, GpioInputPU, B, 5);
GPIO_PIN(Plug_Input3, GpioInputPU, B, 6);
GPIO_PIN(Plug_Input4, GpioInputPU, B, 7);
GPIO_PIN(Plug_Input5, GpioInputPU, B, 10);
GPIO_PIN(Plug_Input6, GpioInputPU, B, 11);
GPIO_PIN(Plug_Input7, GpioInputPU, B, 12);
GPIO_PIN(Plug_Input8, GpioInputPU, B, 13);

typedef GpioInitializer<LED_BLUE_RAW_Pin, ADJ_BUTTON_Pin,
		LedDriver_BLANK_Pin, LedDriver_XLAT_Pin,
		Plug_Input1_Pin, Plug_Input2_Pin, Plug_Input3_Pin, Plug_Input4_Pin,
		Plug_Input5_Pin, Plug_Input6_Pin, Plug_Input7_Pin, Plug_Input8_Pin>
	GpioInit;

typedef LED_BLUE_RAW_Pin BLINKER_RAW_Pin;
typedef BLINKER_Pin LED_BLUE_Pin;

extern std::function<void(volatile uint16_t* data)> CalculateLedDataFn;

