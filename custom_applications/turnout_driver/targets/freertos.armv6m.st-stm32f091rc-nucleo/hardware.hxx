
#include "Stm32Gpio.hxx"
#include "utils/GpioInitializer.hxx"
#include "BlinkerGPIO.hxx"

GPIO_PIN(LED_GREEN_RAW, LedPin, B, 2);  // STATUS_LED

GPIO_PIN(SERVO1, GpioOutputSafeHigh, A, 8);
GPIO_PIN(SERVO2, GpioOutputSafeHigh, A, 9);
GPIO_PIN(SERVO3, GpioOutputSafeHigh, A, 10);
GPIO_PIN(SERVO4, GpioOutputSafeHigh, A, 11);

GPIO_PIN(SERVO1_EN, GpioOutputSafeHigh, B, 12);
GPIO_PIN(SERVO2_EN, GpioOutputSafeHigh, B, 13);
GPIO_PIN(SERVO3_EN, GpioOutputSafeHigh, B, 14);
GPIO_PIN(SERVO4_EN, GpioOutputSafeHigh, B, 15);

typedef GpioInitializer<LED_GREEN_RAW_Pin, //
	SERVO1_Pin, SERVO2_Pin, SERVO3_Pin, SERVO4_Pin, //
	SERVO1_EN_Pin, SERVO2_EN_Pin, SERVO3_EN_Pin, SERVO4_EN_Pin> GpioInit;

typedef LED_GREEN_RAW_Pin BLINKER_RAW_Pin;
typedef BLINKER_Pin LED_GREEN_Pin;

extern PWM* const servo_channels[];
const uint32_t servoPwmCountPerMs = configCPU_CLOCK_HZ / 1000;
