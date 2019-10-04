/** \copyright
 * Copyright (c) 2018, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file HwInit.cxx
 * This file represents the hardware initialization for the STM32F091RC Nucelo
 * board (bare).
 *
 * @author Balazs Racz
 * @date April 18, 2018
 */

#include <new>
#include <cstdint>

#include "stm32f0xx_hal_rcc.h"
#include "stm32f0xx_hal_flash.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_gpio_ex.h"
#include "stm32f0xx_hal_dma.h"
#include "stm32f0xx_hal_tim.h"
#include "stm32f0xx_hal_spi.h"

#include "os/OS.hxx"
#include "Stm32Uart.hxx"
#include "Stm32Can.hxx"
#include "Stm32EEPROMEmulation.hxx"
#include "Stm32PWM.hxx"
#include "hardware.hxx"

/** override stdin */
const char *STDIN_DEVICE = "/dev/ser0";

/** override stdout */
const char *STDOUT_DEVICE = "/dev/ser0";

/** override stderr */
const char *STDERR_DEVICE = "/dev/ser0";

/** UART 0 serial driver instance */
static Stm32Uart uart0("/dev/ser0", USART2, USART2_IRQn);

/** CAN 0 CAN driver instance */
static Stm32Can can0("/dev/can0");

/** EEPROM emulation driver. The file size might be made bigger. */
static Stm32EEPROMEmulation eeprom0("/dev/eeprom", 512);

const size_t EEPROMEmulation::SECTOR_SIZE = 2048;

/*
  Servo PWM timers
   
  PA8 TIM1_CH1 (AF2)
  PA9 TIM1_CH2 (AF2)
  PA10 TIM1_CH3 (AF2)
  PA11 TIM1_CH4 (AF2)
*/
Stm32PWMGroup servo_timer(TIM1,
    /*prescaler=*/ (servoPwmCountPerMs * 6 + 65535) / 65536,
    /*period_counts=*/ servoPwmCountPerMs * 6);

extern PWM* const servo_channels[];
PWM * const servo_channels[4] = { //
    Stm32PWMGroup::get_channel(&servo_timer, 1),
    Stm32PWMGroup::get_channel(&servo_timer, 2),
    Stm32PWMGroup::get_channel(&servo_timer, 3),
    Stm32PWMGroup::get_channel(&servo_timer, 4)};

extern uint8_t RELAY_DATA[];

#define RLY1_SET_GPIO_PORT GPIOB
#define RLY1_SET_GPIO_PIN GPIO_PIN_4
#define RLY1_RESET_GPIO_PORT GPIOB
#define RLY1_RESET_GPIO_PIN GPIO_PIN_5

#define RLY2_SET_GPIO_PORT GPIOB
#define RLY2_SET_GPIO_PIN GPIO_PIN_6
#define RLY2_RESET_GPIO_PORT GPIOB
#define RLY2_RESET_GPIO_PIN GPIO_PIN_7

#define RLY3_SET_GPIO_PORT GPIOB
#define RLY3_SET_GPIO_PIN GPIO_PIN_0
#define RLY3_RESET_GPIO_PORT GPIOB
#define RLY3_RESET_GPIO_PIN GPIO_PIN_1

#define RLY4_SET_GPIO_PORT GPIOB
#define RLY4_SET_GPIO_PIN GPIO_PIN_11
#define RLY4_RESET_GPIO_PORT GPIOB
#define RLY4_RESET_GPIO_PIN GPIO_PIN_10

extern "C" {

/** Blink LED */
uint32_t blinker_pattern = 0;
static uint32_t rest_pattern = 0;

void DisableAllRelays(void) {
	HAL_GPIO_WritePin(RLY1_SET_GPIO_PORT, RLY1_SET_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RLY1_RESET_GPIO_PORT, RLY1_RESET_GPIO_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(RLY2_SET_GPIO_PORT, RLY2_SET_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RLY2_RESET_GPIO_PORT, RLY2_RESET_GPIO_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(RLY3_SET_GPIO_PORT, RLY3_SET_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RLY3_RESET_GPIO_PORT, RLY3_RESET_GPIO_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(RLY4_SET_GPIO_PORT, RLY4_SET_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RLY4_RESET_GPIO_PORT, RLY4_RESET_GPIO_PIN, GPIO_PIN_RESET);
}

const uint16_t RELAY_PINS[8] = {
	RLY1_SET_GPIO_PIN,
	RLY1_RESET_GPIO_PIN,
	RLY2_SET_GPIO_PIN,
	RLY2_RESET_GPIO_PIN,
	RLY3_SET_GPIO_PIN,
	RLY3_RESET_GPIO_PIN,
	RLY4_SET_GPIO_PIN,
	RLY4_RESET_GPIO_PIN,
};

static GPIO_TypeDef* RELAY_PORTS[8] = {
	RLY1_SET_GPIO_PORT,
	RLY1_RESET_GPIO_PORT,
	RLY2_SET_GPIO_PORT,
	RLY2_RESET_GPIO_PORT,
	RLY3_SET_GPIO_PORT,
	RLY3_RESET_GPIO_PORT,
	RLY4_SET_GPIO_PORT,
	RLY4_RESET_GPIO_PORT,
};

void hw_set_to_safe(void)
{
	DisableAllRelays();
}

void resetblink(uint32_t pattern)
{
    blinker_pattern = pattern;
    rest_pattern = pattern ? 1 : 0;
    BLINKER_RAW_Pin::set(pattern ? true : false);
    /* make a timer event trigger immediately */
}

void setblink(uint32_t pattern)
{
    resetblink(pattern);
}


const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

void timer14_interrupt_handler(void)
{
    //
    // Clear the timer interrupt.
    //
    TIM14->SR = ~TIM_IT_UPDATE;

    // Set output LED.
    BLINKER_RAW_Pin::set(rest_pattern & 1);

    // Shift and maybe reset pattern.
    rest_pattern >>= 1;
    if (!rest_pattern)
    {
        rest_pattern = blinker_pattern;
    }
}

// Called every 3ms.
void timer7_interrupt_handler(void) {
	TIM7->SR = ~TIM_IT_UPDATE;

	DisableAllRelays();

	for (uint8_t r = 0; r < 4; r++) {
		const uint8_t relayBaseIdx = r*2;
		const uint8_t is_set = RELAY_DATA[relayBaseIdx];
		const uint8_t is_reset = RELAY_DATA[relayBaseIdx+1];
		if (is_set) {
			HAL_GPIO_WritePin(RELAY_PORTS[relayBaseIdx], RELAY_PINS[relayBaseIdx], GPIO_PIN_SET);
			RELAY_DATA[relayBaseIdx] = 0;
		} else if (is_reset) {
			HAL_GPIO_WritePin(RELAY_PORTS[relayBaseIdx+1], RELAY_PINS[relayBaseIdx+1], GPIO_PIN_SET);
			RELAY_DATA[relayBaseIdx+1] = 0;
		}
	}
}

void diewith(uint32_t pattern)
{
    // vPortClearInterruptMask(0x20);
    asm("cpsie i\n");

    resetblink(pattern);
    while (1)
        ;
}

/** Setup the system clock */
static void clock_setup(void)
{
    /* reset clock configuration to default state */
    RCC->CR = RCC_CR_HSITRIM_4 | RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY))
        ;

#define USE_EXTERNAL_8_MHz_CLOCK_SOURCE 1
/* configure PLL:  8 MHz * 6 = 48 MHz */
#if USE_EXTERNAL_8_MHz_CLOCK_SOURCE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
        ;
    RCC->CFGR = RCC_CFGR_PLLMUL6 | RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_SW_HSE;
    while (!((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSE))
        ;
#else
    RCC->CFGR = RCC_CFGR_PLLMUL6 | RCC_CFGR_PLLSRC_HSI_PREDIV | RCC_CFGR_SW_HSI;
    while (!((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSI))
        ;
#endif
    /* enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    /* set PLL as system clock */
    RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_PLL;
    while (!((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL))
        ;


}

/** Initialize the processor hardware.
 */
void hw_preinit(void)
{
    /* Globally disables interrupts until the FreeRTOS scheduler is up. */
    asm("cpsid i\n");

    /* these FLASH settings enable opertion at 48 MHz */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

    /* setup the system clock */
    clock_setup();

    /* enable peripheral clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_TIM14_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM7_CLK_ENABLE();

    /* setup pinmux */
    GPIO_InitTypeDef gpio_init;
    memset(&gpio_init, 0, sizeof(gpio_init));

    /* USART2 pinmux on PA2 and PA3 */
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF1_USART2;
    gpio_init.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &gpio_init);
    gpio_init.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    /* CAN pinmux on PB8 and PB9 */
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF4_CAN;
    gpio_init.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    gpio_init.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    GpioInit::hw_init();

    /* Initializes the blinker timer. */
    TIM_HandleTypeDef TimHandle;
    memset(&TimHandle, 0, sizeof(TimHandle));
    TimHandle.Instance = TIM14;
    TimHandle.Init.Period = configCPU_CLOCK_HZ / 10000 / 8;
    TimHandle.Init.Prescaler = 10000;
    TimHandle.Init.ClockDivision = 0;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TimHandle.Init.RepetitionCounter = 0;
    if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
    {
        /* Initialization Error */
        HASSERT(0);
    }
    if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
    {
        /* Starting Error */
        HASSERT(0);
    }
    NVIC_SetPriority(TIM14_IRQn, 0);
    NVIC_EnableIRQ(TIM14_IRQn);

    // Relay timer. Fire the update interrupt every 3ms.
    memset(&TimHandle, 0, sizeof(TimHandle));
    TimHandle.Instance = TIM7;
    TimHandle.Init.Period = 30 - 1;
    TimHandle.Init.Prescaler = configCPU_CLOCK_HZ / 10000;
    TimHandle.Init.ClockDivision = 0;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TimHandle.Init.RepetitionCounter = 0;
    HASSERT(HAL_TIM_Base_Init(&TimHandle) == HAL_OK);
    HASSERT(HAL_TIM_Base_Start_IT(&TimHandle) == HAL_OK);
    NVIC_SetPriority(TIM7_IRQn, 0);
    NVIC_EnableIRQ(TIM7_IRQn);

    /*
      Switch servo pins to timer mode.
       
      PA8 TIM1_CH1 (AF2)
      PA9 TIM1_CH2 (AF2)
      PA10 TIM1_CH3 (AF2)
      PA11 TIM1_CH4 (AF2)
    */

    memset(&gpio_init, 0, sizeof(gpio_init));
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF2_TIM1;
    gpio_init.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &gpio_init);
    gpio_init.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOA, &gpio_init);
    gpio_init.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &gpio_init);
    gpio_init.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    // Enable all servo power.
    SERVO1_EN_Pin::set(1);
    SERVO2_EN_Pin::set(1);
    SERVO3_EN_Pin::set(1);
    SERVO4_EN_Pin::set(1);

    /*
     * Relays: Coil operating time = ~2ms
     */
    memset(&gpio_init, 0, sizeof(gpio_init));
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull = GPIO_PULLDOWN;
	gpio_init.Speed = GPIO_SPEED_FREQ_LOW;

	gpio_init.Pin = RLY3_SET_GPIO_PIN;
	HAL_GPIO_Init(RLY3_SET_GPIO_PORT, &gpio_init);
	gpio_init.Pin = RLY3_RESET_GPIO_PIN;
	HAL_GPIO_Init(RLY3_RESET_GPIO_PORT, &gpio_init);
	gpio_init.Pin = RLY4_SET_GPIO_PIN;
	HAL_GPIO_Init(RLY4_SET_GPIO_PORT, &gpio_init);
	gpio_init.Pin = RLY4_RESET_GPIO_PIN;
	HAL_GPIO_Init(RLY4_RESET_GPIO_PORT, &gpio_init);

	HAL_GPIO_WritePin(RLY3_SET_GPIO_PORT, RLY3_SET_GPIO_PIN, GPIO_PIN_SET);

	// PORT_IN5 hack on PA15
	gpio_init.Pin = GPIO_PIN_15;
	HAL_GPIO_Init(GPIOA, &gpio_init);
}

}
