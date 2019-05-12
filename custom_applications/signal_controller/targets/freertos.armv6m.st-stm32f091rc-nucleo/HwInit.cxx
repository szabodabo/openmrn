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
#include "stm32f0xx_hal.h"

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

static SPI_HandleTypeDef hspi1;
volatile static uint16_t LedDriverData[24];
static DMA_HandleTypeDef hdma_led_tx;
static TIM_HandleTypeDef htim6;

extern "C" {

/** Blink LED */
uint32_t blinker_pattern = 0;
static uint32_t rest_pattern = 0;

void hw_set_to_safe(void)
{
	LedDriver_BLANK_Pin::set(1);
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
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_TIM6_CLK_ENABLE();

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

    /*
	  SPI1_SCK: PA5
	  SPI1_MISO: PA6
	  SPI1_MOSI: PA7
	*/
	memset(&gpio_init, 0, sizeof(gpio_init));
	gpio_init.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	gpio_init.Mode = GPIO_MODE_AF_PP;
	gpio_init.Pull = GPIO_NOPULL;
	gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio_init.Alternate = GPIO_AF0_SPI1;
	HAL_GPIO_Init(GPIOA, &gpio_init);

	memset(&hspi1, 0, sizeof(hspi1));
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	// Change if we want to read output data
	hspi1.Init.Direction = SPI_DIRECTION_1LINE;
	hspi1.Init.DataSize = SPI_DATASIZE_12BIT;  // HAL uses 16-bit input
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

	if (HAL_SPI_DeInit(&hspi1) != HAL_OK) {
		HASSERT(0);
	}
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		HASSERT(0);
	}

	hdma_led_tx.Instance = DMA1_Channel3;
	hdma_led_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_led_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_led_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_led_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_led_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_led_tx.Init.Mode = DMA_NORMAL;
	hdma_led_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
	if (HAL_DMA_Init(&hdma_led_tx) != HAL_OK) {
		HASSERT(0);
	}
	__HAL_LINKDMA(&hspi1, hdmatx, hdma_led_tx);
	HAL_NVIC_SetPriority(DMA1_Ch2_3_DMA2_Ch1_2_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(DMA1_Ch2_3_DMA2_Ch1_2_IRQn);

	htim6.Instance = TIM6;
	htim6.Init.Prescaler = (configCPU_CLOCK_HZ/2/150)-1;
	htim6.Init.Period = 1;
	HAL_NVIC_SetPriority(TIM6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM6_IRQn);
	HAL_TIM_Base_Init(&htim6);
	HAL_TIM_Base_Start_IT(&htim6);
}

void dma_ch2_3_dma2_ch1_2_interrupt_handler() {
	HAL_DMA_IRQHandler(hspi1.hdmatx);
}

volatile static uint32_t CURRENT_ANIM_TICK = 0;
#define NUM_TICKS_IN_CYCLE 2000

void AnimateLEDs() {
	if (HAL_SPI_GetState(&hspi1) > HAL_SPI_STATE_READY) {
		return;
	}

	if (++CURRENT_ANIM_TICK > NUM_TICKS_IN_CYCLE) {
		CURRENT_ANIM_TICK = 0;
	}
	const double progress =  (double) CURRENT_ANIM_TICK / (double) NUM_TICKS_IN_CYCLE;
	//const double progress = (CURRENT_ANIM_TICK*2 > NUM_TICKS_IN_CYCLE) ? 1.0 : 0.0;

	for (int i = 0; i < 24; i++) {
		// todo: should be config
		uint16_t bright = 0;
		if (i % 3 == 0) {
			bright = 300;
		} else if (i % 3 == 1) {
			bright = 1200;
		} else {
			bright = 500;
		}

		LedDriverData[i] = (double) bright * progress;
		//LedDriverData[i] = 1000;
	}
	if (HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&LedDriverData, 24) != HAL_OK) {
		HASSERT(0);
	}
}

void timer6_dac_interrupt_handler() {
	AnimateLEDs();
	__HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) {
	LedDriver_XLAT_Pin::set(1);
	//for (volatile uint8_t t=0; t < 30; t++);
	LedDriver_XLAT_Pin::set(0);
}

/*
 * LED Update Strategy:
 * - Continuous DMA writes buffered LED GS info to SPI.
 * - At 100Hz, recalculate animation data and write into DMA buffer.
 */

void StartLEDData() {


	LedDriver_BLANK_Pin::set(0);
	return;
}

}
