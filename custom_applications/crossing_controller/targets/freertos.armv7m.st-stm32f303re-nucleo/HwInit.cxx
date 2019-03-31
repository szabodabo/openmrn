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
 *
 * This file represents the hardware initialization for the STM32F303RE Nucelo
 * board (bare).
 *
 * @author Balazs Racz
 * @date April 18, 2018
 */

#include <new>
#include <cstdint>

#include "stm32f3xx_hal_conf.h"

#include "os/OS.hxx"
#include "Stm32Uart.hxx"
#include "Stm32Can.hxx"
#include "Stm32EEPROMEmulation.hxx"
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
static Stm32EEPROMEmulation eeprom0("/dev/eeprom", 1024);

const size_t EEPROMEmulation::SECTOR_SIZE = 2048;

TIM_HandleTypeDef tim1_handle;

extern "C" {

/** Blink LED */
uint32_t blinker_pattern = 0;
static uint32_t rest_pattern = 0;

void hw_set_to_safe(void)
{
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


/// TIM17 shares this interrupt with certain features of timer1
void tim1_trg_com_tim17_interrupt_handler(void)
{
    //
    // Clear the timer interrupt.
    //
    TIM17->SR = ~TIM_IT_UPDATE;

    // Set output LED.
    BLINKER_RAW_Pin::set(rest_pattern & 1);

    // Shift and maybe reset pattern.
    rest_pattern >>= 1;
    if (!rest_pattern)
    {
        rest_pattern = blinker_pattern;
    }
}

uint16_t LAMP_UPDATE_COUNTER = 0;
#define TIM1_UPDATES_PER_SEC 20000

#define FADE_ON_TICKS 0.3 * TIM1_UPDATES_PER_SEC
#define FADE_OFF_TICKS 0.6 * TIM1_UPDATES_PER_SEC
#define LAMP_CYCLE_TOTAL_LEN_TICKS 1.2 * TIM1_UPDATES_PER_SEC
#define LAMP_NUM_BUCKETS 200

void tim1_up_tim16_interrupt_handler(void) {
    // handles tim1 update only (tim16 unused)
    // called every timer period (20kHz).
    TIM1->SR &= ~TIM_IT_UPDATE;

    if (!CROSSING_ACTIVE) {
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        LAMP_UPDATE_COUNTER = 0;
        return;
    }

    if (++LAMP_UPDATE_COUNTER > LAMP_CYCLE_TOTAL_LEN_TICKS) {
        LAMP_UPDATE_COUNTER = 0;
    }

    // lamp update counter cycles betweeh LAMP1 and LAMP2.
    // first half: lamp1 is turning on, lamp2 begins turning off.
    // second half: lamp2 begins turning on, lamp1 begins turning off.
    const uint32_t progress_ticks = LAMP_CYCLE_TOTAL_LEN_TICKS - LAMP_UPDATE_COUNTER;
    if (progress_ticks < LAMP_CYCLE_TOTAL_LEN_TICKS/2) {
        // turning lamp1 on, turning lamp2 off
        TIM1->CCR1 = LAMP_NUM_BUCKETS;
        TIM1->CCR2 = 0;
    } else {
        // turning lamp1 off, turning lamp2 on
        TIM1->CCR1 = 0;
        TIM1->CCR2 = LAMP_NUM_BUCKETS;
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

/** CPU clock speed. */
const unsigned long cm3_cpu_clock_hz = 72000000;
uint32_t SystemCoreClock;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV                     = 1
  *            PLLMUL                         = 9
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void clock_setup(void)
{
    HAL_RCC_DeInit();
    
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable HSE Oscillator and activate PLL with HSE as source on bypass
     * mode. This allows using the MCO clock output from the ST_Link part of
     * the nucleo board and freeing up the other clock pin for GPIO. */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;

    HAL_RCC_OscConfig(&RCC_OscInitStruct); 
    	
    /* Select PLL as system clock source and configure the HCLK, PCLK1 and
     * PCLK2 clocks dividers
     */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                   RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    HASSERT(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) == HAL_OK);

    // This will fail if the clocks are somehow misconfigured.
    HASSERT(SystemCoreClock == cm3_cpu_clock_hz);
}

/** Initialize the processor hardware.
 */
void hw_preinit(void)
{
    /* Globally disables interrupts until the FreeRTOS scheduler is up. */
    asm("cpsid i\n");

    /* these FLASH settings enable opertion at 72 MHz */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_2);

    /* setup the system clock */
    clock_setup();

    /* enable peripheral clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_TIM17_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* setup pinmux */
    GPIO_InitTypeDef gpio_init;
    memset(&gpio_init, 0, sizeof(gpio_init));

    /* USART2 pinmux on PA2 and PA3 */
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF7_USART2;
    gpio_init.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &gpio_init);
    gpio_init.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    /* CAN pinmux on PB8 and PB9 */
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF9_CAN;
    gpio_init.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    gpio_init.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    // Snd enable
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOC, &gpio_init);

    /**
     * LED Crossbucks timers: PA8 (TIM1_CH1) / PA9 (TIM1_CH2)
     * TIM1_CH1 on DMA1_CH2; TIM1_CH2 on DMA1_CH3 (from reference manual).
     * PSC=17, ARR=199 
     */
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF6_TIM1;
    gpio_init.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &gpio_init);
    gpio_init.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    memset(&tim1_handle, 0, sizeof(tim1_handle));
    tim1_handle.Instance = TIM1;
    tim1_handle.Init.Period = LAMP_NUM_BUCKETS-1;  // 1 less than Number of divisions for animation
    tim1_handle.Init.Prescaler = 17;  // ARR-1
    tim1_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim1_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    tim1_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HASSERT(HAL_TIM_PWM_Init(&tim1_handle) == HAL_OK);

    TIM_OC_InitTypeDef tim1_OCInit;
    memset(&tim1_OCInit, 0, sizeof(tim1_OCInit));
    tim1_OCInit.OCMode = TIM_OCMODE_PWM1;  // on for X counts, then off.
    tim1_OCInit.Pulse = 0;
    tim1_OCInit.OCPolarity = TIM_OCPOLARITY_HIGH;
    tim1_OCInit.OCFastMode = TIM_OCFAST_DISABLE;
    HASSERT(HAL_TIM_PWM_ConfigChannel(&tim1_handle, &tim1_OCInit, TIM_CHANNEL_1) == HAL_OK);
    HASSERT(HAL_TIM_PWM_ConfigChannel(&tim1_handle, &tim1_OCInit, TIM_CHANNEL_2) == HAL_OK);

    NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0xF0);
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

    GpioInit::hw_init();

    /* Initializes the blinker timer. */
    TIM_HandleTypeDef TimHandle;
    memset(&TimHandle, 0, sizeof(TimHandle));
    TimHandle.Instance = TIM17;
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
    NVIC_SetPriority(TIM17_IRQn, 0);
    NVIC_EnableIRQ(TIM17_IRQn);
}

void usart2_interrupt_handler(void)
{
    Stm32Uart::interrupt_handler(1);
}

}
