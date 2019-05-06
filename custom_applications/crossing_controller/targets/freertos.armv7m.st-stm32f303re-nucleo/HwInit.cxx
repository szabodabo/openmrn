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
ADC_HandleTypeDef adc1_handle, adc2_handle, adc3_handle;
DMA_HandleTypeDef hdma_adc1, hdma_adc2, hdma_adc3;

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

#define TIM3_UPDATES_PER_SEC 50
#define TIM3_SERVO_NUM_BUCKETS 999
#define TIM3_SERVO_BUCKETS_PER_MS (TIM3_SERVO_NUM_BUCKETS / 20)  // 50Hz freq = 20ms period
#define TIM3_SERVO_MIN_DUTY (TIM3_SERVO_BUCKETS_PER_MS * 1)
#define TIM3_SERVO_MAX_DUTY (TIM3_SERVO_BUCKETS_PER_MS * 2)
#define SERVO_ANIMATION_LEN_TICKS (TIM3_UPDATES_PER_SEC * 12)

uint32_t tim3_update_count = 0;

void setduty(int32_t duty) {
    TIM3->CCR1 = duty;
    TIM3->CCR2 = duty;
    TIM3->CCR3 = duty;
    TIM3->CCR4 = duty;
}

int32_t animation_progress(int32_t from_value, int32_t to_value,
                           int32_t progress_ticks, int32_t animation_len_ticks) {
    progress_ticks = std::min(progress_ticks, animation_len_ticks);
    const int32_t from_weight = (animation_len_ticks - progress_ticks);
    const int32_t to_weight = progress_ticks;
    return ((from_value * from_weight) + (to_value * to_weight))
            / (from_weight + to_weight);
}

// TODO: is there a way to not depend on from_value?
//       could we read the current value and chart a course?
class AnimationCounter {
public:
    AnimationCounter(int32_t from_value, int32_t to_value,
                     int32_t animation_len_ticks) :
        from_value_(from_value), to_value_(to_value),
        animation_len_ticks_(animation_len_ticks),
        progress_ticks_(0) {
            HASSERT(animation_len_ticks > 0);
        }

    void Tick() {
        if (progress_ticks_ < animation_len_ticks_) {
            progress_ticks_++;
        }
    }

    int32_t Value() {
        return animation_progress(from_value_, to_value_,
            progress_ticks_, animation_len_ticks_);
    }

    bool Done() {
        return progress_ticks_ >= animation_len_ticks_;
    }

private:
    int32_t from_value_;
    int32_t to_value_;
    int32_t animation_len_ticks_;
    int32_t progress_ticks_;
};

bool animation_on_scheduled = false;
bool animation_off_scheduled = false;
AnimationCounter tim3_servo_animation(TIM3_SERVO_MIN_DUTY, TIM3_SERVO_MIN_DUTY, 1);

volatile uint8_t adc_count = 0;
uint16_t adcDet1278[4];
uint16_t adcDet356[3];
uint16_t adcDet4;

void tim3_interrupt_handler(void) {
    TIM3->SR &= ~TIM_IT_UPDATE;

    if (CROSSING_ACTIVE && !animation_on_scheduled) {
        tim3_servo_animation = AnimationCounter(TIM3->CCR1, TIM3_SERVO_MAX_DUTY, SERVO_ANIMATION_LEN_TICKS);
        animation_on_scheduled = true;
        animation_off_scheduled = false;
    } else if (!CROSSING_ACTIVE && !animation_off_scheduled) {
        tim3_servo_animation = AnimationCounter(TIM3->CCR1, TIM3_SERVO_MIN_DUTY, SERVO_ANIMATION_LEN_TICKS);
        animation_off_scheduled = true;
        animation_on_scheduled = false;
    }

    tim3_servo_animation.Tick();
    setduty(tim3_servo_animation.Value());
}

void EnableServoTimer(TIM_HandleTypeDef* tim) {
    __HAL_TIM_ENABLE_IT(tim, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(tim);
    TIM_CCxChannelCmd(tim->Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
    TIM_CCxChannelCmd(tim->Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
    TIM_CCxChannelCmd(tim->Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);
    TIM_CCxChannelCmd(tim->Instance, TIM_CHANNEL_4, TIM_CCx_ENABLE);
    __HAL_TIM_MOE_ENABLE(tim);
}

void adc1_2_interrupt_handler(void) {
    HAL_ADC_IRQHandler(&adc1_handle);
}

// Not being called in DMA mode; not sure why.
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adc) {
    ++adc_count;
}

//
//void adc3_interrupt_handler(void) {
//  HAL_ADC_IRQHandler()
//}

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

 void ADCInit(ADC_HandleTypeDef* hadc, int num_channels) {
    hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc->Init.Resolution = ADC_RESOLUTION_12B;
    hadc->Init.ScanConvMode = ENABLE;
    hadc->Init.ContinuousConvMode = ENABLE;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc->Init.NbrOfConversion = num_channels;
    hadc->Init.DMAContinuousRequests = ENABLE;
    hadc->Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc->Init.LowPowerAutoWait = DISABLE;
    hadc->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    HASSERT(HAL_ADC_Init(hadc) == HAL_OK);
}

void ADC_DMA_Init(ADC_HandleTypeDef* hadc, DMA_HandleTypeDef* hdma, DMA_Channel_TypeDef* dma_ch) {
    hdma->Instance = dma_ch;
    hdma->Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma->Init.MemInc = DMA_MINC_ENABLE;
    hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma->Init.Mode = DMA_CIRCULAR;
    hdma->Init.Priority = DMA_PRIORITY_LOW;
    HASSERT(HAL_DMA_Init(hdma) == HAL_OK);
    __HAL_LINKDMA(hadc, DMA_Handle, *hdma);
    HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);
}

void ADCChannelInit(ADC_HandleTypeDef* hadc, uint32_t channel, int rank) {
    ADC_ChannelConfTypeDef adc_channel = {0};
    adc_channel.SingleDiff = ADC_SINGLE_ENDED;
    adc_channel.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
    adc_channel.Rank = rank;
    adc_channel.Channel = channel;
    HASSERT(HAL_ADC_ConfigChannel(hadc, &adc_channel) == HAL_OK);
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
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_ADC34_CLK_ENABLE();

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

    /**
    * Set up servo timers.
    * TIM3 is a simple timer; TIM8 advanced.
    * For fun, let's use TIM3 here (since we already have adv. timer TIM1).
    * SRV_1: PC6, TIM3_CH1, TIM8_CH1 [TIM3: AF2]
    * SRV_2: PC8 TIM3_CH3, TIM8_CH3
    * SRV_3: PC7 TIM3_CH2, TIM8_CH2
    * SRV_4: PC9 TIM3_CH4, TIM8_CH4
    *
    * Target frequency: 50Hz
    * PSC: 1439; ARR: 999
    */
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF2_TIM3;
    for (const auto pin : { GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9 }) {
        gpio_init.Pin = pin;
        HAL_GPIO_Init(GPIOC, &gpio_init);
    }

    TIM_HandleTypeDef tim3;
    memset(&tim3, 0, sizeof(tim3));
    tim3.Instance = TIM3;
    tim3.Init.Prescaler = 1438;  // PSC-1
    tim3.Init.Period = TIM3_SERVO_NUM_BUCKETS - 1; // 1 less than Number of divisions for animation
    tim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    tim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HASSERT(HAL_TIM_PWM_Init(&tim3) == HAL_OK);

    TIM_OC_InitTypeDef tim3_OCInit;
    memset(&tim3_OCInit, 0, sizeof(tim3_OCInit));
    tim3_OCInit.OCMode = TIM_OCMODE_PWM1;  // on for X counts, then off.
    tim3_OCInit.Pulse = TIM3_SERVO_MIN_DUTY;
    tim3_OCInit.OCPolarity = TIM_OCPOLARITY_HIGH;
    tim3_OCInit.OCFastMode = TIM_OCFAST_DISABLE;
    for (const auto channel : { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 }) {
        HASSERT(HAL_TIM_PWM_ConfigChannel(&tim3, &tim3_OCInit, channel) == HAL_OK);
    }

    NVIC_SetPriority(TIM3_IRQn, 0xE0);
    NVIC_EnableIRQ(TIM3_IRQn);
    EnableServoTimer(&tim3);

    /**
     * Light detectors (phototransistors).
     * 
     * DET1: PA0 ADC1_IN1  (fast)
     * DET2: PA1 ADC1_IN2  (fast)
     * DET3: PA4 ADC2_IN1  (fast)
     * DET4: PB0 ADC3_IN12
     * DET5: PA6 ADC2_IN3  (fast)
     * DET6: PA7 ADC2_IN4  (fast)
     * DET7: PA2 ADC1_IN3  (fast)
     * DET8: PA3 ADC1_IN4  (fast)
     *
     * Grouped by ADC:
     *   ADC1: IN1=DET1, IN2=DET2, IN3=DET7, IN4=DET8
     *   ADC2: IN1=DET3,           IN3=DET5, IN4=DET6
     *   ADC3: IN12=DET4  <-- yes, IN12
     */
    adc1_handle = {0};
    adc1_handle.Instance = ADC1;
    ADCInit(&adc1_handle, 4);

    ADCChannelInit(&adc1_handle, ADC_CHANNEL_1, 1);
    ADCChannelInit(&adc1_handle, ADC_CHANNEL_2, 2);
    ADCChannelInit(&adc1_handle, ADC_CHANNEL_3, 3);
    ADCChannelInit(&adc1_handle, ADC_CHANNEL_4, 4);

    hdma_adc1 = {0};
    ADC_DMA_Init(&adc1_handle, &hdma_adc1, DMA1_Channel1);
    HAL_ADC_Start_DMA(&adc1_handle, (uint32_t*)adcDet1278, 4);

    adc2_handle = {0};
    adc2_handle.Instance = ADC2;
    ADCInit(&adc2_handle, 3);

    hdma_adc2 = {0};
    ADCChannelInit(&adc2_handle, ADC_CHANNEL_1, 1);
    ADCChannelInit(&adc2_handle, ADC_CHANNEL_3, 2);
    ADCChannelInit(&adc2_handle, ADC_CHANNEL_4, 3);

    ADC_DMA_Init(&adc2_handle, &hdma_adc2, DMA2_Channel1);
    HAL_ADC_Start_DMA(&adc2_handle, (uint32_t*)adcDet356, 3);

    adc3_handle = {0};
    adc3_handle.Instance = ADC3;
    ADCInit(&adc3_handle, 1);    

    hdma_adc3 = {0};
    ADCChannelInit(&adc3_handle, ADC_CHANNEL_12, 1);
    ADC_DMA_Init(&adc3_handle, &hdma_adc2, DMA2_Channel5);
    HAL_ADC_Start_DMA(&adc3_handle, (uint32_t*)&adcDet4, 1);

    NVIC_SetPriority(ADC1_2_IRQn, 0);
    NVIC_EnableIRQ(ADC1_2_IRQn);    

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
