/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Dual-Current Monitor — AC True RMS (LUT) & DC Average
  *          STM32F410RBT6 @ 100 MHz | FreeRTOS | DMA Double-Buffer
  *
  * v10-LUT: Replaces single AC multiplier with Look-Up Table interpolation.
  *
  * HOW TO RECALIBRATE THE AC LUT:
  *  1. Set DEBUG_RAW 1, flash, open PuTTY.
  *  2. With each known load, read the "RAW_AC_VRMS=x.xxxxx" value.
  *  3. Update ac_lut_raw[] with those V_rms values.
  *  4. Update ac_lut_actual[] with the multimeter current for each load.
  *  5. Set DEBUG_RAW 0, flash, done.
  *
  * CURRENT LUT POINTS (derived from project calibration data):
  *   V_rms=0.00000 → 0.000A (zero)
  *   V_rms=0.01221 → 0.770A (1 bulb  — may be in EMI noise, verify)
  *   V_rms=0.02474 → 1.560A (2 bulbs)
  *   V_rms=0.03695 → 2.330A (3 bulbs)
  *
  * AC NOISE: idle V_rms ≈ 0.01189V (EMI from live cable).
  *           1-bulb V_rms = 0.01221V. Gap = 0.32mV — very tight.
  *           AC_VRMS_NOISE_FLOOR = 0.01200V sits between them.
  *           If idle bleeds through, raise to 0.01230f (above 1-bulb).
  *           If 1-bulb is missed, lower to 0.01190f.
  *
  * DC calibration unchanged from v9.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Set 1 for PuTTY diagnostic — prints V_rms for LUT calibration           */
#define DEBUG_RAW  0

/*
 * DMA Buffer layout (400 interleaved samples):
 *   Even indices [0,2,4...] = DC  — Rank 1, ADC1_IN0, PA0, WCS1700
 *   Odd  indices [1,3,5...] = AC  — Rank 2, ADC1_IN1, PA1, SCT013
 *   HalfCplt  → stable elements   0–199 (100 DC + 100 AC)
 *   CpltCB    → stable elements 200–399 (100 DC + 100 AC)
 */
#define ADC_BUFFER_SIZE   400u
#define HALF_BUFFER       200u
#define SAMPLES_PER_CH    100u

#define VREF              3.25f
#define ADC_MAX           4095.0f

/* ---------------------------------------------------------------------------
 * AC LUT — maps V_rms (computed internally) → actual current in Amps.
 * LUT replaces single multiplier. Handles sensor non-linearity automatically.
 *
 * TO RECALIBRATE: set DEBUG_RAW 1, read RAW_AC_VRMS at each known load,
 * update ac_lut_raw[] values below, set DEBUG_RAW 0.
 *
 * AC_VRMS_NOISE_FLOOR: V_rms threshold below which output is forced to 0.
 * Idle EMI ≈ 0.01189V RMS. 1-bulb ≈ 0.01221V RMS. Floor between them.
 * --------------------------------------------------------------------------*/
#define AC_LUT_SIZE          4
#define AC_VRMS_NOISE_FLOOR  0.01195f   /* tuned: idle EMA=0.01189 below, 1-bulb EMA=0.01221 above */

static const float ac_lut_raw[AC_LUT_SIZE]    = {0.00000f, 0.01221f, 0.02474f, 0.03695f};
static const float ac_lut_actual[AC_LUT_SIZE] = {0.000f,   0.770f,   1.560f,   2.330f};

/* ---------------------------------------------------------------------------
 * DC constants — v9 calibrated values.
 *
 * DC_BIAS_OFFSET = 1.62265V (WCS1700 actual zero-current output)
 * WCS1700_SENSITIVITY = 0.02900 V/A
 * DC_NOISE_FLOOR = 0.358A (WCS1700 idle noise peak × 1.1)
 * --------------------------------------------------------------------------*/
#define DC_BIAS_OFFSET        1.62265f
#define WCS1700_SENSITIVITY   0.02900f
#define DC_NOISE_FLOOR        0.358f

/* Peripheral handles -------------------------------------------------------*/
ADC_HandleTypeDef  hadc1;
DMA_HandleTypeDef  hdma_adc1;
TIM_HandleTypeDef  htim5;
UART_HandleTypeDef huart2;

/* RTOS handles -------------------------------------------------------------*/
osThreadId_t    defaultTaskHandle;
osThreadId_t    Task_ProcessDCHandle;
osThreadId_t    Task_ProcessACHandle;
osThreadId_t    Task_TelemetryHandle;
osSemaphoreId_t Sem_AC_ReadyHandle;
osSemaphoreId_t Sem_DC_ReadyHandle;

/* RTOS attributes ----------------------------------------------------------*/
static const osThreadAttr_t defaultTask_attr = {
    .name = "defaultTask", .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal
};
static const osThreadAttr_t Task_DC_attr = {
    .name = "Task_ProcessDC", .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal
};
static const osThreadAttr_t Task_AC_attr = {
    .name = "Task_ProcessAC", .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityHigh
};
static const osThreadAttr_t Task_Tel_attr = {
    .name = "Task_Telemetry", .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow
};
static const osSemaphoreAttr_t Sem_AC_attr = { .name = "Sem_AC_Ready" };
static const osSemaphoreAttr_t Sem_DC_attr = { .name = "Sem_DC_Ready" };

/* Shared data --------------------------------------------------------------*/
volatile uint16_t        adc_buffer[ADC_BUFFER_SIZE];
volatile const uint16_t* active_half_ptr   = NULL;
volatile float           global_ac_current  = 0.0f;
volatile float           global_dc_current  = 0.0f;
volatile float           debug_ac_vrms      = 0.0f;

/* Private prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartTask_ProcessDC(void *argument);
void StartTask_ProcessAC(void *argument);
void StartTask_Telemetry(void *argument);

/* ---------------------------------------------------------------------------
 * AC_LUT_Lookup — linear interpolation through the calibration table.
 *
 * If v_rms < AC_VRMS_NOISE_FLOOR → return 0.0 (clamp noise)
 * If v_rms < lut[0]              → return 0.0
 * If v_rms > lut[last]           → extrapolate from last two points
 * Otherwise                      → interpolate between surrounding points
 * --------------------------------------------------------------------------*/
static float AC_LUT_Lookup(float v_rms)
{
    if (v_rms < AC_VRMS_NOISE_FLOOR) return 0.0f;

    /* Below first point */
    if (v_rms <= ac_lut_raw[0]) return ac_lut_actual[0];

    /* Above last point — extrapolate */
    if (v_rms >= ac_lut_raw[AC_LUT_SIZE - 1]) {
        float dv = ac_lut_raw[AC_LUT_SIZE-1] - ac_lut_raw[AC_LUT_SIZE-2];
        float di = ac_lut_actual[AC_LUT_SIZE-1] - ac_lut_actual[AC_LUT_SIZE-2];
        return ac_lut_actual[AC_LUT_SIZE-1] + (v_rms - ac_lut_raw[AC_LUT_SIZE-1]) * (di / dv);
    }

    /* Find surrounding segment and interpolate */
    for (int i = 0; i < AC_LUT_SIZE - 1; i++) {
        if (v_rms >= ac_lut_raw[i] && v_rms < ac_lut_raw[i + 1]) {
            float t  = (v_rms - ac_lut_raw[i]) / (ac_lut_raw[i+1] - ac_lut_raw[i]);
            return ac_lut_actual[i] + t * (ac_lut_actual[i+1] - ac_lut_actual[i]);
        }
    }

    return 0.0f;
}

/* ISR helper ---------------------------------------------------------------*/
static inline void Signal_ProcessingTasks_FromISR(const uint16_t* half)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    active_half_ptr = half;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)Sem_AC_ReadyHandle, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR((SemaphoreHandle_t)Sem_DC_ReadyHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
        Signal_ProcessingTasks_FromISR((const uint16_t*)&adc_buffer[0]);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
        Signal_ProcessingTasks_FromISR((const uint16_t*)&adc_buffer[HALF_BUFFER]);
}

/* main ---------------------------------------------------------------------*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM5_Init();
    MX_USART2_UART_Init();

    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);

    osKernelInitialize();

    Sem_AC_ReadyHandle = osSemaphoreNew(1, 0, &Sem_AC_attr);
    Sem_DC_ReadyHandle = osSemaphoreNew(1, 0, &Sem_DC_attr);

    defaultTaskHandle    = osThreadNew(StartDefaultTask,    NULL, &defaultTask_attr);
    Task_ProcessDCHandle = osThreadNew(StartTask_ProcessDC, NULL, &Task_DC_attr);
    Task_ProcessACHandle = osThreadNew(StartTask_ProcessAC, NULL, &Task_AC_attr);
    Task_TelemetryHandle = osThreadNew(StartTask_Telemetry, NULL, &Task_Tel_attr);

    osKernelStart();
    while (1) {}
}

/* Tasks --------------------------------------------------------------------*/

void StartDefaultTask(void *argument)
{
    for (;;) { osDelay(1000); }
}

/*
 * Task_ProcessDC
 * V_avg = (Σ raw[i] / N) × (VREF / ADC_MAX)
 * I_dc  = (V_avg − DC_BIAS_OFFSET) / WCS1700_SENSITIVITY
 */
void StartTask_ProcessDC(void *argument)
{
    for (;;)
    {
        if (osSemaphoreAcquire(Sem_DC_ReadyHandle, osWaitForever) == osOK)
        {
            const uint16_t* buf = (const uint16_t*)active_half_ptr;
            uint32_t dc_sum = 0u;

            for (uint32_t i = 0; i < HALF_BUFFER; i += 2) {
                dc_sum += buf[i];
            }

            float dc_avg_raw = (float)dc_sum / (float)SAMPLES_PER_CH;
            float dc_voltage = (dc_avg_raw / ADC_MAX) * VREF;
            float current    = (dc_voltage - DC_BIAS_OFFSET) / WCS1700_SENSITIVITY;

            if (current > -DC_NOISE_FLOOR && current < DC_NOISE_FLOOR) {
                current = 0.0f;
            }

            global_dc_current = current;
        }
    }
}

/*
 * Task_ProcessAC — Two-Pass Dynamic Auto-Zero + LUT Lookup
 *
 * Pass 1: Compute actual mean of 100 AC samples = DC bias voltage.
 *         100 samples @ 1kHz/ch = 100ms = exactly 5 × 50Hz cycles.
 *
 * Pass 2: Subtract mean → sum-of-squares → sqrt → V_rms.
 *         V_rms fed into AC_LUT_Lookup() instead of fixed multiplier.
 *         LUT handles any sensor non-linearity across the load range.
 */
void StartTask_ProcessAC(void *argument)
{
    for (;;)
    {
        if (osSemaphoreAcquire(Sem_AC_ReadyHandle, osWaitForever) == osOK)
        {
            const uint16_t* buf = (const uint16_t*)active_half_ptr;

            /* Pass 1: dynamic bias */
            float raw_sum = 0.0f;
            for (uint32_t i = 1; i < HALF_BUFFER; i += 2) {
                raw_sum += ((float)buf[i] / ADC_MAX) * VREF;
            }
            float dynamic_bias = raw_sum / (float)SAMPLES_PER_CH;

            /* Pass 2: True RMS */
            float sum_sq = 0.0f;
            for (uint32_t i = 1; i < HALF_BUFFER; i += 2) {
                float v = ((float)buf[i] / ADC_MAX) * VREF - dynamic_bias;
                sum_sq += (v * v);
            }

            float v_rms_raw   = sqrtf(sum_sq / (float)SAMPLES_PER_CH);

            /* Exponential Moving Average — suppresses threshold bounce.
             * alpha=0.7: fast enough to respond in ~2s, smooths raw noise by 3×.
             * v_rms_ema fluctuates ±0.0003V vs raw ±0.001V — sits stably above
             * or below AC_VRMS_NOISE_FLOOR without toggling every sample.       */
            static float v_rms_ema = 0.0f;
            v_rms_ema     = 0.3f * v_rms_ema + 0.7f * v_rms_raw;

            debug_ac_vrms = v_rms_ema;  /* saved for DEBUG_RAW calibration */

            global_ac_current = AC_LUT_Lookup(v_rms_ema);
        }
    }
}

/*
 * Task_Telemetry
 * DEBUG_RAW 0: "2.330,1.920\r\n"
 * DEBUG_RAW 1: "RAW_AC_VRMS=0.03695 AC=2.330A DC=1.920A\r\n"
 *              Use RAW_AC_VRMS values to populate ac_lut_raw[] above.
 */
void StartTask_Telemetry(void *argument)
{
    char uart_buf[72];

    for (;;)
    {
        float rx_ac = global_ac_current;
        float rx_dc = global_dc_current;

#if DEBUG_RAW
        snprintf(uart_buf, sizeof(uart_buf),
                 "RAW_AC_VRMS=%.5f AC=%.3fA DC=%.3fA\r\n",
                 debug_ac_vrms, rx_ac, rx_dc);
#else
        snprintf(uart_buf, sizeof(uart_buf), "%.3f,%.3f\r\n", rx_ac, rx_dc);
#endif

        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf,
                          strlen(uart_buf), HAL_MAX_DELAY);
        osDelay(500);
    }
}

/* Peripheral init ----------------------------------------------------------*/

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 8;
    RCC_OscInitStruct.PLL.PLLN            = 100;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ENABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T5_CC1;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 2;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

    sConfig.Channel      = ADC_CHANNEL_0;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

    sConfig.Channel      = ADC_CHANNEL_1;
    sConfig.Rank         = 2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM5_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};
    TIM_OC_InitTypeDef      sConfigOC          = {0};

    htim5.Instance               = TIM5;
    htim5.Init.Prescaler         = 99;
    htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim5.Init.Period            = 499;
    htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK) { Error_Handler(); }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) { Error_Handler(); }

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 250;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM11) { HAL_IncTick(); }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
