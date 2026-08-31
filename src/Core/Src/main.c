/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- Failsafe timer configuration ---
#define PULSE_TIMEOUT_MS 100 // 100 ms at 50 Hz is 5 missed pulses I think

// --- MIN BATTERY VOLTAGE CUTOFF CONFIGURATION ---
/* Calculate this yourself:
 * 1. Multiply min cell voltage by number of cells to get min battery voltage
 * 		v_batt_min = v_cell_min * batt_s
 * 2. Calculate voltage divider gain based on R1 and R2 in https://www.digikey.com/en/resources/conversion-calculators/conversion-calculator-voltage-divider
 * 		divider_gain = r2 / (r1 + r2)
 * 3. Calculate min divided voltage (input to ADC pin)
 * 		v_divided_min = v_batt_min * divider_gain
 * 4. Calculate min ADC counts (here we assume 12 bit ADC, so (2^12)-1=4095 max counts at VREF, which we assume is 3.3V)
 * 		MIN_BATT_ADC = (v_divided_min / 3.3) * 4095
 * 5. Round it to the nearest whole number and enter below
 */

#define MIN_BATT_ADC 2286 // 2286 for safe 10.5V min (3.5V/cell) or use 1959 for 9V (3V/cell) absolute min
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
// --- Channel 1 (RX_CH1 -> TIM3_CH2 -> Motor 1) ---
// Note: RX_CH1_Pin (PB7) is TIM3_CH2
volatile uint32_t IC_Val1_CH1 = 0;
volatile uint32_t IC_Val2_CH1 = 0;
volatile uint32_t Pulse_Width_CH1 = 1500; // Default to neutral
volatile uint8_t Is_First_Captured_CH1 = 0; // 0 = waiting for rising, 1 = waiting for falling
volatile uint32_t last_pulse_time_ch1 = 0; // For failsafe

// --- Channel 2 (RX_CH2 -> TIM3_CH1 -> Motor 2) ---
// Note: RX_CH2_Pin (PB6) is TIM3_CH1
volatile uint32_t IC_Val1_CH2 = 0;
volatile uint32_t IC_Val2_CH2 = 0;
volatile uint32_t Pulse_Width_CH2 = 1500; // Default to neutral
volatile uint8_t Is_First_Captured_CH2 = 0; // 0 = waiting for rising, 1 = waiting for falling
volatile uint32_t last_pulse_time_ch2 = 0; // For failsafe

volatile uint32_t now;
uint32_t pulse1, pulse2;
uint32_t last_seen1, last_seen2;
uint8_t error;
uint8_t batt_cutoff;
volatile uint32_t batt_adc_reading = 69;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void set_motor_speed(uint32_t pulse, uint32_t tim_channel_in1, uint32_t tim_channel_in2);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // Turn on LED to indicate boot
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);

  // --- Start Motor 1 (DRV1) PWM Channels ---
  // DRV1_IN1 (PA1) -> TIM1_CH2
  // DRV1_IN2 (PA2) -> TIM1_CH3
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  // --- Start Motor 2 (DRV2) PWM Channels ---
  // DRV2_IN1 (PA11) -> TIM1_CH4
  // DRV2_IN2 (PA8)  -> TIM1_CH1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  // Set all motor outputs to 0 (Coast) initially
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

  // --- Start Input Capture Interrupts ---
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1); // RX_CH2 (PB6) -> Motor 2
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2); // RX_CH1 (PB7) -> Motor 1

  // Start ADC continuous mode
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start(&hadc1);

  // Initialize failsafe timestamps
  now = HAL_GetTick();
  last_pulse_time_ch1 = now;
  last_pulse_time_ch2 = now;
  error = 0;
  batt_cutoff = 0;

  // Turn off LED to indicate success
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  	now = HAL_GetTick();
  	error = 0;

   	batt_adc_reading = HAL_ADC_GetValue(&hadc1);
//  	if (batt_adc_reading < MIN_BATT_ADC) {
//  		batt_cutoff = 1;
//  		error = 1;
//  	} else {
//  		batt_cutoff = 0;
//  	}

  	// Atomically get the latest pulse data from the ISR
  	HAL_NVIC_DisableIRQ(TIM3_IRQn);
  	pulse1 = Pulse_Width_CH1;
  	last_seen1 = last_pulse_time_ch1;
  	pulse2 = Pulse_Width_CH2;
  	last_seen2 = last_pulse_time_ch2;
  	HAL_NVIC_EnableIRQ(TIM3_IRQn);

  	if (!batt_cutoff) {
			if (now - last_seen1 > PULSE_TIMEOUT_MS) // motor 1 failsafe
			{
				// coast IN1=IN2=0
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

				error = 1;
			}
			else // motor 1 good
			{
				set_motor_speed(pulse1, TIM_CHANNEL_2, TIM_CHANNEL_3);
			}

			if (now - last_seen2 > PULSE_TIMEOUT_MS) // motor 2 failsafe
			{
				// coast IN1=IN2=0
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

				error = 1;
			}
			else // motor 2 good
			{
				set_motor_speed(pulse2, TIM_CHANNEL_4, TIM_CHANNEL_1);
			}
  	}

  	// write error state to LED each time
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, !error);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 11;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Converts a 1000-2000us pulse to motor driver PWM outputs
  * @param  pulse: The measured pulse width in microseconds (1000-2000)
  * @param  tim_channel_in1: The TIM1 channel for the driver's IN1 pin
  * @param  tim_channel_in2: The TIM1 channel for the driver's IN2 pin
  * @retval None
  */
void set_motor_speed(uint32_t pulse, uint32_t tim_channel_in1, uint32_t tim_channel_in2)
{
    uint32_t duty = 0;
    const uint32_t MAX_DUTY = 2399; // from htim1.Init.Period
    const uint32_t NEUTRAL_LOW = 1450;
    const uint32_t NEUTRAL_HIGH = 1550;
    const uint32_t MIN_PULSE = 1000;
    const uint32_t MAX_PULSE = 2000;

    // --- Failsafe ---
    // If pulse is way out of range, default to neutral
    if (pulse < 500 || pulse > 2500)
    {
//        pulse = 1500;
    	error = 1;
//        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
    }


    // --- Neutral / Coast ---
    // (DRV8231A Table 8-2: IN1=0, IN2=0 -> Coast)
//    if (pulse > NEUTRAL_LOW && pulse < NEUTRAL_HIGH)
//    {
//        __HAL_TIM_SET_COMPARE(&htim1, tim_channel_in1, 0);
//        __HAL_TIM_SET_COMPARE(&htim1, tim_channel_in2, 0);
//    }
//     --- Neutral / Brake ---
//     (DRV8231A Table 8-2: IN1=0, IN2=0 -> Coast)
    if (pulse > NEUTRAL_LOW && pulse < NEUTRAL_HIGH)
    {
        __HAL_TIM_SET_COMPARE(&htim1, tim_channel_in1, MAX_DUTY);
        __HAL_TIM_SET_COMPARE(&htim1, tim_channel_in2, MAX_DUTY);
    }
    // --- Forward ---
    // (DRV8231A Table 8-2: IN1=PWM, IN2=0 -> Forward)
    else if (pulse >= NEUTRAL_HIGH)
    {
        if(pulse > MAX_PULSE) pulse = MAX_PULSE;

        // Map [1550...2000] to [0...599]
        duty = (pulse - NEUTRAL_HIGH) * MAX_DUTY / (MAX_PULSE - NEUTRAL_HIGH);
        if (duty > MAX_DUTY) duty = MAX_DUTY;

        __HAL_TIM_SET_COMPARE(&htim1, tim_channel_in1, duty); // IN1 = PWM
        __HAL_TIM_SET_COMPARE(&htim1, tim_channel_in2, 0);    // IN2 = 0
    }
    // --- Reverse ---
    // (DRV8231A Table 8-2: IN1=0, IN2=PWM -> Reverse)
    else if (pulse <= NEUTRAL_LOW)
    {
        if(pulse < MIN_PULSE) pulse = MIN_PULSE;

        // Map [1450...1000] to [0...599]
        duty = (NEUTRAL_LOW - pulse) * MAX_DUTY / (NEUTRAL_LOW - MIN_PULSE);
        if (duty > MAX_DUTY) duty = MAX_DUTY;

        __HAL_TIM_SET_COMPARE(&htim1, tim_channel_in1, 0);    // IN1 = 0
        __HAL_TIM_SET_COMPARE(&htim1, tim_channel_in2, duty); // IN2 = PWM
    }
}

/**
  * @brief  Input Capture callback in non-blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        // --- Channel 2 (PB7) -> RX_CH1 -> Motor 1 ---
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        {
            if (Is_First_Captured_CH1 == 0) // First edge (Rising)
            {
                IC_Val1_CH1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
                Is_First_Captured_CH1 = 1;
                // Change polarity to falling edge
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
            }
            else  // Second edge (Falling)
            {
                IC_Val2_CH1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
                // --- FIX: Use a unique variable name ---
                uint32_t new_pulse_width_ch1 = 0;

                if (IC_Val2_CH1 > IC_Val1_CH1)
                {
                    new_pulse_width_ch1 = IC_Val2_CH1 - IC_Val1_CH1;
                }
                else if (IC_Val1_CH1 > IC_Val2_CH1) // Timer overflow
                {
                    new_pulse_width_ch1 = (0xFFFF - IC_Val1_CH1) + IC_Val2_CH1 + 1;
                }

                // --- Validate and save for CH1 ---
                // Discard any junk pulses outside the valid 800-2200us range
                if (new_pulse_width_ch1 > 800 && new_pulse_width_ch1 < 2200)
                {
                    Pulse_Width_CH1 = new_pulse_width_ch1; // Write to CH1's variable
                    last_pulse_time_ch1 = HAL_GetTick();   // Write to CH1's timestamp
                }

                Is_First_Captured_CH1 = 0;
                // Change polarity back to rising edge
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
            }
        }

        // --- Channel 1 (PB6) -> RX_CH2 -> Motor 2 ---
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            if (Is_First_Captured_CH2 == 0) // First edge (Rising)
            {
                IC_Val1_CH2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                Is_First_Captured_CH2 = 1;
                // Change polarity to falling edge
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
            }
            else  // Second edge (Falling)
            {
                IC_Val2_CH2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                // --- FIX: Use a unique variable name ---
                uint32_t new_pulse_width_ch2 = 0;

                if (IC_Val2_CH2 > IC_Val1_CH2)
                {
                    new_pulse_width_ch2 = IC_Val2_CH2 - IC_Val1_CH2;
                }
                else if (IC_Val1_CH2 > IC_Val2_CH2) // Timer overflow
                {
                    new_pulse_width_ch2 = (0xFFFF - IC_Val1_CH2) + IC_Val2_CH2 + 1;
                }

                // --- Validate and save for CH2 ---
                // Discard any junk pulses outside the valid 800-2200us range
                if (new_pulse_width_ch2 > 800 && new_pulse_width_ch2 < 2200)
                {
                    Pulse_Width_CH2 = new_pulse_width_ch2; // Write to CH2's variable
                    last_pulse_time_ch2 = HAL_GetTick();   // Write to CH2's timestamp
                }

                Is_First_Captured_CH2 = 0;
                // Change polarity back to rising edge
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            }
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
