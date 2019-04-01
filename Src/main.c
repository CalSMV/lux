/**
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  ******************************************************************************
  */

#include "main.h"
#include "stdbool.h"

typedef double percent;
typedef struct {
  bool hall_val_1;
  bool hall_val_2;
  bool hall_val_3;
} hallVals;
typedef struct {
  int phase_1_state;    //Phase A
  int phase_2_state;    //Phase B
  int phase_3_state;    //Phase C
} phaseVals;            //0: Enable 0, PWM off; 1: Enable 1, PWM on; -1: Enable 1, PWM off

#define REVERSIBLE_MOTOR false
#define MAX_DUTY_CYCLE   INT_MAX

ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim8;

static char lookup_table[8] = {0, 49, 14, 52, 28, 13, 19, 0};   //Bitfield representation

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);

inline percent   getAccel();
inline bool      getBrake();
inline hallVals  getHalls();
inline phaseVals getPhase(hallVals sensor_values, bool brake_state); //TODO Will call the lookup table for info

inline void enableDrivers(bool driver_1, bool driver_2, bool driver_3);
inline void setDutyCycle(double pwm_duty_cycle);
inline void enablePWM(bool pwm_1, bool pwm_2, bool pwm_3);

inline void initPWM();

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  initPWM();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();

  /* Infinite loop */
  while (1)
  {
    percent   accel  = getAccel();
    bool      brake  = getBrake();
    hallVals  halls  = getHalls();
    phaseVals motors = getPhase(halls, brake);
    
    enableDrivers(motors.phase_1_state, motors.phase_2_state, motors.phase_3_state);
    setDutyCycle(brake ? accel : 0);
    enablePWM(motors.phase_1_state > 0, motors.phase_2_state > 0, motors.phase_3_state > 0);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM8_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim8);

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, Enable_1_Pin|Enable_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Enable_2_GPIO_Port, Enable_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Enable_1_Pin Enable_3_Pin */
  GPIO_InitStruct.Pin = Enable_1_Pin|Enable_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : Hall_1_Pin Hall_2_Pin */
  GPIO_InitStruct.Pin = Hall_1_Pin|Hall_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pin : Hall_3_Pin */
  GPIO_InitStruct.Pin = Hall_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Hall_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Brake_Pin */
  GPIO_InitStruct.Pin = Brake_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Brake_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Enable_2_Pin */
  GPIO_InitStruct.Pin = Enable_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Enable_2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
inline percent   getAccel() {
  return HAL_ADC_GetValue(&hadc1);
}
inline bool      getBrake() {
  return HAL_GPIO_ReadPin(Brake_GPIO_Port, Brake_Pin);
}
inline hallVals  getHalls() {
  hallVals i;
  i.hall_val_1 = HAL_GPIO_ReadPin(Hall_1_GPIO_Port, Hall_1_Pin);
  i.hall_val_2 = HAL_GPIO_ReadPin(Hall_2_GPIO_Port, Hall_2_Pin);
  i.hall_val_3 = HAL_GPIO_ReadPin(Hall_3_GPIO_Port, Hall_3_Pin);
  return i;
}
inline phaseVals getPhase(hallVals sensor_values, bool brake_state) {
  //Doing this without lookup table for now, should be fine
  phaseVals i;
  i.phase_1_state = 0;
  i.phase_2_state = 0;
  i.phase_3_state = 0;
  
  if (brake_state) {
    return i;
  }

  if (REVERSIBLE_MOTOR) { //Counterclockwise rotation not currently supported
    Error_Handler();
  }
  
  int phase = 1*sensor_values.hall_val_1 + 2*sensor_values.hall_val_2 + 4*sensor_values.hall_val_3;
  switch(phase) {
  case 0: //Invalid state, no halls active
    Error_Handler();
    return i;
  case 1: //Phase II
    i.phase_1_state =  1;
    i.phase_2_state =  0;
    i.phase_3_state = -1;
    return i;
  case 2: //Phase IV
    i.phase_1_state = -1;
    i.phase_2_state =  1;
    i.phase_3_state =  0;
    return i;
  case 3: //Phase III
    i.phase_1_state =  0;
    i.phase_2_state =  1;
    i.phase_3_state = -1;
    return i;
  case 4: //Phase VI
    i.phase_1_state =  0;
    i.phase_2_state = -1;
    i.phase_3_state =  1;
    return i;
  case 5: //Phase I
    i.phase_1_state =  1;
    i.phase_2_state = -1;
    i.phase_3_state =  0;
    return i;
  case 6: //Phase V
    i.phase_1_state = -1;
    i.phase_2_state =  0;
    i.phase_3_state =  1;
    return i;
  case 7: //Invalid state, all halls active
    Error_Handler();
    return i;
  }
  
  return i;
}

inline void enableDrivers(bool driver_1, bool driver_2, bool driver_3) {
  HAL_GPIO_WritePin(Enable_1_GPIO_Port, Enable_1_Pin, driver_1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Enable_2_GPIO_Port, Enable_2_Pin, driver_2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Enable_3_GPIO_Port, Enable_3_Pin, driver_3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
inline void setDutyCycle(double pwm_duty_cycle) {
  
}
inline void enablePWM(bool pwm_1, bool pwm_2, bool pwm_3) {
  
}
inline void initPWM() {
  setDutyCycle(0);
  enablePWM(0, 0, 0);
  //SET THE FREQUENCY
}

void Error_Handler(void)
{
  while(1) { ; }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
  while(1) { ; }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
