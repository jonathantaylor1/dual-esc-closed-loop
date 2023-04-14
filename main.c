/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


uint8_t txdata[TX_DATA_SIZE];
uint16_t adcdata [ADC_DATA_SIZE];
uint32_t txlength = 0;
uint8_t rxdata = 0; // for UART receive data

const uint32_t PRD_ARR_SIZE = PRD_ARR_OPEN_ACC_NUMEL + PRD_ARR_CLOSED_REF_NUMEL;
float_t prdArr[PRD_ARR_OPEN_ACC_NUMEL + PRD_ARR_CLOSED_REF_NUMEL];

int16_t piSum = 0;

Motor_TypeDef motor1;
Motor_TypeDef motor2;

uint8_t ctrlState = STATE_CTRL_RUN;
uint8_t sysEn = 0;
uint32_t uvloCtr = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void setPWM(Motor_TypeDef* motor, uint8_t newStep, uint32_t compare);
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
	// Initialize open period array
	arraysInit();
	motorsInit();


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  LL_TIM_EnableCounter(TIM1);
  //adcConvert();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_3)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 100, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(100000000);
  LL_SetSystemCoreClock(100000000);
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
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

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**ADC1 GPIO Configuration
  PA4   ------> ADC1_IN4
  PA5   ------> ADC1_IN5
  PA6   ------> ADC1_IN6
  PA7   ------> ADC1_IN7
  PC4   ------> ADC1_IN14
  PB0   ------> ADC1_IN8
  PB1   ------> ADC1_IN9
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* ADC1 DMA Init */

  /* ADC1 Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_0, LL_DMA_CHANNEL_0);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_0);

  /* ADC1 interrupt Init */
  NVIC_SetPriority(ADC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(ADC_IRQn);

  /* USER CODE BEGIN ADC1_Init 1 */



  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_ENABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM5_CH1;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_6);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_15CYCLES);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_9);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_9, LL_ADC_SAMPLINGTIME_15CYCLES);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_14);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_14, LL_ADC_SAMPLINGTIME_15CYCLES);
  /* USER CODE BEGIN ADC1_Init 2 */
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_15CYCLES);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_15CYCLES);

  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_15CYCLES);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_15CYCLES);

  LL_DMA_DisableStream(DMA2,  LL_DMA_STREAM_0);

  LL_DMA_ClearFlag_TC0(DMA2);

  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);

  LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_0, (uint32_t) adcdata);
  LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_0, (uint32_t) &(ADC1->DR));

  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, ADC_DATA_SIZE);

  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);

  LL_ADC_ClearFlag_EOCS(ADC1);
  LL_ADC_EnableIT_EOCS(ADC1);
  LL_ADC_Enable(ADC1);

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* USER CODE BEGIN TIM1_Init 1 */
  LL_TIM_SetSlaveMode(TIM1, LL_TIM_SLAVEMODE_DISABLED);
  LL_TIM_EnableUpdateEvent(TIM1);
  LL_TIM_SetUpdateSource(TIM1, LL_TIM_UPDATESOURCE_REGULAR);
  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_CENTER_UP_DOWN;
  TIM_InitStruct.Autoreload = 2000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 1;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_ENABLE);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */
  LL_TIM_SetCounter(TIM1, 108);
  LL_TIM_ClearFlag_UPDATE(TIM1);
  LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_EnableAllOutputs(TIM1);
  /* USER CODE END TIM1_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration
  PA8   ------> TIM1_CH1
  PA9   ------> TIM1_CH2
  PA10   ------> TIM1_CH3
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* USER CODE BEGIN TIM2_Init 1 */
  LL_TIM_EnableUpdateEvent(TIM2);
  LL_TIM_SetUpdateSource(TIM2, LL_TIM_UPDATESOURCE_REGULAR);
  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_CENTER_UP_DOWN;
  TIM_InitStruct.Autoreload = 2000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerInput(TIM2, LL_TIM_TS_ITR0);
  LL_TIM_SetSlaveMode(TIM2, LL_TIM_SLAVEMODE_TRIGGER);
  LL_TIM_DisableIT_TRIG(TIM2);
  LL_TIM_DisableDMAReq_TRIG(TIM2);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_ENABLE);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */
  LL_TIM_SetCounter(TIM2, 108);
  //LL_TIM_ClearFlag_UPDATE(TIM2);
  LL_TIM_DisableIT_UPDATE(TIM2);
  LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH); // Set A_EN polarity
  LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH); // Set B_EN polarity
  LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH); // Set C_EN polarity
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4);
  //LL_TIM_EnableAllOutputs(TIM2);
  /* USER CODE END TIM2_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM2 GPIO Configuration
  PA1   ------> TIM2_CH2
  PA2   ------> TIM2_CH3
  PA3   ------> TIM2_CH4
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_CENTER_UP_DOWN;
  TIM_InitStruct.Autoreload = 2000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR0);
  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);
  LL_TIM_DisableIT_TRIG(TIM3);
  LL_TIM_DisableDMAReq_TRIG(TIM3);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */
  LL_TIM_ClearFlag_UPDATE(TIM3);
  LL_TIM_DisableIT_UPDATE(TIM3);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3);
  //LL_TIM_EnableAllOutputs(TIM3);
  /* USER CODE END TIM3_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  /**TIM3 GPIO Configuration
  PC6   ------> TIM3_CH1
  PC7   ------> TIM3_CH2
  PC8   ------> TIM3_CH3
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7|LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_CENTER_UP_DOWN;
  TIM_InitStruct.Autoreload = 2000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM4);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerInput(TIM4, LL_TIM_TS_ITR0);
  LL_TIM_SetSlaveMode(TIM4, LL_TIM_SLAVEMODE_TRIGGER);
  LL_TIM_DisableIT_TRIG(TIM4);
  LL_TIM_DisableDMAReq_TRIG(TIM4);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM4);
  /* USER CODE BEGIN TIM4_Init 2 */
  LL_TIM_ClearFlag_UPDATE(TIM4);
  LL_TIM_DisableIT_UPDATE(TIM4);
  LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH); // Set U_EN polarity
  LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH); // Set V_EN polarity
  LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH); // Set W_EN polarity
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
  //LL_TIM_EnableAllOutputs(TIM4);
  /* USER CODE END TIM4_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**TIM4 GPIO Configuration
  PB6   ------> TIM4_CH1
  PB7   ------> TIM4_CH2
  PB8   ------> TIM4_CH3
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7|LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);

  /* TIM5 interrupt Init */
  NVIC_SetPriority(TIM5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM5_IRQn);

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 3999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM5, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM5);
  LL_TIM_SetClockSource(TIM5, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM5, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 2000;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM5, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM5, LL_TIM_CHANNEL_CH1);
  LL_TIM_SetTriggerInput(TIM5, LL_TIM_TS_ITR0);
  LL_TIM_SetSlaveMode(TIM5, LL_TIM_SLAVEMODE_TRIGGER);
  LL_TIM_DisableIT_TRIG(TIM5);
  LL_TIM_DisableDMAReq_TRIG(TIM5);
  LL_TIM_SetTriggerOutput(TIM5, LL_TIM_TRGO_OC1REF);
  LL_TIM_DisableMasterSlaveMode(TIM5);
  /* USER CODE BEGIN TIM5_Init 2 */
  LL_TIM_OC_SetPolarity(TIM5, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH1);
  LL_TIM_SetCounter(TIM5, 108 + 0*14861);
  LL_TIM_SetUpdateSource(TIM5, LL_TIM_UPDATESOURCE_COUNTER);
  LL_TIM_DisableIT_UPDATE(TIM5);
  LL_TIM_DisableIT_BRK(TIM5);
  LL_TIM_DisableIT_CC1(TIM5);
  LL_TIM_DisableIT_COM(TIM5);
  LL_TIM_DisableIT_TRIG(TIM5);
  /* USER CODE END TIM5_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM5 GPIO Configuration
  PA0-WKUP   ------> TIM5_CH1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART6 GPIO Configuration
  PA11   ------> USART6_TX
  PA12   ------> USART6_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11|LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART6 DMA Init */

  /* USART6_RX Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_1, LL_DMA_CHANNEL_5);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_1, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MEMORY_NOINCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_1);

  /* USART6_TX Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_6, LL_DMA_CHANNEL_5);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_6, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_6, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_6, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_6, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_6, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_6, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_EnableFifoMode(DMA2, LL_DMA_STREAM_6);

  LL_DMA_SetFIFOThreshold(DMA2, LL_DMA_STREAM_6, LL_DMA_FIFOTHRESHOLD_1_4);

  LL_DMA_SetMemoryBurstxfer(DMA2, LL_DMA_STREAM_6, LL_DMA_MBURST_SINGLE);

  LL_DMA_SetPeriphBurstxfer(DMA2, LL_DMA_STREAM_6, LL_DMA_PBURST_SINGLE);

  /* USART6 interrupt Init */
  NVIC_SetPriority(USART6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART6_IRQn);

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_9B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_EVEN;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART6, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART6);
  LL_USART_Enable(USART6);
  /* USER CODE BEGIN USART6_Init 2 */

  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_6); // Disable stream 6 (TX) to configure DMA
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_1); // Disable stream 1 (RX) to configure DMA

  LL_DMA_ClearFlag_TC6(DMA2); // Clear stream 6 TC so interrupt not immediately triggered when TCIE set
  LL_DMA_ClearFlag_TC1(DMA2); // Clear stream 1 TC so interrupt not immediately triggered when TCIE set

  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_6); // Setting TCIE for stream 6 (TX)
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_1); // Setting TCIE for stream 1 (RX)

  LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_1, (uint32_t) &rxdata); // Set rxdata buffer address
  LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_1, (uint32_t) &(USART6->DR)); // Set peripheral

  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, RX_DATA_SIZE); // Setting NDTR register

  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1); // Enable DMA Stream 1 (RX)

  // --- USART CODE ---
  // MX_USART6_UART_Init() already sets USART_CR1_M, USART_CR1_PCE,
  // USART_CR1_PS, USART_CR1_TE, USART_CR1_RE, USART_CR1_OVER8,
  // USART_CR1_UE, USART_CR2_LINEN, USART_CR2_CLKEN, USART_CR3_SCEN,
  // USART_CR3_IREN, USART_CR3_HDSEL, USART_CR2_STOP, USART_CR3_RTSE,
  // USART_CR3_CTSE

  // Set USART DMAT bit to enable DMA requests for the USART6 TX module.
  LL_USART_EnableDMAReq_TX(USART6);
  LL_USART_EnableDMAReq_RX(USART6);
  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_13;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(B1_GPIO_Port, B1_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(B1_GPIO_Port, B1_Pin, LL_GPIO_MODE_INPUT);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void motorsInit(void)
{
	// Initialize motor structs
	motor1.phaseId = PHASE_ID_ABC;
	motor1.zPhaseAdcRank = LL_ADC_REG_RANK_1;
	motor1.phase1adcChannel = LL_ADC_CHANNEL_4;
	motor1.phase2adcChannel = LL_ADC_CHANNEL_5;
	motor1.phase3adcChannel = LL_ADC_CHANNEL_6;
	motor1.dcBusAdcChannel = LL_ADC_CHANNEL_14;
	motor1.adcDataZPhaseIdx = ADC_MOTOR1_Z;
	motor1.adcDataDcBusIdx = ADC_DCBUS;
	motor1.state = STATE_MOTOR_STOP;
	motor1.step = STEP_Z;
	motor1.dir = MOTOR_DIRECTION_FWD;
	motor1.openPrdCtr = 0;
	motor1.cmdPrd = 0xffffffff;
	motor1.cmdFreq = 0;
	motor1.getSpdCtr = 10;
	motor1.openPrdArrIdx = 0;
	motor1.phase1PWMCompare = 0;
	motor1.phase1EnableCompare = 0;
	motor1.phase2PWMCompare = 0;
	motor1.phase2EnableCompare = 0;
	motor1.phase3PWMCompare = 0;
	motor1.phase3EnableCompare = 0;
	motor1.isPrevValValid = 0;
	motor1.isMeasuredFreqValid = 0;
	motor1.piMax = OPEN_DUTY_COEFF/5+OPEN_DUTY_OFFSET;
	motor1.piMin = OPEN_DUTY_COEFF/prdArr[PRD_ARR_OPEN_ACC_NUMEL-1] + OPEN_DUTY_OFFSET;

	motor2.phaseId = PHASE_ID_UVW;
	motor2.zPhaseAdcRank = LL_ADC_REG_RANK_2;
	motor2.phase1adcChannel = LL_ADC_CHANNEL_7;
	motor2.phase2adcChannel = LL_ADC_CHANNEL_8;
	motor2.phase3adcChannel = LL_ADC_CHANNEL_9;
	motor2.dcBusAdcChannel = LL_ADC_CHANNEL_14;
	motor2.adcDataZPhaseIdx = ADC_MOTOR2_Z;
	motor2.adcDataDcBusIdx = ADC_DCBUS;
	motor2.state = STATE_MOTOR_STOP;
	motor2.step = STEP_Z;
	motor2.dir = MOTOR_DIRECTION_FWD;
	motor2.openPrdCtr = 0;
	motor2.cmdPrd = 0xffffffff;
	motor2.cmdFreq = 0;
	motor2.getSpdCtr = 10;
	motor2.openPrdArrIdx = 0;
	motor2.phase1PWMCompare = 0;
	motor2.phase1EnableCompare = 0;
	motor2.phase2PWMCompare = 0;
	motor2.phase2EnableCompare = 0;
	motor2.phase3PWMCompare = 0;
	motor2.phase3EnableCompare = 0;
	motor2.isPrevValValid = 0;
	motor2.isMeasuredFreqValid = 0;
	motor2.piMax = OPEN_DUTY_COEFF/5+OPEN_DUTY_OFFSET;
	motor2.piMin = OPEN_DUTY_COEFF/prdArr[PRD_ARR_OPEN_ACC_NUMEL-1] + OPEN_DUTY_OFFSET;
}

void arraysInit(void)
{
	prdArr[0] = OPEN_INIT_PRD;
	for(int i = 1; i < PRD_ARR_SIZE; i++)
	{
		prdArr[i] = (prdArr[i-1]/(OPEN_ALPHA*6.0*prdArr[i-1]*prdArr[i-1]*T_SAMP*T_SAMP+1.0));
	}
}

void adcConvert(void)
{

	LL_ADC_REG_StartConversionSWStart(ADC1);

}

void uartTx(uint32_t* txdataptr, uint32_t datasize)
{
	if(LL_DMA_IsActiveFlag_DME6(DMA2) == 0)
	{
		LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_6, (uint32_t) &(USART6->DR)); // Set peripheral destination address
		LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_6, (uint32_t) txdataptr); // Set memory source address
		LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_6, datasize); // Set DMA_SxNDTR register
	}
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_6); // Set DMA EN bit
}

void uartRxCpltCallback()
{
	switch(ctrlState)
	{
	case STATE_CTRL_DEBUG:
		switch(rxdata)
		{
		case 0xff:
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Entering run mode...");
			uartTx((uint32_t*) txdata, txlength);
			motor1.state = STATE_MOTOR_STOP;
			motor2.state = STATE_MOTOR_STOP;
			motor1.cmdPrd = 100;
			motor2.cmdPrd = 100;
			ctrlState = STATE_CTRL_RUN;
			break;
		case 0xdd:
			setPWM(&motor1, STEP_Z, DEBUG_DUTY);
			setPWM(&motor2, STEP_Z, DEBUG_DUTY);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Debug step Z");
			uartTx((uint32_t*) txdata, txlength);
			//adcConvert();
			break;
			case 0xd0:
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Debug step 0");
			uartTx((uint32_t*) txdata, txlength);
			setPWM(&motor1, STEP_0, DEBUG_DUTY);
			setPWM(&motor2, STEP_0, DEBUG_DUTY);
			//adcConvert();
			break;
		case 0xd1:
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Debug step 1");
			uartTx((uint32_t*) txdata, txlength);
			setPWM(&motor1, STEP_1, DEBUG_DUTY);
			setPWM(&motor2, STEP_1, DEBUG_DUTY);
			//adcConvert();
			break;
		case 0xd2:
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Debug step 2");
			uartTx((uint32_t*) txdata, txlength);
			setPWM(&motor1, STEP_2, DEBUG_DUTY);
			setPWM(&motor2, STEP_2, DEBUG_DUTY);
			//adcConvert();
			break;
		case 0xd3:
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Debug step 3");
			uartTx((uint32_t*) txdata, txlength);
			setPWM(&motor1, STEP_3, DEBUG_DUTY);
			setPWM(&motor2, STEP_3, DEBUG_DUTY);
			//adcConvert();
			break;
		case 0xd4:
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Debug step 4");
			uartTx((uint32_t*) txdata, txlength);
			setPWM(&motor1, STEP_4, DEBUG_DUTY);
			setPWM(&motor2, STEP_4, DEBUG_DUTY);
			//adcConvert();
			break;
		case 0xd5:
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Debug step 5");
			uartTx((uint32_t*) txdata, txlength);
			setPWM(&motor1, STEP_5, DEBUG_DUTY);
			setPWM(&motor2, STEP_5, DEBUG_DUTY);
			//adcConvert();
			break;
		default:
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Invalid command!");
			uartTx((uint32_t*) txdata, txlength);
			break;
		}
		break;
	case STATE_CTRL_RUN:
		if (rxdata == 0xff)
		{
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Entering debug mode...");
			uartTx((uint32_t*) txdata, txlength);
			ctrlState = STATE_CTRL_DEBUG;
			changeState(&motor1, STATE_MOTOR_DEBUG_Z);
			changeState(&motor2, STATE_MOTOR_DEBUG_Z);
		}
		else if(rxdata <= 126)
		{
			if(sysEn == 1)
			{
				if(rxdata == 0)
				{
					motor1.cmdPrd = 0xffffffff;
					motor1.cmdFreq = 0;
					txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Motor 1: 0 RPM\n\r");
					uartTx((uint32_t*) txdata, txlength);
				}
				else
				{
					// Val is between 1 to 126
					// Set motor 1 cmd prd
					motor1.cmdPrdArrIdx = (PRD_ARR_OPEN_ACC_NUMEL - 1) + (PRD_ARR_CLOSED_REF_NUMEL*rxdata)/127;
					motor1.cmdPrd = (uint32_t) prdArr[motor1.cmdPrdArrIdx];
					motor1.cmdFreq = (F_SAMP/6)/motor1.cmdPrd;
					txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Motor 1: %d RPM\n\r", (int) (30*motor1.cmdFreq));
					uartTx((uint32_t*) txdata, txlength);
				}
			}
			else
			{
				txlength = snprintf((char*) txdata, TX_DATA_SIZE, "UVLO!\n\r");
				uartTx((uint32_t*) txdata, txlength);
			}
		}
		else if(rxdata > 127)
		{
			if(sysEn == 1)
			{
				if(rxdata == 128)
				{
					motor2.cmdPrd = 0xffffffff;
					motor2.cmdFreq = 0;
					txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Motor 2: 0 RPM\n\r");
					uartTx((uint32_t*) txdata, txlength);
				}
				else
				{
					// Val is between 129 to 254
					// Set motor 2 cmd prd
					motor2.cmdPrdArrIdx = (PRD_ARR_OPEN_ACC_NUMEL - 1) + (PRD_ARR_CLOSED_REF_NUMEL*(rxdata-128))/127;
					motor2.cmdPrd = (uint32_t) prdArr[motor2.cmdPrdArrIdx];
					motor2.cmdFreq = (F_SAMP/6)/motor2.cmdPrd;
					txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Motor 2: %d RPM\n\r", (int) (30*motor2.cmdFreq));
					uartTx((uint32_t*) txdata, txlength);
				}
			}
			else
			{
				txlength = snprintf((char*) txdata, TX_DATA_SIZE, "UVLO!\n\r");
				uartTx((uint32_t*) txdata, txlength);
			}
		}
		else
		{
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Invalid cmd!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;
	}
}


void setPWM(Motor_TypeDef* motor, uint8_t newStep, uint32_t compare)
{
	if(motor->step != newStep)
	{
		switch(newStep)
		{
		case STEP_Z:
			motor->phase1EnableCompare = 0;
			motor->phase1PWMCompare = 0;

			motor->phase2EnableCompare = 0;
			motor->phase2PWMCompare = 0;

			motor->phase3EnableCompare = 0;
			motor->phase3PWMCompare = 0;
			break;

		case STEP_0: // PNZ
			motor->phase1EnableCompare = 0xffff;
			motor->phase1PWMCompare = compare;

			motor->phase2EnableCompare = 0xffff;
			motor->phase2PWMCompare = 0;

			motor->phase3EnableCompare = 0;
			motor->phase3PWMCompare = 0;

			motor->adcActiveChannel = motor->phase3adcChannel;
			//LL_ADC_REG_SetSequencerRanks(ADC1, motor->zPhaseAdcRank, motor->phase3adcChannel);
			break;

		case STEP_1: // PZN
			motor->phase1EnableCompare = 0xffff;
			motor->phase1PWMCompare = compare;

			motor->phase2EnableCompare = 0;
			motor->phase2PWMCompare = 0;

			motor->phase3EnableCompare = 0xffff;
			motor->phase3PWMCompare = 0;

			motor->adcActiveChannel = motor->phase2adcChannel;
			//LL_ADC_REG_SetSequencerRanks(ADC1, motor->zPhaseAdcRank, motor->phase2adcChannel);
			break;

		case STEP_2: // ZPN
			motor->phase1EnableCompare = 0;
			motor->phase1PWMCompare = 0;

			motor->phase2EnableCompare = 0xffff;
			motor->phase2PWMCompare = compare;

			motor->phase3EnableCompare = 0xffff;
			motor->phase3PWMCompare = 0;

			motor->adcActiveChannel = motor->phase1adcChannel;
			//LL_ADC_REG_SetSequencerRanks(ADC1, motor->zPhaseAdcRank, motor->phase1adcChannel);
			break;

		case STEP_3: // NPZ
			motor->phase1EnableCompare = 0xffff;
			motor->phase1PWMCompare = 0;

			motor->phase2EnableCompare = 0xffff;
			motor->phase2PWMCompare = compare;

			motor->phase3EnableCompare = 0;
			motor->phase3PWMCompare = 0;

			motor->adcActiveChannel = motor->phase3adcChannel;
			//LL_ADC_REG_SetSequencerRanks(ADC1, motor->zPhaseAdcRank, motor->phase3adcChannel);
			break;

		case STEP_4: // NZP
			motor->phase1EnableCompare = 0xffff;
			motor->phase1PWMCompare = 0;

			motor->phase2EnableCompare = 0;
			motor->phase2PWMCompare = 0;

			motor->phase3EnableCompare = 0xffff;
			motor->phase3PWMCompare = compare;

			motor->adcActiveChannel = motor->phase2adcChannel;
			//LL_ADC_REG_SetSequencerRanks(ADC1, motor->zPhaseAdcRank, motor->phase2adcChannel);
			break;

		case STEP_5: // ZNP
			motor->phase1EnableCompare = 0;
			motor->phase1PWMCompare = 0;

			motor->phase2EnableCompare = 0xffff;
			motor->phase2PWMCompare = 0;

			motor->phase3EnableCompare = 0xffff;
			motor->phase3PWMCompare = compare;

			motor->adcActiveChannel = motor->phase1adcChannel;
			//LL_ADC_REG_SetSequencerRanks(ADC1, motor->zPhaseAdcRank, motor->phase1adcChannel);
			break;

		default:
			break;
		}

		if(motor->phaseId == PHASE_ID_ABC)
		{
			LL_TIM_OC_SetCompareCH1(TIM1, motor->phase1PWMCompare);
			LL_TIM_OC_SetCompareCH2(TIM1, motor->phase2PWMCompare);
			LL_TIM_OC_SetCompareCH3(TIM1, motor->phase3PWMCompare);

			LL_TIM_OC_SetCompareCH2(TIM2, motor->phase1EnableCompare);
			LL_TIM_OC_SetCompareCH3(TIM2, motor->phase2EnableCompare);
			LL_TIM_OC_SetCompareCH4(TIM2, motor->phase3EnableCompare);
		}
		else if(motor->phaseId == PHASE_ID_UVW)
		{
			LL_TIM_OC_SetCompareCH1(TIM3, motor->phase1PWMCompare);
			LL_TIM_OC_SetCompareCH2(TIM3, motor->phase2PWMCompare);
			LL_TIM_OC_SetCompareCH3(TIM3, motor->phase3PWMCompare);

			LL_TIM_OC_SetCompareCH1(TIM4, motor->phase1EnableCompare);
			LL_TIM_OC_SetCompareCH2(TIM4, motor->phase2EnableCompare);
			LL_TIM_OC_SetCompareCH3(TIM4, motor->phase3EnableCompare);
		}

		motor->step = newStep;
	}

}

void adcConvCpltCallback(void)
{
	if(sysEn == 1)
	{
		if(adcdata[ADC_DCBUS] < 2700)
		{
			uvloCtr++;
		}
		else
		{
			uvloCtr = 0;
		}
		if(uvloCtr > UVLO_TIMEOUT)
		{
			changeState(&motor1, STATE_MOTOR_STOP);
			changeState(&motor2, STATE_MOTOR_STOP);
			motor1.cmdPrd = 0xffffffff;
			motor1.cmdFreq = 0;
			motor2.cmdPrd = 0xffffffff;
			motor2.cmdFreq = 0;
			sysEn = 0;
			uvloCtr = 0;
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "DC bus UVLO!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
	}
	else if(sysEn == 0)
	{
		if(adcdata[ADC_DCBUS] >= 2700)
		{
			uvloCtr++;
		}
		else
		{
			uvloCtr = 0;
		}
		if(uvloCtr > UVLO_TIMEOUT)
		{
			sysEn = 1;
			uvloCtr = 0;
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Supply voltage validated!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
	}

	if(sysEn == 1)
	{
		stateEval(&motor1);
		stateEval(&motor2);
	}
}

void pwmPrdCpltCallback(void)
{
	stateEval(&motor1);
	stateEval(&motor2);
}

void changeState(Motor_TypeDef* motor, uint8_t newState)
{
	if(motor->state != newState)
	{
		switch(newState)
		{
		case STATE_MOTOR_DEBUG_Z:
			motor->prd = OPEN_INIT_PRD;
			motor->openPrdCtr = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_Z, DEBUG_DUTY);
			break;

		case STATE_MOTOR_DEBUG_0:
			motor->ctr = 0;
			setPWM(motor, STEP_0, DEBUG_DUTY);
			break;

		case STATE_MOTOR_DEBUG_1:
			motor->ctr = 0;
			setPWM(motor, STEP_1, DEBUG_DUTY);
			break;

		case STATE_MOTOR_DEBUG_2:
			motor->ctr = 0;
			setPWM(motor, STEP_2, DEBUG_DUTY);
			break;

		case STATE_MOTOR_DEBUG_3:
			motor->ctr = 0;
			setPWM(motor, STEP_3, DEBUG_DUTY);
			break;

		case STATE_MOTOR_DEBUG_4:
			motor->ctr = 0;
			setPWM(motor, STEP_4, DEBUG_DUTY);
			break;

		case STATE_MOTOR_DEBUG_5:
			motor->ctr = 0;
			setPWM(motor, STEP_5, DEBUG_DUTY);
			break;

		case STATE_MOTOR_STOP:
			setPWM(motor, STEP_Z, 0);
			motor->handoffCtr = 0;
			motor->isMeasuredFreqValid = 0;
			motor->openPrdArrIdx = 0;
			motor->openPrdCtr = 0;
			motor->ctr = 0;
			motor->duty = 0;
			motor->cmdPrd = 0xffffffff;
			motor->cmdFreq = 0;
			motor->prd = OPEN_INIT_PRD;
			break;

		case STATE_MOTOR_OPEN_ACC_0:
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			motor->duty = OPEN_DUTY_COEFF/motor->prd+OPEN_DUTY_OFFSET;
			setPWM(motor, STEP_0, motor->duty);
			break;

		case STATE_MOTOR_OPEN_ACC_1:
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			motor->duty = OPEN_DUTY_COEFF/motor->prd+OPEN_DUTY_OFFSET;
			setPWM(motor, STEP_1, motor->duty);
			break;

		case STATE_MOTOR_OPEN_ACC_2:
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			motor->duty = OPEN_DUTY_COEFF/motor->prd+OPEN_DUTY_OFFSET;
			setPWM(motor, STEP_2, motor->duty);
			break;

		case STATE_MOTOR_OPEN_ACC_3:
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			motor->duty = OPEN_DUTY_COEFF/motor->prd+OPEN_DUTY_OFFSET;
			setPWM(motor, STEP_3, motor->duty);
			break;

		case STATE_MOTOR_OPEN_ACC_4:
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			motor->duty = OPEN_DUTY_COEFF/motor->prd+OPEN_DUTY_OFFSET;
			setPWM(motor, STEP_4, motor->duty);
			break;

		case STATE_MOTOR_OPEN_ACC_5:
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			motor->duty = OPEN_DUTY_COEFF/motor->prd+OPEN_DUTY_OFFSET;
			setPWM(motor, STEP_5, motor->duty);
			break;

		case STATE_MOTOR_HANDOFF_0:
			motor->openPrdCtr = 0;
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_0, motor->duty);
			break;

		case STATE_MOTOR_HANDOFF_1:
			motor->openPrdCtr = 0;
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_1, motor->duty);
			break;

		case STATE_MOTOR_HANDOFF_2:
			motor->openPrdCtr = 0;
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_2, motor->duty);
			break;

		case STATE_MOTOR_HANDOFF_3:
			motor->openPrdCtr = 0;
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_3, motor->duty);
			break;

		case STATE_MOTOR_HANDOFF_4:
			motor->openPrdCtr = 0;
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_4, motor->duty);
			break;

		case STATE_MOTOR_HANDOFF_5:
			motor->openPrdCtr = 0;
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_5, motor->duty);
			break;

		case STATE_MOTOR_CLOSED_GET_SPD_WATCH_0:
			motor->openPrdCtr = 0;
			motor->closedWatchTimeout = motor->prd;
			motor->piSum = PI_INT_CONST*F_SAMP*motor->duty;
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			motor->getSpdCtr = 0;
			motor->closedWatchTimeout = motor->prd;
			motor->duty = OPEN_DUTY_COEFF/motor->prd + OPEN_DUTY_OFFSET;
			setPWM(motor, STEP_0, motor->duty);
			break;

		case STATE_MOTOR_CLOSED_GET_SPD_WAIT_0:
			motor->isPrevValValid = 0;
			break;

		case STATE_MOTOR_CLOSED_GET_SPD_WATCH_1:
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_1, motor->duty);
			break;

		case STATE_MOTOR_CLOSED_GET_SPD_WAIT_1:
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			motor->prd = motor->getSpdCtr/2;
			motor->closedWatchTimeout = motor->getSpdCtr;
			motor->getSpdCtr = 0;
			break;

		case STATE_MOTOR_CLOSED_WATCH_0:
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_0, motor->duty);
			break;

		case STATE_MOTOR_CLOSED_INT_0:
			motor->fluxIntSum = 0;
			motor->isPrevValValid = 0;
			motor->prd = motor->getSpdCtr/2;
			motor->measuredFreq = (F_SAMP/6)/motor->getSpdCtr;
			motor->closedWatchTimeout = motor->getSpdCtr;
			motor->getSpdCtr = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_0, motor->duty);
			break;

		case STATE_MOTOR_CLOSED_WATCH_1:
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_1, motor->duty);
			break;

		case STATE_MOTOR_CLOSED_INT_1:
			motor->fluxIntSum = 0;
			motor->isPrevValValid = 0;
			motor->prd = motor->getSpdCtr/2;
			motor->measuredFreq = (F_SAMP/6)/motor->getSpdCtr;
			motor->closedWatchTimeout = motor->getSpdCtr;
			motor->getSpdCtr = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_1, motor->duty);
			break;

		case STATE_MOTOR_CLOSED_WATCH_2:
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_2, motor->duty);
			break;

		case STATE_MOTOR_CLOSED_INT_2:
			motor->fluxIntSum = 0;
			motor->isPrevValValid = 0;
			motor->prd = motor->getSpdCtr/2;
			motor->measuredFreq = (F_SAMP/6)/motor->getSpdCtr;
			motor->closedWatchTimeout = motor->getSpdCtr;
			motor->getSpdCtr = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_2, motor->duty);
			break;

		case STATE_MOTOR_CLOSED_WATCH_3:
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_3, motor->duty);
			break;

		case STATE_MOTOR_CLOSED_INT_3:
			motor->fluxIntSum = 0;
			motor->isPrevValValid = 0;
			motor->prd = motor->getSpdCtr/2;
			motor->measuredFreq = (F_SAMP/6)/motor->getSpdCtr;
			motor->closedWatchTimeout = motor->getSpdCtr;
			motor->getSpdCtr = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_3, motor->duty);
			break;

		case STATE_MOTOR_CLOSED_WATCH_4:
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_4, motor->duty);
			break;

		case STATE_MOTOR_CLOSED_INT_4:
			motor->fluxIntSum = 0;
			motor->isPrevValValid = 0;
			motor->prd = motor->getSpdCtr/2;
			motor->measuredFreq = (F_SAMP/6)/motor->getSpdCtr;
			motor->closedWatchTimeout = motor->getSpdCtr;
			motor->getSpdCtr = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_4, motor->duty);
			break;

		case STATE_MOTOR_CLOSED_WATCH_5:
			motor->isPrevValValid = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_5, motor->duty);
			break;

		case STATE_MOTOR_CLOSED_INT_5:
			motor->fluxIntSum = 0;
			motor->isPrevValValid = 0;
			motor->prd = motor->getSpdCtr/2;
			motor->measuredFreq = (F_SAMP/6)/motor->getSpdCtr;
			motor->closedWatchTimeout = motor->getSpdCtr;
			motor->getSpdCtr = 0;
			motor->ctr = 0;
			setPWM(motor, STEP_5, motor->duty);
			break;

		default:
			break;
		}
		motor->state = newState;
	}
}

void stateEval(Motor_TypeDef* motor)
{
	// If command speed is set to zero (command freq is set to 0),
	// set the motor to stop state.
	if((motor->cmdFreq == 0) && (motor->state != STATE_MOTOR_STOP))
	{
		changeState(motor, STATE_MOTOR_STOP);
	}

	// State machine switch statement
	switch(motor->state)
	{

	// @stateName: STATE_MOTOR_STOP
	// @stateDescription: Motor is stopped in this state. All associated output phases are high Z.
	// @stateInitConditions:
	// 1. PWM is set to STEP_Z
	// 2. openPrdArrIdx = 0
	// 3. openPrdCtr = 0
	// 4. ctr = 0
	// 5. duty = 0
	// 6. cmdPrd = 0xffffffff
	// 7. cmdFreq = 0
	// 8. prd = OPEN_INIT_PRD
	case STATE_MOTOR_STOP:
		if(motor->cmdFreq != 0)
		{
			motor->openPrdCtr = 0;
			motor->prd = prdArr[motor->openPrdCtr];
			changeState(motor, STATE_MOTOR_OPEN_ACC_0);
		}
		break;

	// @stateName: STATE_MOTOR_OPEN_ACC_0
	// @stateDescription: Open loop acceleration step 0
	// @stateInitConditions:
	// 1. isPrevValValid = 0
	// 2. ctr = 0
	// 3. duty = OPEN_DUTY_COEFF/motor->prd+OPEN_DUTY_OFFSET
	// 4. setPWM(motor, STEP_0, motor->duty)
	case STATE_MOTOR_OPEN_ACC_0:
		if(motor->openPrdCtr < PRD_ARR_OPEN_ACC_NUMEL)
		{
			if(motor->ctr < motor->prd)
			{
				motor->ctr++;
			}
			else
			{
				motor->prd = (uint32_t) prdArr[motor->openPrdCtr];
				motor->openPrdCtr++;
				if(motor->dir == MOTOR_DIRECTION_FWD)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_1);
				}
				else if (motor->dir == MOTOR_DIRECTION_REV)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_5);
				}
			}
		}
		else
		{
			// ENTER CLOSED LOOP
			if(motor->phaseId == PHASE_ID_ABC){ LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0); }
			// changeState to STATE_MOTOR_CLOSED_GET_SPD_WATCH_0 sets the following values:
			// motor->isPrevValValid = 0;
			// motor->ctr = 0;
			// motor->getSpdCtr = 0;
			// motor->closedWatchTimeout = motor->prd;
			// motor->duty = OPEN_DUTY_COEFF/motor->prd + OPEN_DUTY_OFFSET;
			// setPWM(motor, STEP_0, motor->duty);
			//changeState(motor, STATE_MOTOR_CLOSED_GET_SPD_WATCH_0);
			motor->piSum = (F_SAMP*motor->duty)/PI_INT_CONST;
			changeState(motor, STATE_MOTOR_HANDOFF_0);
		}
		break;

	// @stateName: STATE_MOTOR_OPEN_ACC_1
	// @stateDescription: Open loop acceleration step 1
	// @stateInitConditions:
	// 1. isPrevValValid = 0
	// 2. ctr = 0
	// 3. duty = OPEN_DUTY_COEFF/motor->prd+OPEN_DUTY_OFFSET
	// 4. setPWM(motor, STEP_1, motor->duty)
	case STATE_MOTOR_OPEN_ACC_1:
		if(motor->openPrdCtr < PRD_ARR_OPEN_ACC_NUMEL)
		{
			if(motor->ctr < motor->prd)
			{
				motor->ctr++;
			}
			else
			{
				motor->prd = (uint32_t) prdArr[motor->openPrdCtr];
				motor->openPrdCtr++;
				if(motor->dir == MOTOR_DIRECTION_FWD)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_2);
				}
				else if (motor->dir == MOTOR_DIRECTION_REV)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_0);
				}
			}
		}
		else
		{
			// RUN OPEN AT CONSTANT SPEED
			if(motor->ctr < motor->prd)
			{
				motor->ctr++;
			}
			else
			{
				if(motor->dir == MOTOR_DIRECTION_FWD)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_2);
				}
				else if (motor->dir == MOTOR_DIRECTION_REV)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_0);
				}
			}
		}
		break;

	// @stateName: STATE_MOTOR_OPEN_ACC_2
	// @stateDescription: Open loop acceleration step 2
	// @stateInitConditions:
	// 1. isPrevValValid = 0
	// 2. ctr = 0
	// 3. duty = OPEN_DUTY_COEFF/motor->prd+OPEN_DUTY_OFFSET
	// 4. setPWM(motor, STEP_2, motor->duty)
	case STATE_MOTOR_OPEN_ACC_2:
		if(motor->openPrdCtr < PRD_ARR_OPEN_ACC_NUMEL)
		{
			if(motor->ctr < motor->prd)
			{
				motor->ctr++;
			}
			else
			{
				motor->prd = (uint32_t) prdArr[motor->openPrdCtr];
				motor->openPrdCtr++;
				if(motor->dir == MOTOR_DIRECTION_FWD)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_3);
				}
				else if (motor->dir == MOTOR_DIRECTION_REV)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_1);
				}
			}
		}
		else
		{
			// RUN OPEN AT CONSTANT SPEED
			if(motor->ctr < motor->prd)
			{
				motor->ctr++;
			}
			else
			{
				if(motor->dir == MOTOR_DIRECTION_FWD)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_3);
				}
				else if (motor->dir == MOTOR_DIRECTION_REV)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_1);
				}
			}
		}
		break;

	// @stateName: STATE_MOTOR_OPEN_ACC_3
	// @stateDescription: Open loop acceleration step 3
	// @stateInitConditions:
	// 1. isPrevValValid = 0
	// 2. ctr = 0
	// 3. duty = OPEN_DUTY_COEFF/motor->prd+OPEN_DUTY_OFFSET
	// 4. setPWM(motor, STEP_3, motor->duty)
	case STATE_MOTOR_OPEN_ACC_3:
		if(motor->openPrdCtr < PRD_ARR_OPEN_ACC_NUMEL)
		{
			if(motor->ctr < motor->prd)
			{
				motor->ctr++;
			}
			else
			{
				motor->prd = (uint32_t) prdArr[motor->openPrdCtr];
				motor->openPrdCtr++;
				if(motor->dir == MOTOR_DIRECTION_FWD)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_4);
				}
				else if (motor->dir == MOTOR_DIRECTION_REV)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_2);
				}
			}
		}
		else
		{
			// RUN OPEN AT CONSTANT SPEED
			if(motor->ctr < motor->prd)
			{
				motor->ctr++;
			}
			else
			{
				if(motor->dir == MOTOR_DIRECTION_FWD)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_4);
				}
				else if (motor->dir == MOTOR_DIRECTION_REV)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_2);
				}
			}
		}
		break;

	// @stateName: STATE_MOTOR_OPEN_ACC_4
	// @stateDescription: Open loop acceleration step 4
	// @stateInitConditions:
	// 1. isPrevValValid = 0
	// 2. ctr = 0
	// 3. duty = OPEN_DUTY_COEFF/motor->prd+OPEN_DUTY_OFFSET
	// 4. setPWM(motor, STEP_4, motor->duty)
	case STATE_MOTOR_OPEN_ACC_4:
		if(motor->openPrdCtr < PRD_ARR_OPEN_ACC_NUMEL)
		{
			if(motor->ctr < motor->prd)
			{
				motor->ctr++;
			}
			else
			{
				motor->prd = (uint32_t) prdArr[motor->openPrdCtr];
				motor->openPrdCtr++;
				if(motor->dir == MOTOR_DIRECTION_FWD)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_5);
				}
				else if (motor->dir == MOTOR_DIRECTION_REV)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_3);
				}
			}
		}
		else
		{
			// RUN OPEN AT CONSTANT SPEED
			if(motor->ctr < motor->prd)
			{
				motor->ctr++;
			}
			else
			{
				if(motor->dir == MOTOR_DIRECTION_FWD)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_5);
				}
				else if (motor->dir == MOTOR_DIRECTION_REV)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_3);
				}
			}
		}
		break;

	// @stateName: STATE_MOTOR_OPEN_ACC_5
	// @stateDescription: Open loop acceleration step 5
	// @stateInitConditions:
	// 1. isPrevValValid = 0
	// 2. ctr = 0
	// 3. duty = OPEN_DUTY_COEFF/motor->prd+OPEN_DUTY_OFFSET
	// 4. setPWM(motor, STEP_5, motor->duty)
	case STATE_MOTOR_OPEN_ACC_5:
		if(motor->openPrdCtr < PRD_ARR_OPEN_ACC_NUMEL)
		{
			if(motor->ctr < motor->prd)
			{
				motor->ctr++;
			}
			else
			{
				motor->prd = (uint32_t) prdArr[motor->openPrdCtr];
				motor->openPrdCtr++;
				if(motor->dir == MOTOR_DIRECTION_FWD)
				{

					changeState(motor, STATE_MOTOR_OPEN_ACC_0);
				}
				else if (motor->dir == MOTOR_DIRECTION_REV)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_4);
				}
			}
		}
		else
		{
			// RUN OPEN AT CONSTANT SPEED
			if(motor->ctr < motor->prd)
			{
				motor->ctr++;
			}
			else
			{
				if(motor->dir == MOTOR_DIRECTION_FWD)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_0);
				}
				else if (motor->dir == MOTOR_DIRECTION_REV)
				{
					changeState(motor, STATE_MOTOR_OPEN_ACC_4);
				}
			}
		}
		break;

	// @stateName: STATE_MOTOR_HANDOFF_0
	// @stateInitConditions:
	// 1. motor->openPrdCtr = 0;
	// 2. motor->isPrevValValid = 0;
	// 3. motor->ctr = 0;
	// 4. setPWM(motor, STEP_0, motor->duty);
	case STATE_MOTOR_HANDOFF_0:
		if(motor->handoffCtr < HANDOFF_TIMEOUT)
		{
			if(motor->ctr < CLOSED_TIMEOUT)
			{
				motor->ctr++;
				if(motor->ctr > BLANK_TIME)
				{
					if( (adcdata[motor->adcDataZPhaseIdx] <= (adcdata[motor->adcDataDcBusIdx]/2))&&
						(motor->zPhasePrevVal > (motor->dcBusPrevVal/2)) )
					{
						if(motor->phaseId == PHASE_ID_ABC){ LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0); }
						changeState(motor, STATE_MOTOR_CLOSED_INT_0);
					}
					/*else if( (adcdata[motor->adcDataZPhaseIdx] >= (adcdata[motor->adcDataDcBusIdx]/2))&&
							(motor->zPhasePrevVal < (motor->dcBusPrevVal/2)) )
					{
						if(motor->phaseId == PHASE_ID_ABC){ LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0); }
						changeState(motor, STATE_MOTOR_CLOSED_INT_3);
					}*/
				}
				else
				{
					motor->isPrevValValid = 1;
				}
			}
			else
			{
				//motor->handoffCtr++;
				//changeState(motor, STATE_MOTOR_HANDOFF_1);
				changeState(motor, STATE_MOTOR_STOP);
				txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Handoff stopped!\n\r");
				uartTx((uint32_t*) txdata, txlength);
			}
		}
		else
		{
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Handoff stopped!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	// @stateName: STATE_MOTOR_HANDOFF_1
	// @stateInitConditions:
	// 1. motor->openPrdCtr = 0;
	// 2. motor->isPrevValValid = 0;
	// 3. motor->ctr = 0;
	// 4. setPWM(motor, STEP_1, motor->duty);
	case STATE_MOTOR_HANDOFF_1:
		if(motor->handoffCtr < HANDOFF_TIMEOUT)
		{
			if(motor->ctr < motor->prd)
			{
				motor->ctr++;
				if(motor->ctr > BLANK_TIME)
				{
					if( (adcdata[motor->adcDataZPhaseIdx] >= (adcdata[motor->adcDataDcBusIdx]/2))&&
						(motor->zPhasePrevVal < (motor->dcBusPrevVal/2)) )
					{
						if(motor->phaseId == PHASE_ID_ABC){ LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0); }
						changeState(motor, STATE_MOTOR_CLOSED_INT_1);
					}
					else if( (adcdata[motor->adcDataZPhaseIdx] <= (adcdata[motor->adcDataDcBusIdx]/2))&&
							(motor->zPhasePrevVal > (motor->dcBusPrevVal/2)) )
					{
						if(motor->phaseId == PHASE_ID_ABC){ LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0); }
						changeState(motor, STATE_MOTOR_CLOSED_INT_4);
					}
				}
				else
				{
					motor->isPrevValValid = 1;
				}
			}
			else
			{
				motor->handoffCtr++;
				changeState(motor, STATE_MOTOR_HANDOFF_2);
			}
		}
		else
		{
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Handoff stopped!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	// @stateName: STATE_MOTOR_HANDOFF_2
	// @stateInitConditions:
	// 1. motor->openPrdCtr = 0;
	// 2. motor->isPrevValValid = 0;
	// 3. motor->ctr = 0;
	// 4. setPWM(motor, STEP_2, motor->duty);
	case STATE_MOTOR_HANDOFF_2:
		if(motor->handoffCtr < HANDOFF_TIMEOUT)
		{
			if(motor->ctr < motor->prd)
			{
				motor->ctr++;
				if(motor->ctr > BLANK_TIME)
				{
					if( (adcdata[motor->adcDataZPhaseIdx] <= (adcdata[motor->adcDataDcBusIdx]/2))&&
						(motor->zPhasePrevVal > (motor->dcBusPrevVal/2)) )
					{
						if(motor->phaseId == PHASE_ID_ABC){ LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0); }
						changeState(motor, STATE_MOTOR_CLOSED_INT_2);
					}
					else if( (adcdata[motor->adcDataZPhaseIdx] >= (adcdata[motor->adcDataDcBusIdx]/2))&&
							(motor->zPhasePrevVal < (motor->dcBusPrevVal/2)) )
					{
						if(motor->phaseId == PHASE_ID_ABC){ LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0); }
						changeState(motor, STATE_MOTOR_CLOSED_INT_5);
					}
				}
				else
				{
					motor->isPrevValValid = 1;
				}
			}
			else
			{
				motor->handoffCtr++;
				changeState(motor, STATE_MOTOR_HANDOFF_3);
			}
		}
		else
		{
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Handoff stopped!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	// @stateName: STATE_MOTOR_HANDOFF_3
	// @stateInitConditions:
	// 1. motor->openPrdCtr = 0;
	// 2. motor->isPrevValValid = 0;
	// 3. motor->ctr = 0;
	// 4. setPWM(motor, STEP_3, motor->duty);
	case STATE_MOTOR_HANDOFF_3:
		if(motor->handoffCtr < HANDOFF_TIMEOUT)
		{
			if(motor->ctr < motor->prd)
			{
				motor->ctr++;
				if(motor->ctr > BLANK_TIME)
				{
					if( (adcdata[motor->adcDataZPhaseIdx] >= (adcdata[motor->adcDataDcBusIdx]/2))&&
						(motor->zPhasePrevVal < (motor->dcBusPrevVal/2)) )
					{
						if(motor->phaseId == PHASE_ID_ABC){ LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0); }
						changeState(motor, STATE_MOTOR_CLOSED_INT_3);
					}
					else if( (adcdata[motor->adcDataZPhaseIdx] <= (adcdata[motor->adcDataDcBusIdx]/2))&&
							(motor->zPhasePrevVal > (motor->dcBusPrevVal/2)) )
					{
						if(motor->phaseId == PHASE_ID_ABC){ LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0); }
						changeState(motor, STATE_MOTOR_CLOSED_INT_0);
					}
				}
				else
				{
					motor->isPrevValValid = 1;
				}
			}
			else
			{
				motor->handoffCtr++;
				changeState(motor, STATE_MOTOR_HANDOFF_4);
			}
		}
		else
		{
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Handoff stopped!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	// @stateName: STATE_MOTOR_HANDOFF_4
	// @stateInitConditions:
	// 1. motor->openPrdCtr = 0;
	// 2. motor->isPrevValValid = 0;
	// 3. motor->ctr = 0;
	// 4. setPWM(motor, STEP_4, motor->duty);
	case STATE_MOTOR_HANDOFF_4:
		if(motor->handoffCtr < HANDOFF_TIMEOUT)
		{
			if(motor->ctr < motor->prd)
			{
				motor->ctr++;
				if(motor->ctr > BLANK_TIME)
				{
					if( (adcdata[motor->adcDataZPhaseIdx] <= (adcdata[motor->adcDataDcBusIdx]/2))&&
						(motor->zPhasePrevVal > (motor->dcBusPrevVal/2)) )
					{
						if(motor->phaseId == PHASE_ID_ABC){ LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0); }
						changeState(motor, STATE_MOTOR_CLOSED_INT_4);
					}
					else if( (adcdata[motor->adcDataZPhaseIdx] >= (adcdata[motor->adcDataDcBusIdx]/2))&&
							(motor->zPhasePrevVal < (motor->dcBusPrevVal/2)) )
					{
						if(motor->phaseId == PHASE_ID_ABC){ LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0); }
						changeState(motor, STATE_MOTOR_CLOSED_INT_1);
					}
				}
				else
				{
					motor->isPrevValValid = 1;
				}
			}
			else
			{
				motor->handoffCtr++;
				changeState(motor, STATE_MOTOR_HANDOFF_5);
			}
		}
		else
		{
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Handoff stopped!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	// @stateName: STATE_MOTOR_HANDOFF_5
	// @stateInitConditions:
	// 1. motor->openPrdCtr = 0;
	// 2. motor->isPrevValValid = 0;
	// 3. motor->ctr = 0;
	// 4. setPWM(motor, STEP_5, motor->duty);
	case STATE_MOTOR_HANDOFF_5:
		if(motor->handoffCtr < HANDOFF_TIMEOUT)
		{
			if(motor->ctr < motor->prd)
			{
				motor->ctr++;
				if(motor->ctr > BLANK_TIME)
				{
					if( (adcdata[motor->adcDataZPhaseIdx] >= (adcdata[motor->adcDataDcBusIdx]/2))&&
						(motor->zPhasePrevVal < (motor->dcBusPrevVal/2)) )
					{
						if(motor->phaseId == PHASE_ID_ABC){ LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0); }
						changeState(motor, STATE_MOTOR_CLOSED_INT_5);
					}
					else if( (adcdata[motor->adcDataZPhaseIdx] <= (adcdata[motor->adcDataDcBusIdx]/2))&&
							(motor->zPhasePrevVal > (motor->dcBusPrevVal/2)) )
					{
						if(motor->phaseId == PHASE_ID_ABC){ LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0); }
						changeState(motor, STATE_MOTOR_CLOSED_INT_2);
					}
				}
				else
				{
					motor->isPrevValValid = 1;
				}
			}
			else
			{
				motor->handoffCtr++;
				changeState(motor, STATE_MOTOR_HANDOFF_0);
			}
		}
		else
		{
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "Handoff stopped!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	// @stateName: STATE_MOTOR_GET_SPD_WATCH_0
	// @stateDescription: This state is used during the handoff between open loop and closed loop.
	// This state looks for a zero cross to switch to STATE_MOTOR_CLOSED_GET_SPD_WAIT_0. If the state
	// times out before a zero cross is detected, the motor is set to the STOP state.
	// @stateInitConditions:
	// 1. isPrevValValid = 0
	// 2. ctr = 0
	// 3. getSpdCtr = 0
	// 4. closedWatchTimeout = motor->prd
	// 5. duty = OPEN_DUTY_COEFF/motor->prd + OPEN_DUTY_OFFSET
	// 6. setPWM(motor, STEP_0, motor->duty)
	// 7. openPrdCtr = 0
	// 8. piSum = PI_INT_CONST*F_SAMP*motor->duty
	case STATE_MOTOR_CLOSED_GET_SPD_WATCH_0:

		// If watch period has not timed out, increase counter and check for zero cross.
		if(motor->ctr < motor->closedWatchTimeout)
		{
			motor->ctr++;

			// Wait one sampling cycle for previous value to be valid (isPrevValValid is 0 upon entering the state).
			if(motor->isPrevValValid == 1)
			{
				// If back EMF has crossed above the threshold and the previous value was below, change state to Get Speed Wait 0.
				if((adcdata[motor->adcDataZPhaseIdx] <= (adcdata[motor->adcDataDcBusIdx]/2)) &&
				   (motor->zPhasePrevVal > (motor->dcBusPrevVal/2)) )
				{
					changeState(motor, STATE_MOTOR_CLOSED_GET_SPD_WAIT_0);
					if(motor->phaseId == PHASE_ID_ABC) { LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0); }
				}
			}
			else
			{
				motor->isPrevValValid = 1;
			}

		}
		// If watch period has timed out, stop the motor and send error message.
		else
		{
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "No ZC in GET SPD WATCH 0!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	// @stateName: STATE_MOTOR_CLOSED_GET_SPD_WAIT_0
	// @stateDescription: This state is entered when a zero cross has been detected in
	// STATE_MOTOR_CLOSED_GET_SPD_WATCH_0. ctr is NOT set to zero from WATCH. Once ctr
	// reaches prd (same from last open acc period), the state switches to
	// STATE_MOTOR_CLOSED_GET_SPD_WATCH_1 to wait for another zero cross to measure
	// motor speed.
	// @stateInitConditions:
	// 1. isPrevValValid = 0
	case STATE_MOTOR_CLOSED_GET_SPD_WAIT_0:
		if(motor->ctr < motor->prd)
		{
			motor->ctr++;
			motor->getSpdCtr++;
		}
		else
		{
			if(motor->phaseId == PHASE_ID_ABC) { LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0); }
			changeState(motor, STATE_MOTOR_CLOSED_GET_SPD_WATCH_1);
		}
		break;

	// @stateName: STATE_MOTOR_CLOSED_GET_SPD_WATCH_1
	// @stateDescription: This state watches for a second zero crossing to measure
	// actual motor speed. getSpdCtr is NOT reset. It is carried over from the previous state.
	// If a zero cross is detected, the state changes to TATE_MOTOR_CLOSED_WAIT_1.
	// If the state times out before a zero cross is detected, the motor is stopped.
	// @stateInitConditions:
	// 1. isPrevValValid = 0
	// 2. ctr = 0
	case STATE_MOTOR_CLOSED_GET_SPD_WATCH_1:
		if(motor->ctr < motor->closedWatchTimeout)
		{
			motor->ctr++;
			motor->getSpdCtr++;
			if(motor->ctr > 2)
			{
				if((adcdata[motor->adcDataZPhaseIdx] >= (124*adcdata[motor->adcDataDcBusIdx]/280)) &&
				   (motor->zPhasePrevVal < (124*motor->dcBusPrevVal/280)) )
				{
					changeState(motor, STATE_MOTOR_CLOSED_INT_4);
					if(motor->phaseId == PHASE_ID_ABC) { LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0); }
				}
			}
			else
			{
				motor->isPrevValValid = 1;
			}
		}
		else
		{
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "No ZC in GET SPD WATCH 1!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	// @stateName: STATE_MOTOR_CLOSED_WATCH_0
	// @stateInitConditions:
	// 1. isPrevValValid = 0;
	// 2. ctr = 0;
	// 3. closedSetDuty(motor);
	// 4. setPWM(motor, STEP_0, motor->duty);
	case STATE_MOTOR_CLOSED_WATCH_0:
		if(motor->ctr < CLOSED_TIMEOUT)
		{
			motor->ctr++;
			motor->getSpdCtr++;
			if(motor->isMeasuredFreqValid == 1)
			{
				closedSetDuty(motor);
				setPWM(motor, STEP_0, motor->duty);
			}
			// Make sure previous value is valid
			if(motor->ctr > BLANK_TIME)
			{
				// Zero cross detected?
				if( (adcdata[motor->adcDataZPhaseIdx] <= (124*adcdata[motor->adcDataDcBusIdx]/280)) &&
				    (motor->zPhasePrevVal > (124*motor->dcBusPrevVal/280)))
				{
					// Set isPrevValValid to 0, set prd to ctr,
					// set timeout to 2*prd, set ctr to 0, run PI calc,
					// and change state to closed wait 0.
					if(motor->phaseId == PHASE_ID_ABC) {LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0);}
					motor->isMeasuredFreqValid = 1;
					changeState(motor, STATE_MOTOR_CLOSED_INT_0);
				}
			}
			else
			{
				motor->isPrevValValid = 1;
			}
		}
		else
		{
			// If no zero cross detected, stop motor
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "No ZC in WATCH 0!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	// @stateName: STATE_MOTOR_CLOSED_INT_0
	// @stateInitConditions:
	// 1. isPrevValValid = 0
	// 2. prd = motor->getSpdCtr/2
	// 3. measuredFreq = (F_SAMP/6)/motor->getSpdCtr
	// 4. closedWatchTimeout = motor->getSpdCtr
	// 5. getSpdCtr = 0
	// 6. ctr = 0
	// 7. setPWM(motor, STEP_0, motor->duty)
	case STATE_MOTOR_CLOSED_INT_0:
		if(motor->ctr < CLOSED_TIMEOUT)
		{
			motor->ctr++;
			motor->getSpdCtr++;
			if(motor->isMeasuredFreqValid == 1)
			{
				closedSetDuty(motor);
				setPWM(motor, STEP_0, motor->duty);
			}
			motor->fluxIntSum += adcdata[motor->adcDataZPhaseIdx] - (124*adcdata[motor->adcDataDcBusIdx]/280);
			if(motor->fluxIntSum <= FLUX_LOW_THRESH)
			{
				changeState(motor, STATE_MOTOR_CLOSED_WATCH_1);
			}
		}
		else
		{
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "INT 0 timeout!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	// @stateName: STATE_MOTOR_CLOSED_WATCH_1
	// @stateInitConditions:
	//
	case STATE_MOTOR_CLOSED_WATCH_1:
		if(motor->ctr < CLOSED_TIMEOUT)
		{
			motor->ctr++;
			motor->getSpdCtr++;
			if(motor->isMeasuredFreqValid == 1)
			{
				closedSetDuty(motor);
				setPWM(motor, STEP_1, motor->duty);
			}
			// Make sure previous value is valid
			if(motor->ctr > BLANK_TIME)
			{
				// Zero cross detected?
				if( (adcdata[motor->adcDataZPhaseIdx] >= (124*adcdata[motor->adcDataDcBusIdx]/280)) &&
				    (motor->zPhasePrevVal < (124*motor->dcBusPrevVal/280)))
				{
					// Set isPrevValValid to 0, set prd to ctr,
					// set timeout to 2*prd, set ctr to 0, run PI calc,
					// and change state to closed wait 1.
					if(motor->phaseId == PHASE_ID_ABC) {LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0);}
					motor->isMeasuredFreqValid = 1;
					changeState(motor, STATE_MOTOR_CLOSED_INT_1);
				}
			}
			else
			{
				motor->isPrevValValid = 1;
			}
		}
		else
		{
			// If no zero cross detected, stop motor
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "No ZC in WATCH 1!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	// @stateName: STATE_MOTOR_CLOSED_INT_1
	// @stateDescription: Closed loop operation state. Zero cross has been detected in STATE_MOTOR_CLOSED_WATCH_1.
	// The state waits for half of getSpdCtr to switch to next watch state.
	// @stateInitConditions:
	// isPrevValValid = 0
	// prd = motor->getSpdCtr/2
	// measuredFreq = (F_SAMP/6)/motor->getSpdCtr
	// closedWatchTimeout = motor->getSpdCtr
	// getSpdCtr = 0
	// ctr = 0
	// closedSetDuty(motor)
	// setPWM(motor, STEP_1, motor->duty)
	case STATE_MOTOR_CLOSED_INT_1:
		if(motor->ctr < CLOSED_TIMEOUT)
		{
			motor->ctr++;
			motor->getSpdCtr++;
			if(motor->isMeasuredFreqValid == 1)
			{
				closedSetDuty(motor);
				setPWM(motor, STEP_1, motor->duty);
			}
			motor->fluxIntSum += adcdata[motor->adcDataZPhaseIdx] - (124*adcdata[motor->adcDataDcBusIdx]/280);
			if(motor->fluxIntSum >= FLUX_HIGH_THRESH)
			{
				changeState(motor, STATE_MOTOR_CLOSED_WATCH_2);
			}
		}
		else
		{
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "INT 1 timeout!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	// @stateName: STATE_MOTOR_CLOSED_WATCH_2
	// @stateDescription: This state watches for a zero crossing to switch to the next state.
	// If the state times out before a zero cross is detected, the motor is stopped.
	// @stateInitConditions:
	// 1. isPrevValValid = 0
	// 2. ctr = 0;
	// 3. closedSetDuty(motor);
	// 4. setPWM(motor, STEP_2, motor->duty);
	case STATE_MOTOR_CLOSED_WATCH_2:
		if(motor->ctr < CLOSED_TIMEOUT)
		{
			motor->ctr++;
			motor->getSpdCtr++;
			if(motor->isMeasuredFreqValid == 1)
			{
				closedSetDuty(motor);
				setPWM(motor, STEP_2, motor->duty);
			}
			// Make sure previous value is valid
			if(motor->ctr > BLANK_TIME)
			{
				// Zero cross detected?
				if( (adcdata[motor->adcDataZPhaseIdx] <= (124*adcdata[motor->adcDataDcBusIdx]/280)) &&
				    (motor->zPhasePrevVal > (124*motor->dcBusPrevVal/280)))
				{
					// Set isPrevValValid to 0, set prd to ctr,
					// set timeout to 2*prd, set ctr to 0, run PI calc,
					// and change state to closed wait 2.
					if(motor->phaseId == PHASE_ID_ABC) {LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0);}
					motor->isMeasuredFreqValid = 1;
					changeState(motor, STATE_MOTOR_CLOSED_INT_2);
				}
			}
			else
			{
				motor->isPrevValValid = 1;
			}
		}
		else
		{
			// If no zero cross detected, stop motor
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "No ZC in WATCH 2!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	case STATE_MOTOR_CLOSED_INT_2:
		if(motor->ctr < CLOSED_TIMEOUT)
		{
			motor->ctr++;
			motor->getSpdCtr++;
			if(motor->isMeasuredFreqValid == 1)
			{
				closedSetDuty(motor);
				setPWM(motor, STEP_2, motor->duty);
			}
			motor->fluxIntSum += adcdata[motor->adcDataZPhaseIdx] - (124*adcdata[motor->adcDataDcBusIdx]/280);
			if(motor->fluxIntSum <= FLUX_LOW_THRESH)
			{
				changeState(motor, STATE_MOTOR_CLOSED_WATCH_3);
			}
		}
		else
		{
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "INT 2 timeout!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	case STATE_MOTOR_CLOSED_WATCH_3:
		if(motor->ctr < CLOSED_TIMEOUT)
		{
			motor->ctr++;
			motor->getSpdCtr++;
			if(motor->isMeasuredFreqValid == 1)
			{
				closedSetDuty(motor);
				setPWM(motor, STEP_3, motor->duty);
			}
			// Make sure previous value is valid
			if(motor->ctr > BLANK_TIME)
			{
				// Zero cross detected?
				if( (adcdata[motor->adcDataZPhaseIdx] >= (124*adcdata[motor->adcDataDcBusIdx]/280)) &&
				    (motor->zPhasePrevVal < (124*motor->dcBusPrevVal/280)) )
				{
					// Set isPrevValValid to 0, set prd to ctr,
					// set timeout to 2*prd, set ctr to 0, run PI calc,
					// and change state to closed wait 3.
					if(motor->phaseId == PHASE_ID_ABC) {LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0);}
					motor->isMeasuredFreqValid = 1;
					changeState(motor, STATE_MOTOR_CLOSED_INT_3);
				}
			}
			else
			{
				motor->isPrevValValid = 1;
			}
		}
		else
		{
			// If no zero cross detected, stop motor
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "No ZC in WATCH 3!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	case STATE_MOTOR_CLOSED_INT_3:
		if(motor->ctr < CLOSED_TIMEOUT)
		{
			motor->ctr++;
			motor->getSpdCtr++;
			if(motor->isMeasuredFreqValid == 1)
			{
				closedSetDuty(motor);
				setPWM(motor, STEP_3, motor->duty);
			}
			motor->fluxIntSum += adcdata[motor->adcDataZPhaseIdx] - (124*adcdata[motor->adcDataDcBusIdx]/280);
			if(motor->fluxIntSum >= FLUX_HIGH_THRESH)
			{
				changeState(motor, STATE_MOTOR_CLOSED_WATCH_4);
			}
		}
		else
		{
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "INT 3 timeout!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	case STATE_MOTOR_CLOSED_WATCH_4:
		if(motor->ctr < CLOSED_TIMEOUT)
		{
			motor->ctr++;
			motor->getSpdCtr++;
			if(motor->isMeasuredFreqValid == 1)
			{
				closedSetDuty(motor);
				setPWM(motor, STEP_4, motor->duty);
			}
			// Make sure previous value is valid
			if(motor->ctr > BLANK_TIME)
			{
				// Zero cross detected?
				if( (adcdata[motor->adcDataZPhaseIdx] <= (adcdata[motor->adcDataDcBusIdx]/2)) &&
				    (motor->zPhasePrevVal > (124*motor->dcBusPrevVal/280)) )
				{
					// Set isPrevValValid to 0, set prd to ctr,
					// set timeout to 2*prd, set ctr to 0, run PI calc,
					// and change state to closed wait 4.
					if(motor->phaseId == PHASE_ID_ABC) {LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0);}
					motor->isMeasuredFreqValid = 1;
					changeState(motor, STATE_MOTOR_CLOSED_INT_4);
				}
			}
			else
			{
				motor->isPrevValValid = 1;
			}
		}
		else
		{
			// If no zero cross detected, stop motor
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "No ZC in WATCH 4!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	case STATE_MOTOR_CLOSED_INT_4:
		if(motor->ctr < CLOSED_TIMEOUT)
		{
			motor->ctr++;
			motor->getSpdCtr++;
			if(motor->isMeasuredFreqValid == 1)
			{
				closedSetDuty(motor);
				setPWM(motor, STEP_4, motor->duty);
			}
			motor->fluxIntSum += adcdata[motor->adcDataZPhaseIdx] - (124*adcdata[motor->adcDataDcBusIdx]/280);
			if(motor->fluxIntSum <= FLUX_LOW_THRESH)
			{
				changeState(motor, STATE_MOTOR_CLOSED_WATCH_5);
			}
		}
		else
		{
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "INT 4 timeout!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	case STATE_MOTOR_CLOSED_WATCH_5:
		if(motor->ctr < CLOSED_TIMEOUT)
		{
			motor->ctr++;
			motor->getSpdCtr++;
			if(motor->isMeasuredFreqValid == 1)
			{
				closedSetDuty(motor);
				setPWM(motor, STEP_5, motor->duty);
			}
			// Make sure previous value is valid
			if(motor->ctr > BLANK_TIME)
			{
				// Zero cross detected?
				if((adcdata[motor->adcDataZPhaseIdx] >= (124*adcdata[motor->adcDataDcBusIdx]/280)) &&
				   (motor->zPhasePrevVal < (124*motor->dcBusPrevVal/280)))
				{
					// Set isPrevValValid to 0, set prd to ctr,
					// set timeout to 2*prd, set ctr to 0, run PI calc,
					// and change state to closed wait 5.
					if(motor->phaseId == PHASE_ID_ABC) {LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0);}
					motor->isMeasuredFreqValid = 1;
					changeState(motor, STATE_MOTOR_CLOSED_INT_5);
				}
			}
			else
			{
				motor->isPrevValValid = 1;
			}
		}
		else
		{
			// If no zero cross detected, stop motor
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "No ZC in WATCH 5!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;

	case STATE_MOTOR_CLOSED_INT_5:
		if(motor->ctr < CLOSED_TIMEOUT)
		{
			motor->ctr++;
			motor->getSpdCtr++;
			if(motor->isMeasuredFreqValid == 1)
			{
				closedSetDuty(motor);
				setPWM(motor, STEP_5, motor->duty);
			}
			motor->fluxIntSum += adcdata[motor->adcDataZPhaseIdx] - (124*adcdata[motor->adcDataDcBusIdx]/280);
			if(motor->fluxIntSum >= FLUX_HIGH_THRESH)
			{
				changeState(motor, STATE_MOTOR_CLOSED_WATCH_0);
			}
		}
		else
		{
			changeState(motor, STATE_MOTOR_STOP);
			txlength = snprintf((char*) txdata, TX_DATA_SIZE, "INT 5 timeout!\n\r");
			uartTx((uint32_t*) txdata, txlength);
		}
		break;
	default:
		break;
	}
	motor->zPhasePrevVal = adcdata[motor->adcDataZPhaseIdx];
	motor->dcBusPrevVal = adcdata[motor->adcDataDcBusIdx];
	LL_ADC_REG_SetSequencerRanks(ADC1, motor->zPhaseAdcRank, motor->adcActiveChannel);
}



// @func getClosedDuty
// This function takes in the error signal and uses
// a PI algorithm to return the PWM duty during
// closed loop control.
// @param int16_t error: the error between the
// actual and command speed
// @param int16_t sum: pointer to the running sum
// for the PI integrator
void closedSetDuty(Motor_TypeDef* motor)
{

	motor->piError = motor->cmdFreq - motor->measuredFreq;
	motor->piPropCalc = motor->piError*PI_PROP_CONST;
	//motor->piIntCalc = (motor->piError + motor->piSum)/(PI_INT_CONST*F_SAMP);
	motor->piIntCalc = PI_INT_CONST*(motor->piError + motor->piSum)/(F_SAMP);

	// Clamp integral term
	if(motor->piIntCalc > motor->piMax)
	{
		motor->piIntCalc = motor->piMax;
		//motor->piSum = PI_INT_CONST*F_SAMP*motor->piMax - motor->piError;
		motor->piSum = (F_SAMP*motor->piMax)/PI_INT_CONST - motor->piError;
	}
	else if(motor->piIntCalc < motor->piMin)
	{
		motor->piIntCalc = motor->piMin;
		motor->piSum = (F_SAMP*motor->piMin)/PI_INT_CONST - motor->piError;
	}
	else
	{
		motor->piSum += motor->piError;
	}

	// Calculate PI output
	motor->piCalc = motor->piPropCalc + motor->piIntCalc;

	// Clamp PI output
	if(motor->piCalc > motor->piMax)
	{
		motor->piCalc = motor->piMax;
	}
	else if (motor->piCalc < motor->piMin)
	{
		motor->piCalc = motor->piMin;
	}

	// Set duty to PI calculation
	motor->duty = (uint16_t) motor->piCalc;
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

#ifdef  USE_FULL_ASSERT
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
