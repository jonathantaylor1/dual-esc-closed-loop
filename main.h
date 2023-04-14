/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct{
	uint8_t phaseId;
	uint8_t state;
	uint8_t step;
	uint32_t phase1PWMCompare;
	uint32_t phase1EnableCompare;
	uint32_t phase2PWMCompare;
	uint32_t phase2EnableCompare;
	uint32_t phase3PWMCompare;
	uint32_t phase3EnableCompare;
	uint32_t cmdPrd;
	uint32_t cmdFreq;
	int32_t piSum;
	int32_t piError;
	int32_t piPropCalc;
	int32_t piIntCalc;
	int32_t piCalc;
	int32_t piMax;
	int32_t piMin;
	uint16_t duty;
	uint8_t dir;
	uint32_t ctr;
	uint32_t openPrdCtr;
	uint32_t openAccCtr;
	uint32_t getSpdCtr;
	int32_t fluxIntSum;
	uint32_t openPrdArrIdx;
	uint32_t cmdPrdArrIdx;
	uint32_t prd;
	uint32_t measuredFreq;
	uint8_t isMeasuredFreqValid;
	uint32_t zPhaseAdcRank;
	uint32_t phase1adcChannel;
	uint32_t phase2adcChannel;
	uint32_t phase3adcChannel;
	uint32_t dcBusAdcChannel;
	uint32_t adcActiveChannel;
	uint16_t adcDataZPhaseIdx;
	uint16_t adcDataDcBusIdx;
	uint16_t zPhasePrevVal;
	uint16_t dcBusPrevVal;
	uint32_t isPrevValValid;
	uint32_t closedWatchTimeout;
} Motor_TypeDef;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void motorsInit(void);
void arraysInit(void);
void setStep(uint8_t, uint8_t*, uint8_t, uint16_t);
void uartTx(uint32_t*, uint32_t);
void adcConvCpltCallback(void);
void uartRxCpltCallback(void);
void pwmPrdCpltCallback(void);
void adcConvert(void);
void changeState(Motor_TypeDef* motor, uint8_t newState);
void stateEval(Motor_TypeDef*);
void closedSetDuty(Motor_TypeDef* motor);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin LL_GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define TMS_Pin LL_GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin LL_GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin LL_GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */

// ADC DATA INDEX DEFINES
#define ADC_MOTOR1_Z	0
#define ADC_MOTOR2_Z	1
#define ADC_DCBUS		2

// PHASE ID DEFINES
#define PHASE_ID_ABC	0
#define PHASE_ID_UVW	1

// OUTPUT STEP DEFINES
#define STEP_Z	0
#define STEP_0	1
#define STEP_1	2
#define STEP_2	3
#define STEP_3	4
#define STEP_4	5
#define STEP_5	6

// MOTOR DIRECTION DEFINES
#define MOTOR_DIRECTION_FWD	0
#define MOTOR_DIRECTION_REV	1

// MOTOR STATES
#define STATE_MOTOR_DEBUG_Z						0
#define STATE_MOTOR_DEBUG_0						1
#define STATE_MOTOR_DEBUG_1						2
#define STATE_MOTOR_DEBUG_2						3
#define STATE_MOTOR_DEBUG_3						4
#define STATE_MOTOR_DEBUG_4						5
#define STATE_MOTOR_DEBUG_5						6
#define STATE_MOTOR_STOP						7
#define STATE_MOTOR_ALIGN_3						8
#define STATE_MOTOR_ALIGN_4						9
#define STATE_MOTOR_OPEN_ACC_0					10
#define STATE_MOTOR_OPEN_ACC_1					11
#define STATE_MOTOR_OPEN_ACC_2					12
#define STATE_MOTOR_OPEN_ACC_3					13
#define STATE_MOTOR_OPEN_ACC_4					14
#define STATE_MOTOR_OPEN_ACC_5					15
#define STATE_MOTOR_HANDOFF_0					16
#define STATE_MOTOR_HANDOFF_1					17
#define STATE_MOTOR_HANDOFF_2					18
#define STATE_MOTOR_HANDOFF_3					19
#define STATE_MOTOR_HANDOFF_4					20
#define STATE_MOTOR_HANDOFF_5					21
#define STATE_MOTOR_CLOSED_GET_SPD_WATCH_0		22
#define STATE_MOTOR_CLOSED_GET_SPD_WAIT_0		23
#define STATE_MOTOR_CLOSED_GET_SPD_WATCH_1		24
#define STATE_MOTOR_CLOSED_GET_SPD_WAIT_1		25
#define STATE_MOTOR_CLOSED_WATCH_0				26
#define STATE_MOTOR_CLOSED_INT_0				27
#define STATE_MOTOR_CLOSED_WATCH_1				28
#define STATE_MOTOR_CLOSED_INT_1				29
#define STATE_MOTOR_CLOSED_WATCH_2				30
#define STATE_MOTOR_CLOSED_INT_2				31
#define STATE_MOTOR_CLOSED_WATCH_3				32
#define STATE_MOTOR_CLOSED_INT_3				33
#define STATE_MOTOR_CLOSED_WATCH_4				34
#define STATE_MOTOR_CLOSED_INT_4				35
#define STATE_MOTOR_CLOSED_WATCH_5				36
#define STATE_MOTOR_CLOSED_INT_5				37

// UART CONTROL STATES
#define STATE_CTRL_DEBUG	0
#define STATE_CTRL_RUN		1

// GENERAL CONSTANTS
#define ADC_DATA_SIZE						3
#define RX_DATA_SIZE						1
#define TX_DATA_SIZE						50
#define ZERO_DUTY							5000
#define DEBUG_DUTY							80
#define EMF_THRESH							317
#define NOISE_THRESH						50
#define COMMUTATION_THRESH					250
#define KV_CONSTANT							1900
#define NUM_POLES							4
#define UVLO_TIMEOUT						5000
#define F_SAMP								5000

// SPIN CHECK CONSTANTS
#define SPIN_CHECK_OBSERVE_TIMEOUT			300
#define SPIN_CHECK_GET_SPD_TIMEOUT			50

// ALIGN CONSTANTS
#define ALIGN_TIMEOUT						1000
#define ALIGN_DUTY							200

// OPEN-LOOP CONSTANTS
#define F_SAMP_SQRD							25000000
#define SIX_A1								180
#define THREE_A2							90
#define HANDOFF_PRD							20
#define OPEN_DUTY_COEFF						10000
#define OPEN_ALPHA							2
#define PRD_ARR_OPEN_ACC_NUMEL				30
#define OPEN_DUTY_OFFSET					0
#define OPEN_INIT_PRD						10000
#define T_SAMP								0.0002

// CLOSED-LOOP CONSTANTS
#define CLOSED_TIMEOUT				200
#define PRD_ARR_CLOSED_REF_NUMEL	4500
#define FLUX_HIGH_THRESH			4083
#define FLUX_LOW_THRESH				-4083
#define BLANK_TIME					0
#define PI_INT_CONST				50
#define PI_PROP_CONST				50
#define PI_MAX_TRIM					1
#define PI_MIN_TRIM					10
#define CMD_FREQ_ARR_SIZE			127
#define CMD_FREQ_MIN				40
#define CMD_FREQ_MAX				83

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
