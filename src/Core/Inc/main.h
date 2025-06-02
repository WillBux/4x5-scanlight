/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

#include "stm32g0xx_ll_ucpd.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_dma.h"

#include "stm32g0xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VDDA_APPLI 3300
#define R_SW_Pin GPIO_PIN_3
#define R_SW_GPIO_Port GPIOA
#define R_Q_Pin GPIO_PIN_4
#define R_Q_GPIO_Port GPIOA
#define R_I_Pin GPIO_PIN_5
#define R_I_GPIO_Port GPIOA
#define VSENSE_Pin GPIO_PIN_10
#define VSENSE_GPIO_Port GPIOB
#define FLT_IN_Pin GPIO_PIN_9
#define FLT_IN_GPIO_Port GPIOA
#define DB_OUT_Pin GPIO_PIN_6
#define DB_OUT_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define R_GPIO_Port GPIOA
#define R_IQSW_Pins (R_SW_Pin | R_Q_Pin | R_I_Pin)
#define R_IQ_Pins R_Q_PIN | R_I_PIN
#define PWM_R_Pin GPIO_PIN_0
#define PWM_R_Port GPIOA
#define PWM_G_Pin GPIO_PIN_1
#define PWM_G_Port GPIOA
#define PWM_B_Pin GPIO_PIN_2
#define PWM_B_Port GPIOA


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
