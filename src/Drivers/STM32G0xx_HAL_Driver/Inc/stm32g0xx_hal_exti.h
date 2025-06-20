/**
 ******************************************************************************
 * @file    stm32g0xx_hal_exti.h
 * @author  MCD Application Team
 * @brief   Header file of EXTI HAL module.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2018 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32G0xx_HAL_EXTI_H
#define STM32G0xx_HAL_EXTI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal_def.h"

/** @addtogroup STM32G0xx_HAL_Driver
 * @{
 */

/** @defgroup EXTI EXTI
 * @brief EXTI HAL module driver
 * @{
 */

/* Exported types ------------------------------------------------------------*/

/** @defgroup EXTI_Exported_Types EXTI Exported Types
 * @{
 */
typedef enum {
	HAL_EXTI_COMMON_CB_ID = 0x00U,
	HAL_EXTI_RISING_CB_ID = 0x01U,
	HAL_EXTI_FALLING_CB_ID = 0x02U,
} EXTI_CallbackIDTypeDef;

/**
 * @brief  EXTI Handle structure definition
 */
typedef struct {
	uint32_t Line; /*!<  Exti line number */
	void (*RisingCallback)(void); /*!<  Exti rising callback */
	void (*FallingCallback)(void); /*!<  Exti falling callback */
} EXTI_HandleTypeDef;

/**
 * @brief  EXTI Configuration structure definition
 */
typedef struct {
	uint32_t Line; /*!< The Exti line to be configured. This parameter
	 can be a value of @ref EXTI_Line */
	uint32_t Mode; /*!< The Exit Mode to be configured for a core.
	 This parameter can be a combination of @ref EXTI_Mode */
	uint32_t Trigger; /*!< The Exti Trigger to be configured. This parameter
	 can be a value of @ref EXTI_Trigger */
	uint32_t GPIOSel; /*!< The Exti GPIO multiplexer selection to be configured.
	 This parameter is only possible for line 0 to 15. It
	 can be a value of @ref EXTI_GPIOSel */
} EXTI_ConfigTypeDef;

/**
 * @}
 */

/* Exported constants --------------------------------------------------------*/
/** @defgroup EXTI_Exported_Constants EXTI Exported Constants
 * @{
 */

/** @defgroup EXTI_Line  EXTI Line
 * @{
 */
#define EXTI_LINE_0                         (EXTI_GPIO     | EXTI_REG1 | 0x00u)
#define EXTI_LINE_1                         (EXTI_GPIO     | EXTI_REG1 | 0x01u)
#define EXTI_LINE_2                         (EXTI_GPIO     | EXTI_REG1 | 0x02u)
#define EXTI_LINE_3                         (EXTI_GPIO     | EXTI_REG1 | 0x03u)
#define EXTI_LINE_4                         (EXTI_GPIO     | EXTI_REG1 | 0x04u)
#define EXTI_LINE_5                         (EXTI_GPIO     | EXTI_REG1 | 0x05u)
#define EXTI_LINE_6                         (EXTI_GPIO     | EXTI_REG1 | 0x06u)
#define EXTI_LINE_7                         (EXTI_GPIO     | EXTI_REG1 | 0x07u)
#define EXTI_LINE_8                         (EXTI_GPIO     | EXTI_REG1 | 0x08u)
#define EXTI_LINE_9                         (EXTI_GPIO     | EXTI_REG1 | 0x09u)
#define EXTI_LINE_10                        (EXTI_GPIO     | EXTI_REG1 | 0x0Au)
#define EXTI_LINE_11                        (EXTI_GPIO     | EXTI_REG1 | 0x0Bu)
#define EXTI_LINE_12                        (EXTI_GPIO     | EXTI_REG1 | 0x0Cu)
#define EXTI_LINE_13                        (EXTI_GPIO     | EXTI_REG1 | 0x0Du)
#define EXTI_LINE_14                        (EXTI_GPIO     | EXTI_REG1 | 0x0Eu)
#define EXTI_LINE_15                        (EXTI_GPIO     | EXTI_REG1 | 0x0Fu)
#define EXTI_LINE_16                        (EXTI_CONFIG   | EXTI_REG1 | 0x10u)
#if defined(COMP1)
#define EXTI_LINE_17                        (EXTI_CONFIG   | EXTI_REG1 | 0x11u)
#else
#define EXTI_LINE_17                        (EXTI_RESERVED | EXTI_REG1 | 0x11u)
#endif /* COMP1 */
#if defined(COMP2)
#define EXTI_LINE_18                        (EXTI_CONFIG   | EXTI_REG1 | 0x12u)
#else
#define EXTI_LINE_18                        (EXTI_RESERVED | EXTI_REG1 | 0x12u)
#endif /* COMP2 */
#define EXTI_LINE_19                        (EXTI_DIRECT   | EXTI_REG1 | 0x13u)
#if defined(COMP3)
#define EXTI_LINE_20                        (EXTI_CONFIG   | EXTI_REG1 | 0x14u)
#else
#define EXTI_LINE_20                        (EXTI_RESERVED | EXTI_REG1 | 0x14u)
#endif /* COMP3 */
#define EXTI_LINE_21                        (EXTI_DIRECT   | EXTI_REG1 | 0x15u)
#if defined(RCC_CCIPR_I2C2SEL)
#define EXTI_LINE_22                        (EXTI_DIRECT   | EXTI_REG1 | 0x16u)
#else
#define EXTI_LINE_22                        (EXTI_RESERVED | EXTI_REG1 | 0x16u)
#endif /* RCC_CCIPR_I2C2SEL */
#define EXTI_LINE_23                        (EXTI_DIRECT   | EXTI_REG1 | 0x17u)
#if defined(RCC_CCIPR_USART3SEL)
#define EXTI_LINE_24                        (EXTI_DIRECT   | EXTI_REG1 | 0x18u)
#else
#define EXTI_LINE_24                        (EXTI_RESERVED | EXTI_REG1 | 0x18u)
#endif /* RCC_CCIPR_USART3SEL */
#define EXTI_LINE_25                        (EXTI_DIRECT   | EXTI_REG1 | 0x19u)
#if defined(RCC_CCIPR_USART2SEL)
#define EXTI_LINE_26                        (EXTI_DIRECT   | EXTI_REG1 | 0x1Au)
#else
#define EXTI_LINE_26                        (EXTI_RESERVED | EXTI_REG1 | 0x1Au)
#endif /* RCC_CCIPR_USART2SEL */
#if defined(CEC)
#define EXTI_LINE_27                        (EXTI_DIRECT   | EXTI_REG1 | 0x1Bu)
#else
#define EXTI_LINE_27                        (EXTI_RESERVED | EXTI_REG1 | 0x1Bu)
#endif /* CEC */
#if defined(LPUART1)
#define EXTI_LINE_28                        (EXTI_DIRECT   | EXTI_REG1 | 0x1Cu)
#else
#define EXTI_LINE_28                        (EXTI_RESERVED | EXTI_REG1 | 0x1Cu)
#endif /* LPUART1 */
#if defined(LPTIM1)
#define EXTI_LINE_29                        (EXTI_DIRECT   | EXTI_REG1 | 0x1Du)
#else
#define EXTI_LINE_29                        (EXTI_RESERVED | EXTI_REG1 | 0x1Du)
#endif /* LPTIM1 */
#if defined(LPTIM2)
#define EXTI_LINE_30                        (EXTI_DIRECT   | EXTI_REG1 | 0x1Eu)
#else
#define EXTI_LINE_30                        (EXTI_RESERVED | EXTI_REG1 | 0x1Eu)
#endif /* LPTIM2 */
#define EXTI_LINE_31                        (EXTI_DIRECT   | EXTI_REG1 | 0x1Fu)
#if defined(UCPD1)
#define EXTI_LINE_32                        (EXTI_DIRECT   | EXTI_REG2 | 0x00u)
#else
#define EXTI_LINE_32                        (EXTI_RESERVED | EXTI_REG2 | 0x00u)
#endif /* UCPD1 */
#if defined(UCPD2)
#define EXTI_LINE_33                        (EXTI_DIRECT   | EXTI_REG2 | 0x01u)
#else
#define EXTI_LINE_33                        (EXTI_RESERVED | EXTI_REG2 | 0x01u)
#endif /* UCPD2 */ 
#if defined(STM32G0C1xx) || defined(STM32G0B1xx)
#define EXTI_LINE_34                        (EXTI_CONFIG   | EXTI_REG2 | 0x02u)
#else
#define EXTI_LINE_34                        (EXTI_RESERVED | EXTI_REG2 | 0x02u)
#endif /* STM32G0C1xx || STM32G0B1xx */
#if defined(LPUART2)
#define EXTI_LINE_35                        (EXTI_DIRECT   | EXTI_REG2 | 0x03u)
#else
#define EXTI_LINE_35                        (EXTI_RESERVED | EXTI_REG2 | 0x03u)
#endif /* LPUART2 */
#if defined(STM32G0C1xx) || defined(STM32G0B1xx) || defined(STM32G0B0xx)
#define EXTI_LINE_36                        (EXTI_DIRECT | EXTI_REG2 | 0x04u)
#endif /* STM32G0C1xx || STM32G0B1xx || STM32G0B0xx */
/**
 * @}
 */

/** @defgroup EXTI_Mode  EXTI Mode
 * @{
 */
#define EXTI_MODE_NONE                      0x00000000u
#define EXTI_MODE_INTERRUPT                 0x00000001u
#define EXTI_MODE_EVENT                     0x00000002u
/**
 * @}
 */

/** @defgroup EXTI_Trigger  EXTI Trigger
 * @{
 */
#define EXTI_TRIGGER_NONE                   0x00000000u
#define EXTI_TRIGGER_RISING                 0x00000001u
#define EXTI_TRIGGER_FALLING                0x00000002u
#define EXTI_TRIGGER_RISING_FALLING         (EXTI_TRIGGER_RISING | EXTI_TRIGGER_FALLING)
/**
 * @}
 */

/** @defgroup EXTI_GPIOSel  EXTI GPIOSel
 * @brief
 * @{
 */
#define EXTI_GPIOA                          0x00000000u
#define EXTI_GPIOB                          0x00000001u
#define EXTI_GPIOC                          0x00000002u
#define EXTI_GPIOD                          0x00000003u
#if defined(GPIOE)
#define EXTI_GPIOE                          0x00000004u
#endif /* GPIOE */
#define EXTI_GPIOF                          0x00000005u
/**
 * @}
 */

/**
 * @}
 */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup EXTI_Exported_Macros EXTI Exported Macros
 * @{
 */

/**
 * @}
 */

/* Private constants --------------------------------------------------------*/
/** @defgroup EXTI_Private_Constants EXTI Private Constants
 * @{
 */
/**
 * @brief  EXTI Line property definition
 */
#define EXTI_PROPERTY_SHIFT                  24u
#define EXTI_DIRECT                         (0x01uL << EXTI_PROPERTY_SHIFT)
#define EXTI_CONFIG                         (0x02uL << EXTI_PROPERTY_SHIFT)
#define EXTI_GPIO                           ((0x04uL << EXTI_PROPERTY_SHIFT) | EXTI_CONFIG)
#define EXTI_RESERVED                       (0x08uL << EXTI_PROPERTY_SHIFT)
#define EXTI_PROPERTY_MASK                  (EXTI_DIRECT | EXTI_CONFIG | EXTI_GPIO)

/**
 * @brief  EXTI Register and bit usage
 */
#define EXTI_REG_SHIFT                      16u
#define EXTI_REG1                           (0x00uL << EXTI_REG_SHIFT)
#define EXTI_REG2                           (0x01uL << EXTI_REG_SHIFT)
#define EXTI_REG_MASK                       (EXTI_REG1 | EXTI_REG2)
#define EXTI_PIN_MASK                       0x0000001Fu

/**
 * @brief  EXTI Mask for interrupt & event mode
 */
#define EXTI_MODE_MASK                      (EXTI_MODE_EVENT | EXTI_MODE_INTERRUPT)

/**
 * @brief  EXTI Mask for trigger possibilities
 */
#define EXTI_TRIGGER_MASK                   (EXTI_TRIGGER_RISING | EXTI_TRIGGER_FALLING)

/**
 * @brief  EXTI Line number
 */
#if defined(STM32G0C1xx) || defined(STM32G0B1xx)
#define EXTI_LINE_NB                        37uL
#elif defined(STM32G0B0xx)
#define EXTI_LINE_NB                        37uL
#elif defined(STM32G081xx) || defined(STM32G071xx)
#define EXTI_LINE_NB                        34uL
#elif defined(STM32G070xx)
#define EXTI_LINE_NB                        34uL
#elif defined(STM32G041xx) || defined(STM32G031xx)
#define EXTI_LINE_NB                        32uL
#else
#define EXTI_LINE_NB                        32uL
#endif /* STM32G0C1xx || STM32G0B1xx */

/**
 * @}
 */

/* Private macros ------------------------------------------------------------*/
/** @defgroup EXTI_Private_Macros EXTI Private Macros
 * @{
 */
#define IS_EXTI_LINE(__EXTI_LINE__)     ((((__EXTI_LINE__) & ~(EXTI_PROPERTY_MASK | EXTI_REG_MASK | EXTI_PIN_MASK)) == 0x00u) && \
                                        ((((__EXTI_LINE__) & EXTI_PROPERTY_MASK) == EXTI_DIRECT)   || \
                                         (((__EXTI_LINE__) & EXTI_PROPERTY_MASK) == EXTI_CONFIG)   || \
                                         (((__EXTI_LINE__) & EXTI_PROPERTY_MASK) == EXTI_GPIO))    && \
                                         (((__EXTI_LINE__) & (EXTI_REG_MASK | EXTI_PIN_MASK))      < \
                                         (((EXTI_LINE_NB / 32u) << EXTI_REG_SHIFT) | (EXTI_LINE_NB % 32u))))

#define IS_EXTI_MODE(__MODE__)          ((((__MODE__) & EXTI_MODE_MASK) != 0x00u) && \
                                         (((__MODE__) & ~EXTI_MODE_MASK) == 0x00u))

#define IS_EXTI_TRIGGER(__EXTI_LINE__)  (((__EXTI_LINE__) & ~EXTI_TRIGGER_MASK) == 0x00u)

#define IS_EXTI_PENDING_EDGE(__EXTI_LINE__)  (((__EXTI_LINE__) == EXTI_TRIGGER_RISING) || \
                                              ((__EXTI_LINE__) == EXTI_TRIGGER_FALLING))

#define IS_EXTI_CONFIG_LINE(__EXTI_LINE__)   (((__EXTI_LINE__) & EXTI_CONFIG) != 0x00u)

#if defined(GPIOE)
#define IS_EXTI_GPIO_PORT(__PORT__)     (((__PORT__) == EXTI_GPIOA) || \
                                         ((__PORT__) == EXTI_GPIOB) || \
                                         ((__PORT__) == EXTI_GPIOC) || \
                                         ((__PORT__) == EXTI_GPIOD) || \
                                         ((__PORT__) == EXTI_GPIOE) || \
                                         ((__PORT__) == EXTI_GPIOF))
#else
#define IS_EXTI_GPIO_PORT(__PORT__)     (((__PORT__) == EXTI_GPIOA) || \
                                         ((__PORT__) == EXTI_GPIOB) || \
                                         ((__PORT__) == EXTI_GPIOC) || \
                                         ((__PORT__) == EXTI_GPIOD) || \
                                         ((__PORT__) == EXTI_GPIOF))
#endif /* GPIOE */

#define IS_EXTI_GPIO_PIN(__PIN__)       ((__PIN__) < 16u)

/**
 * @}
 */

/* Exported functions --------------------------------------------------------*/
/** @defgroup EXTI_Exported_Functions EXTI Exported Functions
 * @brief    EXTI Exported Functions
 * @{
 */

/** @defgroup EXTI_Exported_Functions_Group1 Configuration functions
 * @brief    Configuration functions
 * @{
 */
/* Configuration functions ****************************************************/
HAL_StatusTypeDef HAL_EXTI_SetConfigLine(EXTI_HandleTypeDef *hexti,
		EXTI_ConfigTypeDef *pExtiConfig);
HAL_StatusTypeDef HAL_EXTI_GetConfigLine(EXTI_HandleTypeDef *hexti,
		EXTI_ConfigTypeDef *pExtiConfig);
HAL_StatusTypeDef HAL_EXTI_ClearConfigLine(EXTI_HandleTypeDef *hexti);
HAL_StatusTypeDef HAL_EXTI_RegisterCallback(EXTI_HandleTypeDef *hexti,
		EXTI_CallbackIDTypeDef CallbackID, void (*pPendingCbfn)(void));
HAL_StatusTypeDef HAL_EXTI_GetHandle(EXTI_HandleTypeDef *hexti,
		uint32_t ExtiLine);
/**
 * @}
 */

/** @defgroup EXTI_Exported_Functions_Group2 IO operation functions
 * @brief    IO operation functions
 * @{
 */
/* IO operation functions *****************************************************/
void HAL_EXTI_IRQHandler(EXTI_HandleTypeDef *hexti);
uint32_t HAL_EXTI_GetPending(EXTI_HandleTypeDef *hexti, uint32_t Edge);
void HAL_EXTI_ClearPending(EXTI_HandleTypeDef *hexti, uint32_t Edge);
void HAL_EXTI_GenerateSWI(EXTI_HandleTypeDef *hexti);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* STM32G0xx_HAL_EXTI_H */

