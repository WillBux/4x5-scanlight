/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32g0xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32g0xx_it.h"
#include "usbpd.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_DRD_FS;
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim3;

/* USER CODE BEGIN EV */
/* During the cleanup phase in EE_Init, AddressRead is the address being read */
extern __IO uint32_t AddressRead;
/* Flag equal to 1 when the cleanup phase is in progress, 0 if not */
extern __IO uint8_t CleanupPhase;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* Check if NMI is due to flash ECCD (error detection) */
	if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_ECCD)) {
		if (CleanupPhase == 1) {
			if ((AddressRead >= START_PAGE_ADDRESS)
					&& (AddressRead <= END_EEPROM_ADDRESS)) {
				/* Delete the corrupted flash address */
				if (EE_DeleteCorruptedFlashAddress((uint32_t) AddressRead)
						== EE_OK) {
					/* Resume execution if deletion succeeds */
					return;
				}
				/* If we do not succeed to delete the corrupted flash address */
				/* This might be because we try to write 0 at a line already considered at 0 which is a forbidden operation */
				/* This problem triggers PROGERR, PGAERR and PGSERR flags */
				else {
					/* We check if the flags concerned have been triggered */
					if ((__HAL_FLASH_GET_FLAG(FLASH_FLAG_PROGERR))
							&& (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGAERR))
							&& (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGSERR))) {
						/* If yes, we clear them */
						__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PROGERR);
						__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
						__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);

						/* And we exit from NMI without doing anything */
						/* We do not invalidate that line because it is not programmable at 0 till the next page erase */
						/* The only consequence is that this line will trigger a new NMI later */
						return;
					}
				}
			}
		} else {
			__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCD);
			return;
		}
	}

	/* Go to infinite loop when NMI occurs in case:
	 - ECCD is raised in eeprom emulation flash pages but corrupted flash address deletion fails (except PROGERR, PGAERR and PGSERR)
	 - ECCD is raised out of eeprom emulation flash pages
	 - no ECCD is raised */

	/* Go to infinite loop when NMI occurs */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1) {
	}
	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles USB, UCPD1 and UCPD2 global interrupts.
 */
void USB_UCPD1_2_IRQHandler(void) {
	/* USER CODE BEGIN USB_UCPD1_2_IRQn 0 */

	/* USER CODE END USB_UCPD1_2_IRQn 0 */
	HAL_PCD_IRQHandler(&hpcd_USB_DRD_FS);
	USBPD_PORT0_IRQHandler();

	/* USER CODE BEGIN USB_UCPD1_2_IRQn 1 */

	/* USER CODE END USB_UCPD1_2_IRQn 1 */
}

/**
 * @brief This function handles DMA1 channel 1 interrupt.
 */
void DMA1_Channel1_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

	/* USER CODE END DMA1_Channel1_IRQn 0 */
	/* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

	/* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
 * @brief This function handles DMA1 channel 2 and channel 3 interrupts.
 */
void DMA1_Channel2_3_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

	/* USER CODE END DMA1_Channel2_3_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_i2c2_tx);
	/* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

	/* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
 * @brief This function handles TIM3, TIM4 global Interrupt.
 */
void TIM3_TIM4_IRQHandler(void) {
	/* USER CODE BEGIN TIM3_TIM4_IRQn 0 */

	/* USER CODE END TIM3_TIM4_IRQn 0 */
	HAL_TIM_IRQHandler(&htim3);
	/* USER CODE BEGIN TIM3_TIM4_IRQn 1 */

	/* USER CODE END TIM3_TIM4_IRQn 1 */
}

/**
 * @brief This function handles I2C2, I2C3 Interrupt (combined with EXTI 24 and EXTI 22).
 */
void I2C2_3_IRQHandler(void) {
	/* USER CODE BEGIN I2C2_3_IRQn 0 */

	/* USER CODE END I2C2_3_IRQn 0 */
	if (hi2c2.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR)) {
		HAL_I2C_ER_IRQHandler(&hi2c2);
	} else {
		HAL_I2C_EV_IRQHandler(&hi2c2);
	}
	/* USER CODE BEGIN I2C2_3_IRQn 1 */

	/* USER CODE END I2C2_3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void FLASH_IRQHandler(void) {
	if ((FLASH->ECCR & FLASH_FLAG_ECCC) != 0) {
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCC);
	}
	HAL_FLASH_IRQHandler();
	__HAL_FLASH_ENABLE_IT(FLASH_IT_ECCC);
}

/* USER CODE END 1 */
