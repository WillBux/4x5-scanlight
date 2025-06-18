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
#include "cmsis_os.h"
#include "app_tcpp.h"
#include "usbpd.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>

#include "ssd1306.h"
#include "eeprom_emul.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY 2 // milliseconds
#define BUTTON_DEBOUNCE_MS 15
#define BOOTLOADER_ADDRESS 0x1FFF0000
#define BOOTLOADER_FLAG_VALUE 0xDEADBEEF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_tx;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.priority = (osPriority_t) osPriorityNormal, .stack_size = 128 * 4 };
/* USER CODE BEGIN PV */
extern int _bflag; // boot flag
uint32_t *dfu_boot_flag;
typedef void (*pFunction)(void);
pFunction JumpToApplication;
uint32_t JumpAddress;

#define MAX_DISPLAY_TXT 18
char display_txt[18];

// Rotary encoder vars
volatile uint8_t colors[3] = { 0, 0, 0 };
volatile uint8_t pwm[3] = { 0, 0, 0 };
volatile uint8_t select;
volatile uint8_t on = 0;
volatile uint8_t save_no = 0;

void EncoderTask(void *argument);

osThreadId_t encoderTaskHandle;
const osThreadAttr_t encoderTask_attributes = { .name = "encoderTask",
		.priority = (osPriority_t) osPriorityNormal1, .stack_size = 256 * 4 };

// Gray code state table
const int8_t rotary_table[16] = { 0, -1, +1, 0, +1, 0, 0, -1, -1, 0, 0, +1, 0,
		+1, -1, 0 };

const uint8_t ULINE_X[5] = { 0, 42, 84, 0, 28 };
const uint8_t ULINE_Y[5] = { 16, 16, 16, 29, 29 };
const uint8_t ULINE_LEN[5] = { 7, 7, 7, 7, 28 };

// EEPROM
EE_Status ee_status = EE_OK;
uint32_t eeprom_read;
__IO uint32_t ErasingOnGoing = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_UCPD1_Init(void);
static void MX_IWDG_Init(void);
static void MX_CRC_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  FLASH end of operation interrupt callback.
 * @param  ReturnValue: The value saved in this parameter depends on the ongoing procedure
 *                  Mass Erase: Bank number which has been requested to erase
 *                  Page Erase: Page which has been erased
 *                  Program: Address which was selected for data program
 * @retval None
 */
void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue) {
	/* Call CleanUp callback when all requested pages have been erased */
	if ((ReturnValue == (START_PAGE + PAGES_NUMBER / 2 - 1))
			|| (ReturnValue == (START_PAGE + PAGES_NUMBER - 1))) {
		EE_EndOfCleanup_UserCallback();
	}
}

/**
 * @brief  Clean Up end of operation interrupt callback.
 * @param  None
 * @retval None
 */
void EE_EndOfCleanup_UserCallback(void) {
	ErasingOnGoing = 0;
}

void InitDisplay(void) {
	ssd1306_Init();
	ssd1306_FlipScreenVertically();
	ssd1306_Clear();
	ssd1306_SetColor(White);
	ssd1306_UpdateScreen();
	HAL_Delay(10);
}

void JumpToBootloader(void) {
	/* Wait any cleanup is completed before accessing flash again */
	while (ErasingOnGoing == 1) {
	}

	ssd1306_DisplayOff();
	ssd1306_UpdateScreen();
	osDelay(10);
	vTaskSuspendAll();
	*dfu_boot_flag = BOOTLOADER_FLAG_VALUE;
	HAL_NVIC_SystemReset();
}
// Overwrite printf for USB CDC
int _write(int file, char *ptr, int len) {
	CDC_Transmit_FS((uint8_t*) ptr, len);
	return len;
}

void USB_ProcessReceivedData(uint8_t *Buf, uint32_t Len) {
	CDC_Transmit_FS(Buf, Len); 	// echo input
	// check for boot flag 'b'
	for (uint32_t i = 0; i < Len; ++i) {
		if (Buf[i] == 0x62) { // `b`
			JumpToBootloader();
		}
	}
}

void voltageTooLow(uint8_t volts) {
	snprintf(display_txt, MAX_DISPLAY_TXT, "USB-PD not 20V");
	ssd1306_SetCursor(0, 12);
	ssd1306_WriteString(display_txt, Font_7x10);
	ssd1306_UpdateScreen();
}

void displayEepromIssue() {
	snprintf(display_txt, MAX_DISPLAY_TXT, "EEPROM %i", ee_status);
	ssd1306_SetCursor(0, 12);
	ssd1306_WriteString(display_txt, Font_7x10);
	ssd1306_UpdateScreen();
}

void updateScreen() {
	ssd1306_Clear();
	snprintf(display_txt, MAX_DISPLAY_TXT, "R:%-3i G:%-3i B:%-3i", colors[0],
			colors[1], colors[2]);
	ssd1306_SetCursor(0, 6);
	ssd1306_WriteString(display_txt, Font_7x10);
	ssd1306_SetCursor(0, 19);
	if (save_no == 0) {
		snprintf(display_txt, MAX_DISPLAY_TXT, "%s Save", on ? "ON " : "OFF");
	} else {
		snprintf(display_txt, MAX_DISPLAY_TXT, "%s %s %i", on ? "ON " : "OFF",
				(save_no % 2 == 0) ? "Save" : "Read", (save_no + 1) / 2);
	}
	ssd1306_WriteString(display_txt, Font_7x10);
	ssd1306_DrawHorizontalLine(ULINE_X[select], ULINE_Y[select],
			ULINE_LEN[select]);
	ssd1306_UpdateScreen();
}

void EE_Read_Color(uint8_t address) {
	if (address > NB_OF_VARIABLES) {
		return;
	}
	/* Wait any cleanup is completed before accessing flash again */
	while (ErasingOnGoing == 1) {
	}
	ee_status = EE_ReadVariable32bits(address, &eeprom_read);
	if ((ee_status & EE_STATUSMASK_CLEANUP ) == EE_STATUSMASK_CLEANUP) {
		ErasingOnGoing = 1;
		ee_status |= EE_CleanUp();
	}
	if ((ee_status & EE_STATUSMASK_ERROR ) == EE_STATUSMASK_ERROR) {
		displayEepromIssue();
	}
	colors[0] = eeprom_read >> 0;
	colors[1] = eeprom_read >> 8;
	colors[2] = eeprom_read >> 16;
}

void EE_Write_Color(uint8_t address) {
	/* Wait any cleanup is completed before accessing flash again */
	while (ErasingOnGoing == 1) {
	}
	eeprom_read = colors[0] | colors[1] << 8 | colors[2] << 16;
	ee_status = EE_WriteVariable32bits(address, eeprom_read);
	if ((ee_status & EE_STATUSMASK_CLEANUP ) == EE_STATUSMASK_CLEANUP) {
		ErasingOnGoing = 1;
		ee_status |= EE_CleanUp();
	}
	if ((ee_status & EE_STATUSMASK_ERROR ) == EE_STATUSMASK_ERROR) {
		Error_Handler();
	}
}

void EncoderTask(void *argument) {
	uint8_t prev_IQ = ((HAL_GPIO_ReadPin(R_I_GPIO_Port, R_I_Pin) << 1)
			| HAL_GPIO_ReadPin(R_Q_GPIO_Port, R_Q_Pin));
	// for debounce button
	uint8_t lastBtn = HAL_GPIO_ReadPin(R_SW_GPIO_Port, R_SW_Pin);
	uint8_t btnStableState = lastBtn;
	uint32_t btnChangedTime = 0;
	uint8_t currBtn;

	// for rotary fsm
	uint8_t curr_I, curr_Q, curr_IQ;
	uint8_t index;
	int8_t delta = 0;

	for (;;) {
		// rotary FSM
		curr_I = HAL_GPIO_ReadPin(R_I_GPIO_Port, R_I_Pin);
		curr_Q = HAL_GPIO_ReadPin(R_Q_GPIO_Port, R_Q_Pin);
		curr_IQ = (curr_I << 1) | curr_Q;

		index = prev_IQ << 2 | curr_IQ;
		delta += rotary_table[index];

		if (rotary_table[index] != 0) {
			prev_IQ = curr_IQ;
		}

		if (delta >= 4) {
			// only add 1 if under 100
			if (select == 3) {
				on = 1;
			} else if (select == 4) {
				save_no += save_no > NB_OF_VARIABLES * 2 ? 0 : 1;
			} else if (colors[select] < 100) {
				++colors[select];
			}
			delta -= 4;
			//printf("Rot -1\n");
		} else if (delta <= -4) {
			// only subtract 1 if over 0
			if (select == 3) {
				on = 0;
			} else if (select == 4) {
				save_no -= save_no > 0 ? 1 : 0;
			} else if (colors[select] > 0) {
				--colors[select];
			}
			delta += 4;
			//printf("Rot +1\n");
		}

		// Button debounce
		currBtn = HAL_GPIO_ReadPin(R_SW_GPIO_Port, R_SW_Pin);
		if (currBtn != lastBtn) { // button state changed, restart debounce timer
			btnChangedTime = osKernelGetTickCount();
			lastBtn = currBtn;
		} else {
			// button state stable check if it's held long enough
			if (currBtn
					!= btnStableState&& (osKernelGetTickCount() - btnChangedTime) >= BUTTON_DEBOUNCE_MS) {
				btnStableState = currBtn;
				if (btnStableState == 0) {
					//button has been stable pressed
					if (select == 4 && save_no > 0) {
						if (save_no % 2 == 0) {
							EE_Write_Color((save_no + 1) / 2);
						} else {
							EE_Read_Color((save_no + 1) / 2);
						}
						save_no = 0;
					}
					select = (select + 1) % 5;
				}
			}
		}

		osDelay(DEBOUNCE_DELAY); // 2ms polling
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	dfu_boot_flag = (uint32_t*) (&_bflag); // set in linker script
	if (*dfu_boot_flag == BOOTLOADER_FLAG_VALUE) {
		*dfu_boot_flag = 0; // so next boot won't be affected
		/* Jump to system memory bootloader */
		JumpAddress = *(__IO uint32_t*) (BOOTLOADER_ADDRESS + 4);
		JumpToApplication = (pFunction) JumpAddress;
		JumpToApplication();
	}
	*dfu_boot_flag = 0; // So next boot won't be affected

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	/* Enable and set FLASH Interrupt priority */
	/* FLASH interrupt is used for the purpose of pages clean up under interrupt */
	HAL_NVIC_SetPriority(FLASH_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(FLASH_IRQn);

	/* Unlock the Flash Program Erase controller */
	HAL_FLASH_Unlock();
	/* Activate NMI generation when two errors are detected */
	__HAL_FLASH_ENABLE_IT(FLASH_IT_ECCC);

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_I2C2_Init();
	MX_TIM2_Init();
	MX_UCPD1_Init();
	MX_IWDG_Init();
	MX_CRC_Init();
	/* USER CODE BEGIN 2 */
	InitDisplay();
	ee_status = EE_Init(EE_FORCED_ERASE);
	if (ee_status != EE_OK) {
		displayEepromIssue();
	} else {
		EE_Read_Color(1);
	}

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();
	/* USBPD initialisation ---------------------------------*/
	MX_USBPD_Init();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	encoderTaskHandle = osThreadNew(EncoderTask, NULL, &encoderTask_attributes);

	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 8;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* Peripheral clock enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	LL_CRC_SetInputDataReverseMode(CRC, LL_CRC_INDATA_REVERSE_NONE);
	LL_CRC_SetOutputDataReverseMode(CRC, LL_CRC_OUTDATA_REVERSE_NONE);
	LL_CRC_SetPolynomialCoef(CRC, LL_CRC_DEFAULT_CRC32_POLY);
	LL_CRC_SetPolynomialSize(CRC, LL_CRC_POLYLENGTH_32B);
	LL_CRC_SetInitialData(CRC, LL_CRC_DEFAULT_CRC_INITVALUE);
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x00C12166;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void) {

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
	hiwdg.Init.Window = 4095;
	hiwdg.Init.Reload = 4095;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */
	__HAL_RCC_TIM2_CLK_ENABLE();

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 100;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief UCPD1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UCPD1_Init(void) {

	/* USER CODE BEGIN UCPD1_Init 0 */

	/* USER CODE END UCPD1_Init 0 */

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UCPD1);

	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	/**UCPD1 GPIO Configuration
	 PB15   ------> UCPD1_CC2
	 PA8   ------> UCPD1_CC1
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* UCPD1 DMA Init */

	/* UCPD1_RX Init */
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_UCPD1_RX);

	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1,
	LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);

	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);

	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);

	/* UCPD1_TX Init */
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_UCPD1_TX);

	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2,
	LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

	/* UCPD1 interrupt Init */
	NVIC_SetPriority(USB_UCPD1_2_IRQn, 3);
	NVIC_EnableIRQ(USB_UCPD1_2_IRQn);

	/* USER CODE BEGIN UCPD1_Init 1 */

	/* USER CODE END UCPD1_Init 1 */
	/* USER CODE BEGIN UCPD1_Init 2 */

	/* USER CODE END UCPD1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	NVIC_SetPriority(DMA1_Channel1_IRQn, 3);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel2_3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(DB_OUT_GPIO_Port, DB_OUT_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : R_SW_Pin R_Q_Pin R_I_Pin */
	GPIO_InitStruct.Pin = R_SW_Pin | R_Q_Pin | R_I_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : FLT_IN_Pin */
	GPIO_InitStruct.Pin = FLT_IN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(FLT_IN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DB_OUT_Pin */
	GPIO_InitStruct.Pin = DB_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DB_OUT_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	GPIO_InitStruct.Pin = PWM_R_Pin | PWM_G_Pin | PWM_B_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
	HAL_GPIO_Init(DB_OUT_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = SDA_Pin | SCL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF8_I2C2;
	HAL_GPIO_Init(SDA_Port, &GPIO_InitStruct);

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// restart into the bootloader by setting the correct option bytes and then trigger reset
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* init code for USB_Device */
	MX_USB_Device_Init();
	/* USER CODE BEGIN 5 */
	MX_TCPP_Init();
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	TIM2->ARR = 100;
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->CCR3 = 0;
	/* Infinite loop */

	for (;;) {
		HAL_IWDG_Refresh(&hiwdg);
		updateScreen();
		TIM2->CCR1 = on ? colors[0] : 0;
		TIM2->CCR2 = on ? colors[1] : 0;
		TIM2->CCR3 = on ? colors[2] : 0;
		osDelay(100);
	}
	/* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM3 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM3) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
