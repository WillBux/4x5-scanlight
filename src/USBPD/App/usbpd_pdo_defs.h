/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usbpd_pdo_defs.h
 * @author  MCD Application Team
 * @brief   Header file for definition of PDO/APDO values for 2 ports(DRP/SNK) configuration
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

#ifndef __USBPD_PDO_DEF_H_
#define __USBPD_PDO_DEF_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbpd_def.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Define   ------------------------------------------------------------------*/
#define PORT0_NB_SOURCEPDO         0U   /* Number of Source PDOs (applicable for port 0)   */
#define PORT0_NB_SINKPDO           1U   /* Number of Sink PDOs (applicable for port 0)     */
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Exported typedef ----------------------------------------------------------*/
/* USER CODE BEGIN typedef */
/**
 * @brief  USBPD Port PDO Structure definition
 *
 */

/**
 * @brief  USBPD Port PDO Structure definition
 */
typedef struct {
	uint32_t *ListOfPDO; /*!< Pointer on Power Data Objects list, defining port capabilities */
	uint8_t *NumberOfPDO; /*!< Number of Power Data Objects defined in ListOfPDO
	 This parameter must be set at max to @ref USBPD_MAX_NB_PDO value */
} USBPD_PortPDO_TypeDef;

/**
 * @brief  USBPD Port PDO Storage Structure definition
 */

typedef struct {
	USBPD_PortPDO_TypeDef SourcePDO; /*!< SRC Power Data Objects */
	USBPD_PortPDO_TypeDef SinkPDO; /*!< SNK Power Data Objects */

} USBPD_PWR_Port_PDO_Storage_TypeDef;
/* USER CODE END typedef */

/* Exported define -----------------------------------------------------------*/
/* USER CODE BEGIN Exported_Define */

#define USBPD_CORE_PDO_SRC_FIXED_MAX_CURRENT 3
#define USBPD_CORE_PDO_SNK_FIXED_MAX_CURRENT 1500

/* USER CODE END Exported_Define */

/* Exported constants --------------------------------------------------------*/

/* USER CODE BEGIN constants */

/* USER CODE END constants */

/* Exported macro ------------------------------------------------------------*/

/* USER CODE BEGIN macro */

/* USER CODE END macro */

/* Exported variables --------------------------------------------------------*/
/* USER CODE BEGIN variables */

#ifndef _GUI_INTERFACE
#ifndef __USBPD_PWR_IF_C
extern uint8_t USBPD_NbPDO[4];
extern uint32_t PORT0_PDO_ListSRC[USBPD_MAX_NB_PDO];
extern uint32_t PORT0_PDO_ListSNK[USBPD_MAX_NB_PDO];
#else /* __USBPD_PWR_IF_C */
uint8_t USBPD_NbPDO[4] = {(PORT0_NB_SINKPDO),
                          (PORT0_NB_SOURCEPDO)};
#endif /* __USBPD_PWR_IF_C */
#endif /* _GUI_INTERFACE */

/* USER CODE END variables */

#ifndef __USBPD_PWR_IF_C
extern uint32_t PORT0_PDO_ListSRC[USBPD_MAX_NB_PDO];
extern uint32_t PORT0_PDO_ListSNK[USBPD_MAX_NB_PDO];
#else
/* Definition of Source PDO for Port 0 */
uint32_t PORT0_PDO_ListSRC[USBPD_MAX_NB_PDO] =
{
  /* PDO 1 */ (0x00000000U),
  /* PDO 2 */ (0x00000000U),
  /* PDO 3 */ (0x00000000U),
  /* PDO 4 */ (0x00000000U),
  /* PDO 5 */ (0x00000000U),
  /* PDO 6 */ (0x00000000U),
  /* PDO 7 */ (0x00000000U),
};

/* Definition of Sink PDO for Port 0 */
uint32_t PORT0_PDO_ListSNK[USBPD_MAX_NB_PDO] =
{
  /* PDO 1 */
  (
    USBPD_PDO_TYPE_FIXED                 | /* Fixed supply PDO            */

    USBPD_PDO_SNK_FIXED_SET_VOLTAGE(20000U)         | /* Voltage in mV               */
    USBPD_PDO_SNK_FIXED_SET_OP_CURRENT(1500U)     | /* Operating current in  mA            */

    /* Common definitions applicable to all PDOs, defined only in PDO 1 */
    USBPD_PDO_SNK_FIXED_FRS_NOT_SUPPORTED          | /* Fast Role Swap				 */
    USBPD_PDO_SNK_FIXED_DRD_SUPPORTED          | /* Dual-Role Data              */
    USBPD_PDO_SNK_FIXED_USBCOMM_SUPPORTED      | /* USB Communications          */
    USBPD_PDO_SNK_FIXED_EXT_POWER_NOT_AVAILABLE    | /* External Power              */
    USBPD_PDO_SNK_FIXED_HIGHERCAPAB_NOT_SUPPORTED   | /* Higher Capability           */
    USBPD_PDO_SNK_FIXED_DRP_NOT_SUPPORTED            /* Dual-Role Power             */
  ),
  /* PDO 2 */ (0x00000000U),
  /* PDO 3 */ (0x00000000U),
  /* PDO 4 */ (0x00000000U),
  /* PDO 5 */ (0x00000000U),
  /* PDO 6 */ (0x00000000U),
  /* PDO 7 */ (0x00000000U),
};

#endif

/* Exported functions --------------------------------------------------------*/

/* USER CODE BEGIN functions */

/* USER CODE END functions */

#ifdef __cplusplus
}
#endif

#endif /* __USBPD_PDO_DEF_H_ */
