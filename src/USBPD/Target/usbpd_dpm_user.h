/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usbpd_dpm_user.h
 * @author  MCD Application Team
 * @brief   Header file for usbpd_dpm_user.c file
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

#ifndef __USBPD_DPM_USER_H_
#define __USBPD_DPM_USER_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Include */

/* USER CODE END Include */

/** @addtogroup STM32_USBPD_APPLICATION
 * @{
 */

/** @addtogroup STM32_USBPD_APPLICATION_DPM_USER
 * @{
 */

/* Exported typedef ----------------------------------------------------------*/
typedef struct {
	uint32_t PE_DataSwap :1U; /*!< support data swap                                     */
	uint32_t PE_VconnSwap :1U; /*!< support VCONN swap                                    */
	uint32_t PE_DR_Swap_To_DFP :1U; /*!< If supported, DR Swap to DFP can be accepted or not by the user else directly rejected */
	uint32_t PE_DR_Swap_To_UFP :1U; /*!< If supported, DR Swap to UFP can be accepted or not by the user else directly rejected */
	uint32_t Reserved1 :28U; /*!< Reserved bits */
	USBPD_SNKPowerRequest_TypeDef DPM_SNKRequestedPower; /*!< Requested Power by the sink board                     */
	USBPD_MIDB_TypeDef DPM_ManuInfoPort; /*!< Manufacturer information used for the port            */
	USBPD_SKEDB_TypeDef DPM_SNKExtendedCapa; /*!< SNK Extended Capability                             */
	uint16_t ReservedManu; /*!< Reserved bits to match with Manufacturer information            */
} USBPD_USER_SettingsTypeDef;

typedef struct {
	uint32_t XID; /*!< Value provided by the USB-IF assigned to the product   */
	uint16_t VID; /*!< Vendor ID (assigned by the USB-IF)                     */
	uint16_t PID; /*!< Product ID (assigned by the manufacturer)              */
} USBPD_IdSettingsTypeDef;
/* USER CODE BEGIN Typedef */

#if !defined(USBPD_REV_MAJOR)
#define USBPD_REV_MAJOR      (3U)    /* USBPD Specification revision major */
#define USBPD_REV_MINOR      (1U)    /* USBPD Specification revision minor */
#define USBPD_VERSION_MAJOR  (1U)    /* USBPD Specification version major  */
#define USBPD_VERSION_MINOR  (7U)    /* USBPD Specification version minor  */
#endif /* !USBPD_REV_MAJOR */

/** @brief  USBPD PDO Selection method enum definition
 *
 */
typedef enum {
	PDO_SEL_METHOD_MAX_PWR,
	PDO_SEL_METHOD_MIN_PWR,
	PDO_SEL_METHOD_MAX_VOLT,
	PDO_SEL_METHOD_MIN_VOLT,
	PDO_SEL_METHOD_MAX_CUR,
	PDO_SEL_METHOD_MIN_CUR
} USBPD_DPM_PDO_SelectionMethodTypeDef;

/**
 * @brief  USBPD DPM handle Structure definition
 * @{
 */
typedef struct {
	uint32_t DPM_ListOfRcvSNKPDO[USBPD_MAX_NB_PDO]; /*!< The list of received Sink Power Data Objects from Port partner (when Port partner is a Sink or a DRP port). */
	uint32_t DPM_NumberOfRcvSNKPDO; /*!< The number of received Sink Power Data Objects from port Partner (when Port partner is a Sink or a DRP port). */
	uint32_t DPM_ListOfRcvSRCPDO[USBPD_MAX_NB_PDO]; /*!< The list of received Source Power Data Objects from Port partner     */
	uint32_t DPM_NumberOfRcvSRCPDO; /*!< The number of received Source Power Data Objects from port Partner  (when Port partner is a Source or a DRP port). */
	uint32_t DPM_RcvRequestDOMsg; /*!< Received request Power Data Object message from the port Partner     */
	uint32_t DPM_RequestDOMsgPrevious; /*!< Previous Request Power Data Object message to be sent                */

	USBPD_PPSSDB_TypeDef DPM_RcvPPSStatus; /*!< PPS Status received by port partner                                  */
	USBPD_SKEDB_TypeDef DPM_RcvSNKExtendedCapa; /*!< SNK Extended Capability received by port partner                     */

	uint32_t DPM_RequestDOMsg; /*!< Request Power Data Object message to be sent                         */
	uint32_t DPM_RDOPosition; /*!< RDO Position of requested DO in Source list of capabilities          */
	uint32_t DPM_RDOPositionPrevious; /*!< RDO Position of previous requested DO in Source list of capabilities */
	uint32_t DPM_RequestedVoltage; /*!< Value of requested voltage                                           */
	uint32_t DPM_RequestedCurrent; /*!< Value of requested current                                           */
} USBPD_HandleTypeDef;

/* USER CODE END Typedef */

/* Exported define -----------------------------------------------------------*/
/* USER CODE BEGIN Define */

#define DPM_NO_SRC_PDO_FOUND   0xFFU /*!< No match found between Received SRC PDO and SNK capabilities */

/* USER CODE END Define */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN Constant */

/* USER CODE END Constant */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Exported variables --------------------------------------------------------*/
/* USER CODE BEGIN Private_Variables */

/* USER CODE END Private_Variables */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup USBPD_USER_EXPORTED_FUNCTIONS
 * @{
 */
/** @addtogroup USBPD_USER_EXPORTED_FUNCTIONS_GROUP1
 * @{
 */
USBPD_StatusTypeDef USBPD_DPM_UserInit(void);
void USBPD_DPM_WaitForTime(uint32_t Time);
void USBPD_DPM_UserCableDetection(uint8_t PortNum, USBPD_CAD_EVENT State);
void USBPD_DPM_UserTimerCounter(uint8_t PortNum);

/**
 * @}
 */

/** @addtogroup USBPD_USER_EXPORTED_FUNCTIONS_GROUP2
 * @{
 */
void USBPD_DPM_Notification(uint8_t PortNum,
		USBPD_NotifyEventValue_TypeDef EventVal);
void USBPD_DPM_HardReset(uint8_t PortNum,
		USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_HR_Status_TypeDef Status);
void USBPD_DPM_ExtendedMessageReceived(uint8_t PortNum,
		USBPD_ExtendedMsg_TypeDef MsgType, uint8_t *ptrData, uint16_t DataSize);
void USBPD_DPM_GetDataInfo(uint8_t PortNum,
		USBPD_CORE_DataInfoType_TypeDef DataId, uint8_t *Ptr, uint32_t *Size);
void USBPD_DPM_SetDataInfo(uint8_t PortNum,
		USBPD_CORE_DataInfoType_TypeDef DataId, uint8_t *Ptr, uint32_t Size);
void USBPD_DPM_SNK_EvaluateCapabilities(uint8_t PortNum,
		uint32_t *PtrRequestData,
		USBPD_CORE_PDO_Type_TypeDef *PtrPowerObjectType);
uint32_t USBPD_DPM_SNK_EvaluateMatchWithSRCPDO(uint8_t PortNum, uint32_t SrcPDO,
		uint32_t *PtrRequestedVoltage, uint32_t *PtrRequestedPower);
USBPD_StatusTypeDef USBPD_DPM_EvaluateVconnSwap(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_PE_VconnPwr(uint8_t PortNum,
		USBPD_FunctionalState State);
void USBPD_DPM_EnterErrorRecovery(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_EvaluateDataRoleSwap(uint8_t PortNum);
USBPD_FunctionalState USBPD_DPM_IsPowerReady(uint8_t PortNum,
		USBPD_VSAFE_StatusTypeDef Vsafe);

/**
 * @}
 */

/** @addtogroup USBPD_USER_EXPORTED_FUNCTIONS_GROUP3
 * @{
 */
USBPD_StatusTypeDef USBPD_DPM_RequestHardReset(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestCableReset(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestGotoMin(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestPing(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestMessageRequest(uint8_t PortNum,
		uint8_t IndexSrcPDO, uint16_t RequestedVoltage);
USBPD_StatusTypeDef USBPD_DPM_RequestGetSourceCapability(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestGetSinkCapability(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestDataRoleSwap(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestPowerRoleSwap(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestVconnSwap(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestSoftReset(uint8_t PortNum,
		USBPD_SOPType_TypeDef SOPType);
USBPD_StatusTypeDef USBPD_DPM_RequestSourceCapability(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestVDM_DiscoveryIdentify(uint8_t PortNum,
		USBPD_SOPType_TypeDef SOPType);
USBPD_StatusTypeDef USBPD_DPM_RequestVDM_DiscoverySVID(uint8_t PortNum,
		USBPD_SOPType_TypeDef SOPType);
USBPD_StatusTypeDef USBPD_DPM_RequestVDM_DiscoveryMode(uint8_t PortNum,
		USBPD_SOPType_TypeDef SOPType, uint16_t SVID);
USBPD_StatusTypeDef USBPD_DPM_RequestVDM_EnterMode(uint8_t PortNum,
		USBPD_SOPType_TypeDef SOPType, uint16_t SVID, uint8_t ModeIndex);
USBPD_StatusTypeDef USBPD_DPM_RequestVDM_ExitMode(uint8_t PortNum,
		USBPD_SOPType_TypeDef SOPType, uint16_t SVID, uint8_t ModeIndex);
USBPD_StatusTypeDef USBPD_DPM_RequestDisplayPortStatus(uint8_t PortNum,
		USBPD_SOPType_TypeDef SOPType, uint16_t SVID, uint32_t *pDPStatus);
USBPD_StatusTypeDef USBPD_DPM_RequestDisplayPortConfig(uint8_t PortNum,
		USBPD_SOPType_TypeDef SOPType, uint16_t SVID, uint32_t *pDPConfig);
USBPD_StatusTypeDef USBPD_DPM_RequestAttention(uint8_t PortNum,
		USBPD_SOPType_TypeDef SOPType, uint16_t SVID);
USBPD_StatusTypeDef USBPD_DPM_RequestAlert(uint8_t PortNum,
		USBPD_ADO_TypeDef Alert);
USBPD_StatusTypeDef USBPD_DPM_RequestGetSourceCapabilityExt(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestGetSinkCapabilityExt(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestGetManufacturerInfo(uint8_t PortNum,
		USBPD_SOPType_TypeDef SOPType, uint8_t *pManuInfoData);
USBPD_StatusTypeDef USBPD_DPM_RequestGetStatus(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestFastRoleSwap(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestGetPPS_Status(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestGetCountryCodes(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestGetCountryInfo(uint8_t PortNum,
		uint16_t CountryCode);
USBPD_StatusTypeDef USBPD_DPM_RequestGetBatteryCapability(uint8_t PortNum,
		uint8_t *pBatteryCapRef);
USBPD_StatusTypeDef USBPD_DPM_RequestGetBatteryStatus(uint8_t PortNum,
		uint8_t *pBatteryStatusRef);
USBPD_StatusTypeDef USBPD_DPM_RequestSecurityRequest(uint8_t PortNum);
/* USER CODE BEGIN Function */

/* USER CODE END Function */
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

#endif /* __USBPD_DPM_USER_H_ */

