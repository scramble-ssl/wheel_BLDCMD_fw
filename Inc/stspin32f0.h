/*
 * stspin32f0.h
 *
 *  Created on: 2018/04/18
 *      Author: Yuki Kusakabe
 */

#ifndef STSPIN32F0_H_
#define STSPIN32F0_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include "gpio.h"

#define OC_SEL_Pin LL_GPIO_PIN_11
#define OC_SEL_GPIO_Port GPIOA
#define OC_Detect_Pin LL_GPIO_PIN_12
#define OC_Detect_GPIO_Port GPIOB
#define OC_Detect_EXTI_IRQn EXTI4_15_IRQn
#define OC_TH_STBY2_Pin LL_GPIO_PIN_6
#define OC_TH_STBY2_GPIO_Port GPIOF
#define OC_TH_STBY1_Pin LL_GPIO_PIN_7
#define OC_TH_STBY1_GPIO_Port GPIOF

/* Exported types ------------------------------------------------------------*/

/** @defgroup STSPIN32F0_SIP_Exported_Types STSPIN32F0 SIP Exported Types
  * @{
  */
typedef enum {
  BSP_SIP_SEL_VIS_FROM_MCU = 0,
  BSP_SIP_SEL_VIS_FROM_MCU_AND_GATE_LOGIC = 1,
} overcurrentSelectionVisibility_t;

typedef enum {
  BSP_SIP_OC_TH_STBY  = 0,
  BSP_SIP_OC_TH_100mV = 1,
  BSP_SIP_OC_TH_250mV = 2,
  BSP_SIP_OC_TH_500mV = 3
} overcurrentThresholds_t;

/**
  * @} end STSPIN32F0 SIP Exported Types
  */

/* Exported Functions  -------------------------------------------------------*/
/** @defgroup STSPIN32F0_SIP_Exported_Functions STSPIN32F0 SIP Exported Functions
  * @{
  */
void Overcurrent_Selection(overcurrentSelectionVisibility_t ocSel);
void Overcurrent_Threshold_Setvalue(overcurrentThresholds_t ocThres);

/**
  * @} STSPIN32F0 SIP Exported Functions
  */

#ifdef __cplusplus
}
#endif

#endif /* STSPIN32F0_H_ */
