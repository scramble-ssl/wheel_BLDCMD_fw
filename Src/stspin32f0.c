/*
 * stspin32f0.c
 *
 *  Created on: 2018/04/18
 *      Author: Yuki Kusakabe
 */

#include "stspin32f0.h"

/******************************************************//**
  * @brief  Select the overcurrent comparator output signal visibility
  * (0) Visible only to MCU
  * (1) Visible to MCU and acts on gate driver logic
 * @param[in] ocSel
  * @retval None
 **********************************************************/
void Overcurrent_Selection(overcurrentSelectionVisibility_t ocSel){

  if(ocSel){
      LL_GPIO_SetOutputPin(OC_SEL_GPIO_Port, OC_SEL_Pin);
  }else{
      LL_GPIO_ResetOutputPin(OC_SEL_GPIO_Port, OC_SEL_Pin);
  }
}

/******************************************************//**
  * @brief  Set the overcurrent threshold value and commands standby mode
  * (0) Standy mode
  * (1) 100mV
  * (2) 250mV
  * (3) 500mV
 * @param[in] ocThres
  * @retval None
 **********************************************************/
void Overcurrent_Threshold_Setvalue(overcurrentThresholds_t ocThres){

  switch(ocThres){
    case BSP_SIP_OC_TH_STBY :
      LL_GPIO_ResetOutputPin(OC_TH_STBY1_GPIO_Port, OC_TH_STBY1_Pin);
      LL_GPIO_ResetOutputPin(OC_TH_STBY2_GPIO_Port, OC_TH_STBY2_Pin);
      break;
    case BSP_SIP_OC_TH_100mV :
      LL_GPIO_SetOutputPin(OC_TH_STBY1_GPIO_Port, OC_TH_STBY1_Pin);
      LL_GPIO_ResetOutputPin(OC_TH_STBY2_GPIO_Port, OC_TH_STBY2_Pin);
      break;
    case BSP_SIP_OC_TH_250mV :
      LL_GPIO_ResetOutputPin(OC_TH_STBY1_GPIO_Port, OC_TH_STBY1_Pin);
      LL_GPIO_SetOutputPin(OC_TH_STBY2_GPIO_Port, OC_TH_STBY2_Pin);
      break;
    case BSP_SIP_OC_TH_500mV :
      LL_GPIO_SetOutputPin(OC_TH_STBY1_GPIO_Port, OC_TH_STBY1_Pin);
      LL_GPIO_SetOutputPin(OC_TH_STBY2_GPIO_Port, OC_TH_STBY2_Pin);
      break;
    default :
      break;
  }
}
