/*
 * board_v1_2.c
 *
 *  Created on: 2018/04/18
 *      Author: Yuki Kusakabe
 */

#include "board_v1_2.h"

void RS485_SetState(rs485transceiverState_t state){

  if(state){
      LL_GPIO_SetOutputPin(RS485_RTS_GPIO_Port, RS485_RTS_GPIO_Pin);
  }else{
      LL_GPIO_ResetOutputPin(RS485_RTS_GPIO_Port, RS485_RTS_GPIO_Pin);
  }
}

/******************************************************//**
 * @brief     Turns selected LED On
 * @retval    None
 **********************************************************/
void LED_OFF(void){

  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_GPIO_Pin);
}

/******************************************************//**
 * @brief     Turns selected LED On
 * @retval    None
 **********************************************************/
void LED_ON(void){

  LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_GPIO_Pin);
}

/******************************************************//**
 * @brief     Turns selected LED Toggle
 * @retval    None
 **********************************************************/
void LED_TOGGLE(void){

  LL_GPIO_TogglePin(LED_GPIO_Port, LED_GPIO_Pin);
}

uint8_t GetHallState(void){
  uint16_t H1, H2, H3;

  H1 = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0);
  H2 = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_1);
  H3 = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_2);
  return ( (H1 << 2) | (H2 << 1) | H3 );
}
