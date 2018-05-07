/*
 * board_v1_2.h
 *
 *  Created on: 2018/04/18
 *      Author: Yuki Kusakabe
 */

#ifndef BOARD_V1_2_H_
#define BOARD_V1_2_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stspin32f0.h"
#include "gpio.h"

#define AS5048A_GPIO_Port   GPIOA
#define AS5048A_GPIO_Pin    LL_GPIO_PIN_4
#define RS485_RTS_GPIO_Port GPIOB
#define RS485_RTS_GPIO_Pin  LL_GPIO_PIN_1
#define LED_GPIO_Port       GPIOB
#define LED_GPIO_Pin        LL_GPIO_PIN_7

typedef enum {
  RS485_RECEIVE = 0,
  RS485_TRANSMIT = 1,
}rs485transceiverState_t;


void RS485_SetState(rs485transceiverState_t state);
void LED_OFF(void);
void LED_ON(void);
void LED_TOGGLE(void);
uint8_t GetHallState(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_V1_2_H_ */
