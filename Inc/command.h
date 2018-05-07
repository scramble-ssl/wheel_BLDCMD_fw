/*
 * command.h
 *
 *  Created on: 2018/04/28
 *      Author: Yuki Kusakabe
 */

#ifndef INC_COMMAND_H_
#define INC_COMMAND_H_

typedef enum{
  HEAD_1 = 0xFA,
  HEAD_2 = 0xAF
}Header_Typedef;

typedef enum{
  MD_1 = 0x10,
  MD_2 = 0x11,
  MD_3 = 0x12,
  MD_4 = 0x13,
  SO_1 = 0x20
}DeviceID_Typedef;

#endif /* INC_COMMAND_H_ */
