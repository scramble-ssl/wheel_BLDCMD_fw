/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "adc.h"
#include "usart.h"
#include "board_v1_2.h"
#include "main.h"
int32_t Mech_Speed_RPM = 0;                         /*!<  Mechanical motor speed */
LL_TIM_HALLSENSOR_InitTypeDef   HallSensorInitStruct;
int32_t Ref_RPM = 0;
#define Kp_Gain 500
#define Ki_Gain 0
#define Kd_Gain 20
#define Kp_Scall  12
#define Ki_Scall  16
#define Kd_Scall  16
int32_t wIntegral_sum = 0;
int16_t pulse = 0;
int32_t wOutput_32 = 0;
#define set_CCR1(val)   TIM1->CCR1 = val
#define set_CCR2(val)   TIM1->CCR2 = val
#define set_CCR3(val)   TIM1->CCR3 = val

uint32_t hall_capture = 0;                 /*!< Input capture register value when hall status changes */

#define abs(x) ((x) < 0 ? -(x) : (x))

#define FILTER_DEEP_SHIFT                    4
#define FILTER_DEEP     (1<<FILTER_DEEP_SHIFT)     /*!< Number of bits for digital filter */

//static uint16_t index = 0;
int16_t vel_tmp[FILTER_DEEP] = {0,0,0,0};

int32_t Error = 0;
int32_t Error_z = 0;


/* USER CODE END 0 */

/* TIM1 init function */
void MX_TIM1_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct;
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;
  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0);
  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);

  TIM_InitStruct.Prescaler = TIM1_Prescaler;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = TIM1_Period;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);

  LL_TIM_EnableARRPreload(TIM1);

  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);

  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);

  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);

  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);

  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);

  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);

  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);

  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);

  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);

  LL_TIM_SetTriggerInput(TIM1, LL_TIM_TS_ITR1);

  LL_TIM_SetSlaveMode(TIM1, LL_TIM_SLAVEMODE_DISABLED);

  LL_TIM_DisableIT_TRIG(TIM1);

  LL_TIM_DisableDMAReq_TRIG(TIM1);

  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);

  LL_TIM_DisableMasterSlaveMode(TIM1);

  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 10;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_ENABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_ENABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);

    /**TIM1 GPIO Configuration    
    PB13     ------> TIM1_CH1N
    PB14     ------> TIM1_CH2N
    PB15     ------> TIM1_CH3N
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    PA10     ------> TIM1_CH3 
    */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}
/* TIM2 init function */
void MX_TIM2_Init(void)
{
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;
  LL_TIM_InitTypeDef TIM_InitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;
  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
  
  /**TIM2 GPIO Configuration  
  PA0   ------> TIM2_CH1
  PA1   ------> TIM2_CH2
  PA2   ------> TIM2_CH3 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, 0);
  NVIC_EnableIRQ(TIM2_IRQn);

  LL_TIM_CC_DisableChannel(TIM2, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);

  LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_TRC);

  LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);

  LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);

  LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);

  LL_TIM_IC_EnableXORCombination(TIM2);

  LL_TIM_SetTriggerInput(TIM2, LL_TIM_TS_TI1F_ED);

  LL_TIM_SetSlaveMode(TIM2, LL_TIM_SLAVEMODE_RESET);

  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;

  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);

  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH2);

  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 4294967295;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);

  LL_TIM_DisableARRPreload(TIM2);

  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_OC2REF);

  LL_TIM_DisableMasterSlaveMode(TIM2);

}
/* TIM16 init function */
void MX_TIM16_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct;

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM16);

  /* TIM16 interrupt Init */
  NVIC_SetPriority(TIM16_IRQn, 1);
  NVIC_EnableIRQ(TIM16_IRQn);

  TIM_InitStruct.Prescaler = TIM16_Prescaler;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = TIM16_Period;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM16, &TIM_InitStruct);

  LL_TIM_DisableARRPreload(TIM16);

}
/* TIM17 init function */
void MX_TIM17_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct;

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM17);

  /* TIM17 interrupt Init */
  NVIC_SetPriority(TIM17_IRQn, 0);
  NVIC_EnableIRQ(TIM17_IRQn);

  TIM_InitStruct.Prescaler = TIM17_Prescaler;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = TIM17_Period;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM17, &TIM_InitStruct);

  LL_TIM_DisableARRPreload(TIM17);

}

/* USER CODE BEGIN 1 */

void Hallsensor_Activate(void){

  LL_TIM_HALLSENSOR_StructInit(&HallSensorInitStruct);
  //Set Commutation Delay
  HallSensorInitStruct.CommutationDelay = 1;

  LL_TIM_HALLSENSOR_Init(TIM2, &HallSensorInitStruct);

  //Enable Capture Compare ISR
  LL_TIM_EnableIT_CC1(TIM2);
  LL_TIM_EnableIT_CC2(TIM2);
  //
  LL_TIM_EnableCounter(TIM2);
}

void MC_Init(void){
  /*LL_TIM_CC_EnableChannel(TIM1,
                          LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
                          LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
                          LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N
                          );*/
  LL_TIM_EnableAllOutputs(TIM1);
  //Enable Commutation ISR
  /*
  LL_TIM_CC_EnablePreload(TIM1);
  LL_TIM_CC_SetUpdate(TIM1, LL_TIM_CCUPDATESOURCE_COMG_AND_TRGI);
  LL_TIM_EnableIT_COM(TIM1);
  */
  //
  LL_TIM_EnableCounter(TIM1);
}

void TIM1Break_Callbck(void){

}

void TIM1Commutation_Callback(void){
  static uint8_t i = 0;
  i++;
  printf("com %3d\n",i);
}

void TIM2CC1_Callback(void){
  /*
  static uint8_t i = 0;
  i++;
  */
  hall_capture = LL_TIM_OC_GetCompareCH1(TIM2);
  //Mech_Speed_RPM = MC_GetMechSpeedRPM();
  //LED_TOGGLE();
  //printf("cc1 %10d\n",hall_capture);
  //printf("%10d\n",Mech_Speed_RPM);
}

void TIM2CC2_Callback(void){
  /*
  static uint8_t i = 0;
  i++;
  printf("cc2 %3d\n",i);
  */
  //Mech_Speed_RPM = MC_GetMechSpeedRPM();
  //printf("%10d\n",Mech_Speed_RPM);
  uint8_t state = GetHallState();
  //int16_t pulse = 100;

  if(pulse > 0){
    //uint16_t pwm_pulse = -pulse;
    switch(state){
    case 4:
      TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
      TIM1->CCR1 = 0;
      TIM1->CCR2 = 0;
      TIM1->CCR3 = pulse;
      break;
    case 6:
      TIM1->CCER = TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
      TIM1->CCR1 = 0;
      TIM1->CCR2 = 0;
      TIM1->CCR3 = pulse;
     break;
    case 2:
      TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE;
      TIM1->CCR1 = pulse;
      TIM1->CCR2 = 0;
      TIM1->CCR3 = 0;
      break;
    case 3 :
      TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
      TIM1->CCR1 = pulse;
      TIM1->CCR2 = 0;
      TIM1->CCR3 = 0;
      break;
    case 1:
      TIM1->CCER = TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
      TIM1->CCR1 = 0;
      TIM1->CCR2 = pulse;
      TIM1->CCR3 = 0;
      break;
    case 5:
      TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE;
      TIM1->CCR1 = 0;
      TIM1->CCR2 = pulse;
      TIM1->CCR3 = 0;
      break;
    default :
      break;
    }  //end switch()
    Mech_Speed_RPM = MC_GetMechSpeedRPM();
  }else{
    // pulse <= 0
    switch(state){
    case 2:
      TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE;
      TIM1->CCR1 = 0;
      TIM1->CCR2 = -pulse;
      TIM1->CCR3 = 0;
      break;
    case 3:
      TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
      TIM1->CCR1 = 0;
      TIM1->CCR2 = 0;
      TIM1->CCR3 = -pulse;
      break;
    case 1:
      TIM1->CCER = TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
      TIM1->CCR1 = 0;
      TIM1->CCR2 = 0;
      TIM1->CCR3 = -pulse;
      break;
    case 5 :
      TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE;
      TIM1->CCR1 = -pulse;
      TIM1->CCR2 = 0;
      TIM1->CCR3 = 0;
      break;
    case 4:
      TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
      TIM1->CCR1 = -pulse;
      TIM1->CCR2 = 0;
      TIM1->CCR3 = 0;
      break;
    case 6:
      TIM1->CCER = TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
      TIM1->CCR1 = 0;
      TIM1->CCR2 = -pulse;
      TIM1->CCR3 = 0;
      break;
    default :
      break;
    } //end switch()
    Mech_Speed_RPM = -MC_GetMechSpeedRPM();
  } //end if(pulse > 0)
} //end TIM2CC2_Callback()

void TIM17Update_Callback(void){
  static uint16_t angle = 0, angle_z1 = 0;
  int16_t velocity = 0;
  int32_t rpm = 0;
  static uint8_t heatbeat = 0;
  //printf("%6ld\n",Mech_Speed_RPM);
  /*
  angle = as5048a_GetAngle();
  velocity = angle - angle_z1;

  if(velocity < -COUNT_LIM){
    velocity += (RESOLUTION -1);
  }else if(velocity > COUNT_LIM){
    velocity -= (RESOLUTION-1);
  }

  vel_tmp[index] = velocity;

  int32_t velocity_tmp_sum = 0;
  //Sinple Moving Average(SMA)
  for (uint16_t i = 0 ; i < FILTER_DEEP -1 ; i++){
    velocity_tmp_sum += (int32_t)vel_tmp[i];

  }
  index++;
  if(index >= FILTER_DEEP)index = 0;
  velocity = velocity_tmp_sum >> FILTER_DEEP_SHIFT;
  rpm = (int32_t)velocity * 60 * 1000 / RESOLUTION;
*/

  heatbeat++;
  if(command_update == true){
    Ref_RPM = -Get_Ref_RPM;
    heatbeat = 0;
    command_update = false;
    LL_TIM_GenerateEvent_CC2(TIM2);
  }
  if(command_update == false && heatbeat >= 250){
    Ref_RPM = 0;
    heatbeat = 0;
  }
  //PI func
  int32_t wProportional_Term=0, wIntegral_Term=0, wDifferentinal_Term = 0;
  //Error = Ref_RPM - rpm;
  Error = Ref_RPM - Mech_Speed_RPM;
  //printf("%6d\n",Get_Ref_RPM);
  //printf("%6d\n",ADCBuffer[0]);

  wProportional_Term = Kp_Gain * Error;

  if(abs(wOutput_32) < TIM1_Period){
    wIntegral_sum += Error;
    wIntegral_Term = Ki_Gain * wIntegral_sum;
  }
  wDifferentinal_Term = Kd_Gain * (Error - Error_z);


  wOutput_32 = (wProportional_Term >> Kp_Scall) + (wIntegral_Term >> Ki_Scall) + (wDifferentinal_Term >> Kd_Scall);
  if(wOutput_32 > (1200-1)){
    wOutput_32 = 1199;
  }
  if(wOutput_32 < -(1200-1)){
    wOutput_32 = -1199;
  }
  pulse = wOutput_32;
  angle_z1 = angle;
  Error_z = Error;
}

/** @defgroup MC_GetMechSpeedRPM    MC_GetMechSpeedRPM
  *  @{
  * @brief Get the Mechanical Motor Speed (RPM)
  * @retval int32_t Return the mechanical motor speed (RPM)
*/
int32_t MC_GetMechSpeedRPM(void){

  uint32_t mech_rpm = (uint32_t)((48000000*10)/(1*8*hall_capture));
  return mech_rpm;
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
