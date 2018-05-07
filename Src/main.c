
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "board_v1_2.h"
#include "command.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const uint8_t MyID = MD_4;
bool command_update = false;
int32_t Get_Ref_RPM = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM17_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  //LL_USART_EnableIT_RXNE(USART1);
  RingBuf_Init();
  printf("Build: %s %s\r\n",__DATE__,__TIME__);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  Hallsensor_Activate();
  Activate_ADC();
  Activate_USART_RX();
  
  Overcurrent_Selection(BSP_SIP_SEL_VIS_FROM_MCU_AND_GATE_LOGIC);
  Overcurrent_Threshold_Setvalue(BSP_SIP_OC_TH_500mV);
  MC_Init();
  LL_TIM_EnableIT_UPDATE(TIM17);
  LL_TIM_EnableCounter(TIM17);

  LL_DBGMCU_EnableDBGStopMode();
  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_APB1_GRP1_TIM2_STOP);
  LL_DBGMCU_APB1_GRP2_FreezePeriph(LL_DBGMCU_APB1_GRP2_TIM1_STOP);
  LL_DBGMCU_APB1_GRP2_FreezePeriph(LL_DBGMCU_APB1_GRP2_TIM17_STOP);
  

  uint8_t rxbuf[64];
  uint8_t *p_RxDatalen;
  uint8_t *p_addr;
  //uint8_t c = 0;
  p_RxDatalen = &rxbuf[3];
  p_addr = &rxbuf[2];
  uint8_t parity = 0x00;

  RS485_SetState(RS485_RECEIVE);

  while (1)
  {

  /* USER CODE END WHILE */
/*
 * Packet Structure
 * 0xFA : Header 1
 * rxbuf[0]         0xAF   : Header 2
 * rxbuf[1]         ID     : Device ID
 * rxbuf[2]         addr   : Address
 * rxbuf[3]         len(n) : Data length n byte
 * rxbuf[4]         data_1 : Data_1
 *    |
 * rxbuf[4 + (n-1)] data_n : Data_n
 * rxbuf[4 + n]     parity : XOR ID ~ data_n
 */

  /* USER CODE BEGIN 3 */
    LL_mDelay(0);
    if(Uart_read_byte() == HEAD_1){   //最初のヘッダが来るまで読んでは捨てのポーリング
      Uart_read(rxbuf,2);
      if(rxbuf[0] == HEAD_2 && rxbuf[1] == MyID){   //0xFAの次が0xAFかつ自分のIDなら以降の受信処理を実行する。
        Uart_read(&rxbuf[2],2);
        Uart_read(&rxbuf[4],*p_RxDatalen + 1);  //以降のデータを一気に読み出す
        parity = 0x00;
        for(uint8_t i = 1; i < (4 + *p_RxDatalen); i++){
          parity ^= rxbuf[i];
        }
        if(rxbuf[(4 + *p_RxDatalen)] == parity){  //パリティチェック
          LED_TOGGLE(); //Packet Receive Complete
          //Get_Ref_RPM = (rxbuf[4] << 24) | (rxbuf[5] << 16) | (rxbuf[6] << 8) | (rxbuf[7] << 0);
          Get_Ref_RPM = *(int32_t *)(rxbuf+4);
          command_update = true;
        }
      }
    }
    //LED_TOGGLE();
    //printf("%6d\r\n",Mech_Speed_RPM);
  }
  /* USER CODE END 3 */

}

static void LL_Init(void)
{
  

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

  /* System interrupt init*/
  /* SVC_IRQn interrupt configuration */
  NVIC_SetPriority(SVC_IRQn, 0);
  /* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn, 0);
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 0);

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  Error_Handler();  
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);

  LL_RCC_HSI14_Enable();

   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI14_SetCalibTrimming(16);

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);

  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(48000000);

  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);

  LL_SetSystemCoreClock(48000000);

  LL_RCC_HSI14_EnableADCControl();

  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
