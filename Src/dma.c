/**
  ******************************************************************************
  * File Name          : dma.c
  * Description        : This file provides code for the configuration
  *                      of all the requested memory to memory DMA transfers.
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
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/* USER CODE BEGIN 2 */
void Configure_DMA_ADC(uint32_t *DMATransferData, uint32_t TransferDataSIZE){

  /* Set DMA transfer addresses of source and destination */
  LL_DMA_ConfigAddresses(DMA1,
                         LL_DMA_CHANNEL_1,
                         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                         (uint32_t)DMATransferData,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  /* Set DMA transfer size */
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, TransferDataSIZE);

  /* Enable DMA transfer interruption: transfer complete */
  //LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

  /* Enable DMA transfer interruption: half transfer */
  //LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);

  /* Enable DMA transfer interruption: transfer error */
  //LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);

  /*## Activation of DMA ##*/
  /* Enable the DMA transfer */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

void Configure_DMA_USART1_TX(uint32_t *DMATransferData, uint32_t TransferDataSIZE){

  /* Set DMA transfer addresses of source and destination */
  LL_DMA_ConfigAddresses(DMA1,
                         LL_DMA_CHANNEL_2,
                         (uint32_t)DMATransferData,
                         LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT),
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2));
  /* Set DMA transfer size */
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, TransferDataSIZE);

  /* Enable DMA transfer interruption: transfer complete */
  //LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);

  /* Enable DMA transfer interruption: half transfer */
  //LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_2);

  /* Enable DMA transfer interruption: transfer error */
  //LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);

  /*## Activation of DMA ##*/
  /* Enable the DMA transfer */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
}

void Configure_DMA_USART1_RX(uint32_t *DMATransferData, uint32_t TransferDataSIZE){

  /* Set DMA transfer addresses of source and destination */
  LL_DMA_ConfigAddresses(DMA1,
                         LL_DMA_CHANNEL_3,
                         LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_RECEIVE),
                         (uint32_t)DMATransferData,
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
  /* Set DMA transfer size */
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, TransferDataSIZE);

  /* Enable DMA transfer interruption: transfer complete */
  //LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);

  /* Enable DMA transfer interruption: half transfer */
  //LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_3);

  /* Enable DMA transfer interruption: transfer error */
  //LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);

  /*## Activation of DMA ##*/
  LL_USART_EnableDMAReq_RX(USART1);
  /* Enable the DMA transfer */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);

}
/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
