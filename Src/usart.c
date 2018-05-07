/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#include "usart.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */
static USART_Buffer_t* pUSART_Buf;
USART_Buffer_t USARTx_Buf;
uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
static size_t rx_buffer_tail;
/* USER CODE END 0 */

/* USART1 init function */

void MX_USART1_UART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;
  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
  
  /**USART1 GPIO Configuration  
  PA15   ------> USART1_RX
  PB6   ------> USART1_TX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART1 DMA Init */
  
  /* USART1_RX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

  /* USART1_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, 0);
  NVIC_EnableIRQ(USART1_IRQn);

  USART_InitStruct.BaudRate = Baudrate;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);

  LL_USART_DisableIT_CTS(USART1);

  LL_USART_DisableOverrunDetect(USART1);

  LL_USART_ConfigAsyncMode(USART1);

  LL_USART_Enable(USART1);

}

/* USER CODE BEGIN 1 */

void Activate_USART_RX(void){
  Configure_DMA_USART1_RX(rx_buffer, UART_RX_BUFFER_SIZE);

}

size_t Uart_rcv_size(void){
  size_t size;

  // DMAが転送したデータ数を取得
  const size_t dma_cnt = UART_RX_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3);
  if (rx_buffer_tail <= dma_cnt) size = dma_cnt - rx_buffer_tail;
  else size = dma_cnt + (UART_RX_BUFFER_SIZE - rx_buffer_tail);

  return size;
}

uint8_t Uart_read_byte()
{
  uint8_t read_data;

  Uart_read(&read_data,1);
  return read_data;
}

size_t Uart_read(uint8_t *buffer, size_t size)
{
  // 受信しているサイズを取得
  const size_t available = Uart_rcv_size();

  // 読み出せる分だけ読み出す
  const size_t read_size = MIN(available, size);
  for (size_t i=0;i<read_size;i++) {
    buffer[i] =  rx_buffer[rx_buffer_tail];
    rx_buffer_tail = (rx_buffer_tail + 1) % UART_RX_BUFFER_SIZE;
  }

  return read_size;
}

void RingBuf_Init(void){

  pUSART_Buf = &USARTx_Buf;
  USARTx_Buf.RX_Tail = 0;
  USARTx_Buf.RX_Head = 0;
  USARTx_Buf.TX_Tail = 0;
  USARTx_Buf.TX_Head = 0;
}

/**************************************************************************/
/*!
    Check UART TX Buffer Empty.
*/
/**************************************************************************/
bool USART_TXBuffer_FreeSpace(USART_Buffer_t* USART_buf)
{
    /* Make copies to make sure that volatile access is specified. */
    unsigned int tempHead = (USART_buf->TX_Head + 1) & (UART_BUFSIZE-1);
    unsigned int tempTail = USART_buf->TX_Tail;

    /* There are data left in the buffer unless Head and Tail are equal. */
    return (tempHead != tempTail);
}

/**************************************************************************/
/*!
    Put Bytedata with Buffering.
*/
/**************************************************************************/
bool USART_TXBuffer_PutByte(USART_Buffer_t* USART_buf, uint8_t data)
{

    unsigned int tempTX_Head;
    bool TXBuffer_FreeSpace;

    TXBuffer_FreeSpace = USART_TXBuffer_FreeSpace(USART_buf);


    if(TXBuffer_FreeSpace)
    {
        tempTX_Head = USART_buf->TX_Head;

        __disable_irq();
        USART_buf->TX[tempTX_Head]= data;
        /* Advance buffer head. */
        USART_buf->TX_Head = (tempTX_Head + 1) & (UART_BUFSIZE-1);
        __enable_irq();

        /* Enable TXE interrupt. */
        USART1->CR1 |= USART_CR1_TXEIE;
    }
    return TXBuffer_FreeSpace;
}

/**************************************************************************/
/*!
    Check UART RX Buffer Empty.
*/
/**************************************************************************/
bool USART_RXBufferData_Available(USART_Buffer_t* USART_buf)
{
    /* Make copies to make sure that volatile access is specified. */
    unsigned int tempHead = pUSART_Buf->RX_Head;
    unsigned int tempTail = pUSART_Buf->RX_Tail;

    /* There are data left in the buffer unless Head and Tail are equal. */
    return (tempHead != tempTail);
}

/**************************************************************************/
/*!
    Get Bytedata with Buffering.
*/
/**************************************************************************/
uint8_t USART_RXBuffer_GetByte(USART_Buffer_t* USART_buf)
{
    uint8_t ans;

    __disable_irq();
    ans = (pUSART_Buf->RX[pUSART_Buf->RX_Tail]);

    /* Advance buffer tail. */
    pUSART_Buf->RX_Tail = (pUSART_Buf->RX_Tail + 1) & (UART_BUFSIZE-1);

    __enable_irq();

    return ans;
}

/**************************************************************************/
/*!
    High Level function.
*/
/**************************************************************************/
/* Send 1 character */
inline void putch(uint8_t data)
{
#if defined(UART_INTERRUPT_MODE)
    /* Interrupt Version */
    while(!USART_TXBuffer_FreeSpace(pUSART_Buf));
    USART_TXBuffer_PutByte(pUSART_Buf,data);
#else
    /* Polling version */
    while (!(USART1->ISR & USART_ISR_TXE));
    USART1->TDR = data;
#endif
}

/**************************************************************************/
/*!
    High Level function.
*/
/**************************************************************************/
/* Receive 1 character */
uint8_t getch(void)
{
#if defined(UART_INTERRUPT_MODE)
    if (USART_RXBufferData_Available(pUSART_Buf))  return USART_RXBuffer_GetByte(pUSART_Buf);
    else                                           return false;
#else
    /* Polling version */
    while (!(USART1->ISR & USART_ISR_RXNE));
    return (uint8_t)(USART1->RDR);
#endif
}

/**************************************************************************/
/*!
    High Level function.
*/
/**************************************************************************/
/* Return 1 if key pressed */
uint8_t keypressed(void)
{
#if defined(UART_INTERRUPT_MODE)
    return (USART_RXBufferData_Available(pUSART_Buf));
#else
    return (USART1->ISR & USART_ISR_RXNE);
#endif
}

/**************************************************************************/
/*!
    High Level function.
*/
/**************************************************************************/
/* Send a string */
void cputs(char *s)
{
    while (*s)
    putch(*s++);
}

/**************************************************************************/
/*!
    High Level function.
*/
/**************************************************************************/
/* Receive a string, with rudimentary line editing */
void cgets(char *s, int bufsize)
{
    char *p;
    uint8_t c;

    memset(s, 0, bufsize);

    p = s;

    for (p = s; p < s + bufsize-1;)
    {
        /* 20090521Nemui */
        do{
            c = getch();
        }while(c == false);
        /* 20090521Nemui */
        switch (c)
        {
            case '\r' :
            case '\n' :
                putch('\r');
                putch('\n');
                *p = '\n';
            return;

            case '\b' :
                if (p > s)
                {
                  *p-- = 0;
                  putch('\b');
                  putch(' ');
                  putch('\b');
                }
            break;

            default :
                putch(c);
                *p++ = c;
            break;
        }
    }
    return;
}

void uart_rx_callback(void){
  /* Advance buffer head. */
  unsigned int tempRX_Head = ((&USARTx_Buf)->RX_Head + 1) & (UART_BUFSIZE-1);

  /* Check for overflow. */
  unsigned int tempRX_Tail = (&USARTx_Buf)->RX_Tail;
  uint8_t data =  USART1->RDR;

  if (tempRX_Head == tempRX_Tail) {
      /* Overflow MAX size Situation */
      /* Disable the UART Receive interrupt */
    USART1->CR1 &= ~(USART_CR1_RXNEIE);
  }else{
      (&USARTx_Buf)->RX[(&USARTx_Buf)->RX_Head] = data;
      (&USARTx_Buf)->RX_Head = tempRX_Head;
  }
}

void uart_tx_callback(void){
  /* Check if all data is transmitted. */
  unsigned int tempTX_Tail = (&USARTx_Buf)->TX_Tail;
  if ((&USARTx_Buf)->TX_Head == tempTX_Tail){
      /* Overflow MAX size Situation */
      /* Disable the UART Transmit interrupt */
    USART1->CR1 &= ~(USART_CR1_TXEIE);
  }else{
      /* Start transmitting. */
      uint8_t data = (&USARTx_Buf)->TX[(&USARTx_Buf)->TX_Tail];
      USART1->TDR = data;

      /* Advance buffer tail. */
      (&USARTx_Buf)->TX_Tail = ((&USARTx_Buf)->TX_Tail + 1) & (UART_BUFSIZE-1);
  }
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
