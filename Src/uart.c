#include "uart.h"
#include "stm32l0xx.h"
#include <string.h>
#include "protocol.h"
UartFrame_TX_TypDef DownCom_TxBuf;
uint8_t Secret_GKey_Flag;  //密钥公钥存在标志
uint8_t Secret_KeyOk_Flag; //密钥OK标志
UartFrame_TX_TypDef UpCom_TxBuf;

extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart2;
UpCom_Rx_TypDef UpCom_RxBuf;
uint8_t rec;
uint8_t uart2_rec;

UartRec_t lpuart1Rec;
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int fputc(int ch, FILE *f)
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART2 and Loop until the end of transmission */

    LPUART1->TDR = ch;
    while ((LPUART1->ISR & 0X40) == 0)
        ;
    return ch;
}





void UartSendData(USART_TypeDef *UARTx, uint8_t byte)
{
    UARTx->TDR = byte;

    while ((UARTx->ISR & 0X40) == 0)
        ;
}
void UartSendBytes(USART_TypeDef *UARTx, uint8_t *buf, uint16_t len)
{
    uint8_t i = 0;
    for (i = 0; i < len; i++)
    {
        UartSendData(UARTx,*buf++);
    }
}

