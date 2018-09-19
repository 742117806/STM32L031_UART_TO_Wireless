#ifndef __UART_H
#define __UART_H

#include "stm32l0xx.h"
#include "protocol.h"

#define _DEBUG_ 0				//定义是否打印调试信息：0不打印，1打印

#if _DEBUG_
#define DEBUG_Printf(...)   printf(__VA_ARGS__)
#define DEBUT_SendBytes(b,n)	UartSendBytes(LPUART1,b,n);
#else
#define DEBUG_Printf(...) 
#define DEBUT_SendBytes(b,n)
#endif


extern UART_HandleTypeDef hlpuart1,husart2;



typedef struct UartRec_
{
	uint8_t buff[300];
	uint16_t Len;
	uint16_t cnt;
	uint8_t rec_ok:1;
	uint8_t state:7;
	uint8_t timeOut;
}UartRec_t;
 
extern UartRec_t lpuart1Rec;
extern uint8_t uart2_rec;
void UartSendData(USART_TypeDef *UARTx, uint8_t byte);

void UartSendBytes(USART_TypeDef *UARTx, uint8_t *buf, uint16_t len);

#define DownCom                      USART2
#define UpCom                        USART2

#define UpCom_TXINT_EN()              (UpCom->CR1 |= 1<<7)
#define UpCom_TXINT_DIS()             (UpCom->CR1 &= ~(1<<7))
#define UpCom_TCIE_EN()               (UpCom->CR1 |= 1<<6)
#define UpCom_TCIE_DIS()              (UpCom->CR1 &= ~(1<<6))
#define UpCom_RXINT_EN()              (UpCom->CR1 |= 1<<5)
#define UpCom_RXINT_DIS()             (UpCom->CR1 &= ~(1<<5))

#define DownCom_TXINT_EN()              (DownCom->CR1 |= 1<<7)
#define DownCom_TXINT_DIS()             (DownCom->CR1 &= ~(1<<7))
#define DownCom_TCIE_EN()               (DownCom->CR1 |= 1<<6)
#define DownCom_TCIE_DIS()              (DownCom->CR1 &= ~(1<<6))
#define DownCom_RXINT_EN()              (DownCom->CR1 |= 1<<5)
#define DownCom_RXINT_DIS()             (DownCom->CR1 &= ~(1<<5))





#define Up_TimeOut_Val                              9     //250Byte约62.5ms










typedef enum
{
  Uart_NoError = 0,
  Uart_TxBusy,
  Uart_ParaError,
}Uart_ErrorType;


  
  






#define Tx_GapTime_Size             6 


typedef enum
{
  UartTx_Finished = 0,  //发送完成
  UartTx_Start,         //启动发送
  UartTx_End,         //发送结束
}Uart_TxSta_TypDef;

//#define UARTFrame_TxLen_MAX   MAXDOWNBUF - 32
typedef struct
{
  Uart_TxSta_TypDef Tx_Status;
  uint8_t Frame_Len;
  uint8_t Byte_Cnt;
  uint8_t Frame_Data[HKFrame_LenMax]; 
}UartFrame_TX_TypDef;


/*************************************************************/







/************************* 外部变量 ***************************/

extern UpCom_Rx_TypDef         DownCom_RxBuf;
extern UartFrame_TX_TypDef     DownCom_TxBuf;


extern UpCom_Rx_TypDef         UpCom_RxBuf;
extern UartFrame_TX_TypDef     UpCom_TxBuf;

extern uint8_t  Secret_GKey_Flag;     //密钥公钥存在标志
extern uint8_t  Secret_KeyOk_Flag;     //密钥OK标志
/************************* 外部函数 ***************************/


#endif

