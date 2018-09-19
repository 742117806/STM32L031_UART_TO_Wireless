#include "user_call_back.h"
#include "uart.h"
#include "wireless_app.h"

/**
  * @brief  SYSTICK callback.
  * @retval None
  */
void HAL_SYSTICK_Callback(void)
{
    //	WirelessRx_Timeout_Cnt ++;			//���߽��յ�ͬ���ֺ���պ������ݳ�ʱ����
    lpuart1Rec.timeOut++;
    //DEBUG_Printf("lpuart1Rec.timeOut = %d\r\n",lpuart1Rec.timeOut);
    if ((lpuart1Rec.timeOut > 30) && (lpuart1Rec.cnt > 0))
    {
        lpuart1Rec.Len = lpuart1Rec.cnt;
        lpuart1Rec.cnt = 0;
        lpuart1Rec.rec_ok = 1;
    }
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected to the EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        Si4438_Interrupt_Handler(&Wireless_Buf);
    }
}

/*
����2������ɻص�����
*/
uint8_t frameLen = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    extern uint8_t LP_UartRec;
    if (huart->Instance == LPUART1) //����Ǵ���1
    {
#if 0
		if(lpuart1Rec.rec_ok == 0)
		{
			switch(lpuart1Rec.state)
			{
				case 0:			//�ȴ�֡ͷ
					if(rec == 0x55)		
					{
						lpuart1Rec.state = 1;
						lpuart1Rec.cnt = 0;
						lpuart1Rec.buff[lpuart1Rec.cnt++]=rec;
					}
					break;
				case 1:
					lpuart1Rec.Len = rec+5;		//֡���ݳ���+5=��һ֡����
					lpuart1Rec.state = 2;
					lpuart1Rec.buff[lpuart1Rec.cnt++]=rec;
					break;
				case 2:
					lpuart1Rec.buff[lpuart1Rec.cnt++]=rec;
					if(lpuart1Rec.cnt>=lpuart1Rec.Len)
					{
						if(rec == 0xAA)
						{
							lpuart1Rec.rec_ok = 1;
						}
						else
						{
							lpuart1Rec.state = 0;
						}
					}
					break;
			}
		}

#else
        if (lpuart1Rec.rec_ok == 0)
        {
            lpuart1Rec.timeOut = 0;
            lpuart1Rec.buff[lpuart1Rec.cnt++] = LP_UartRec;
        }
#endif
    }
    if (huart->Instance == USART2)
    {
        UpUart_RX_INT_Process(uart2_rec, &UpCom_RxBuf);
    }
}
