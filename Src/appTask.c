
#include "appTask.h"
#include "wireless_app.h"
#include "wireless_drv.h"
#include "delay.h"
#include <string.h>
#include "aes.h"
#include "encrypt.h"
#include "device.h"
#include "74.h"

//密文初始化
void AES_Init(void)
{
	//计算出密文，存放在aes_w，供加解密用
	memcpy(&aes_out[2*RsaByte_Size],devicInfo.aes,16);
	memcpy(&aes_out[3*RsaByte_Size],devicInfo.addr_GA,3);
	
	Rsa_Decode(aes_out);  
	key_expansion(aes_out, aes_w);  
}



/*
无线数据处理任务
*/
void WirelessTask(void)
{
	uint8_t   out_len;  
	FRAME_CMD_t*p;

    if (WIRELESS_STATUS == Wireless_RX_Finish)
    {
        #if _74CODE_EN
		p = (FRAME_CMD_t*)Wireless_Buf.Wireless_RxData;
		if(p->Ctrl.c_AFN == 0)
		{
			FrameData_74Convert((FRAME_CMD_t*)Wireless_Buf.Wireless_RxData,Wireless_Buf.Wireless_PacketLength,&out_len,0); //解码
			Wireless_Buf.Wireless_PacketLength = out_len;		//解码后长度
		}
		#endif
		#if _ENCRYPT_EN
		if(p->FSQ.encryptType != 0)
		{
		
			Encrypt_Convert(Wireless_Buf.Wireless_RxData,Wireless_Buf.Wireless_PacketLength,&out_len,0);		//解密
			Wireless_Buf.Wireless_PacketLength = out_len;
		}
		#endif
		UartSendBytes(LPUART1,Wireless_Buf.Wireless_RxData, Wireless_Buf.Wireless_PacketLength);			
		Si4438_Receive_Start(Wireless_Channel[0]);

    }
    if (WIRELESS_STATUS == Wireless_TX_Finish)
    {
        DEBUG_Printf("Wireless_TX_Finish\r\n");
        Si4438_Receive_Start(Wireless_Channel[0]); //开始接收无线数据
    }
    else if (WIRELESS_STATUS == Wireless_RX_Failure)
    {
        WirelessRx_Timeout_Cnt = 0;
        DEBUG_Printf("Wireless_RX_Failure\r\n");
        delay_ms(30);
        Set_Property(Interrupt_Close);
        delay_ms(200);
        Si4438_Receive_Start(Wireless_Channel[0]); //开始接收无线数据
    }
    else if ((WIRELESS_STATUS == Wireless_RX_Sync) && (WirelessRx_Timeout_Cnt > 500)) //500ms超时
    {

        DEBUG_Printf("Wireless_RX_Sync\r\n");
        delay_ms(30);
        Set_Property(Interrupt_Close);
        delay_ms(200);
        Si4438_Receive_Start(Wireless_Channel[0]); //开始接收无线数据
        WirelessRx_Timeout_Cnt = 0;
    }
}
/*
串口数据处理任务
*/
void UartTask(void)
{
    uint16_t i;
	uint8_t out_len;

	
	//串口1
    if (lpuart1Rec.rec_ok == 1)
    {
//		sleep_delay_cnt = 0;		//复位延迟睡眠计算
        lpuart1Rec.rec_ok = 0;
        #if _ENCRYPT_EN
		Encrypt_Convert(lpuart1Rec.buff, lpuart1Rec.Len, &out_len, 1);		//加密
		lpuart1Rec.Len = out_len;											//把加密后的数据长度赋值给原来数据的长度
		#endif
		#if _74CODE_EN
		FrameData_74Convert((FRAME_CMD_t*)lpuart1Rec.buff,lpuart1Rec.Len,&out_len,1); //编码
		lpuart1Rec.Len = out_len;		//编码后长度	
		#endif
        UartSendBytes(LPUART1,lpuart1Rec.buff, lpuart1Rec.Len);
        Si4438_Transmit_Start(&Wireless_Buf, Wireless_Channel[0], lpuart1Rec.buff, lpuart1Rec.Len);
		
    }
	//串口2
    if (UpCom_RxBuf.Rx_Status == UartRx_Finished)
    {
        UpCom_RxBuf.Rx_Status = UartRx_FrameHead;

        for (i = 0; i < UpCom_RxBuf.FrameTotalLen; i++)
        {
            DEBUG_Printf("%02x ", UpCom_RxBuf.Frame_Data[i]);
        }
    }
}

