#include "protocol.h"
#include "crc16.h"
#include "string.h"
#include "74.h"

//整合一帧符合协议的帧数据
uint8_t Frame_Compose(uint8_t *p)
{
    uint8_t ComposeFrame_Len;
    uint16_t crc16_val;


    p[Region_HeaderNumber] = HKFreamHeader;

    p[Region_CmdNumber] &= ~(1 << 5);
    ComposeFrame_Len = p[Region_DataLenNumber] + (Region_DataLenNumber + 1);
    crc16_val = CRC16_2(p, ComposeFrame_Len);
    p[ComposeFrame_Len] = crc16_val >> 8;
    ComposeFrame_Len++;
    p[ComposeFrame_Len] = crc16_val & 0xff;
    ComposeFrame_Len++;
    p[ComposeFrame_Len] = HKFreamEnd;
    ComposeFrame_Len++;

    return ComposeFrame_Len;
}

//给无线编码使用的校验函数
uint8_t Frame_Check(uint8_t *p, uint8_t Len)
{
    uint8_t ComposeFrame_Len;
    uint16_t crc16_val;

    p[Region_HeaderNumber] = HKFreamHeader;

    ComposeFrame_Len = Len;
    crc16_val = CRC16_2(p, ComposeFrame_Len);
    p[ComposeFrame_Len] = crc16_val >> 8;
    ComposeFrame_Len++;
    p[ComposeFrame_Len] = crc16_val & 0xff;
    ComposeFrame_Len++;
    p[ComposeFrame_Len] = HKFreamEnd;
    ComposeFrame_Len++;

    return ComposeFrame_Len;
}

/****************************************************************
**功   能：按照应用协议进行74编码(从一帧数据的用户数据区开始到帧结束符前面的数据)
**参   数：
        @srcData 源数据
        @srcLen 源数据的长度（字节数）
		@outLen	编码后一帧数据的长度
        @mode 1编码，0解码
**返回值:无
****************************************************************/
void FrameData_74Convert(FRAME_CMD_t *srcData,uint8_t srcLen,uint8_t *outLen,uint8_t mode)
{
	uint8_t frame_len;
	uint8_t temp[256]={0};
	uint16_t crc16;
	uint8_t *p_frame_data;
	
	if(mode==0)	//解码
	{
       	frame_len = _74DecodeBytes((uint8_t*)&srcData->userData,temp,srcLen-11);       //把编码过后加的CRC16(2个字节)去掉
	    temp[frame_len] = HKFreamEnd;
		memcpy((uint8_t*)&srcData->userData,temp,frame_len+1);
		*outLen = frame_len+9;		
	}
	else        //编码
	{
		frame_len = _74CodeBytes((uint8_t*)&srcData->userData,temp,srcLen-9);    //74编码
		
		memcpy((uint8_t*)&srcData->userData,temp,frame_len);					//把编码好的数据复制回原来数据的缓存区，
		crc16 = CRC16_2((uint8_t*)srcData,frame_len+8);			//编码后长度+协议帧前面没编码的8个字节帧数据
		p_frame_data =  (uint8_t*)&srcData->userData;
		p_frame_data[frame_len]= (crc16 >> 8);
		p_frame_data[frame_len+1]= (crc16 & 0x00ff);
		p_frame_data[frame_len+2] = HKFreamEnd;				                            //编码后加上帧结束0x53
		*outLen = frame_len+8+3;
	}
}

/*********************************************************************
**功  能：接收电脑串口发来的协议数据
**参  数：
        @ rx_data 串口接收到的一个字节数据 
        @ pu_buf 协议帧数据存储缓存指针
**返回值:无
*********************************************************************/
void UpUart_RX_INT_Process(uint8_t rx_data, UpCom_Rx_TypDef *pu_buf)
{

    switch (pu_buf->Rx_Status)
    {
    case UartRx_FrameHead:
        if (rx_data == AESFreamHeader)
        {
            pu_buf->Frame_Data[0] = rx_data;
            pu_buf->Over_time_count = 1;
            pu_buf->Rx_Status = UartRx_AesCmd;
        }
        break;

    case UartRx_AesCmd:
        pu_buf->Frame_Data[1] = rx_data;
        pu_buf->Rx_Status = UartRx_Aeslen;
        break;

    case UartRx_Aeslen:
        if (rx_data == 0)
        {
            pu_buf->Frame_Data[2] = 0;
            pu_buf->Rx_Status = UartRx_AesFrameCs1;
            pu_buf->FrameTotalLen = 3;
        }
        else if (rx_data > GK_Data_Len)
        {
            pu_buf->Over_time_count = 0;
            pu_buf->Rx_Status = UartRx_FrameHead;
        }
        else
        {
            pu_buf->Frame_Data[2] = rx_data;
            pu_buf->Rx_Status = UartRx_AesData;
            pu_buf->FrameTotalLen = 3;
        }
        break;

    case UartRx_AesData:
        pu_buf->Frame_Data[pu_buf->FrameTotalLen] = rx_data;
        if (++pu_buf->FrameTotalLen >= (pu_buf->Frame_Data[2] + 3))
        {
            pu_buf->Rx_Status = UartRx_AesFrameCs1;
        }

        break;

    case UartRx_AesFrameCs1:
        pu_buf->Frame_Data[pu_buf->FrameTotalLen] = rx_data;
        pu_buf->FrameTotalLen++;
        pu_buf->Rx_Status = UartRx_AesFrameCs2;
        break;

    case UartRx_AesFrameCs2:
        pu_buf->Frame_Data[pu_buf->FrameTotalLen] = rx_data;
        pu_buf->FrameTotalLen++;
        //UpCom_RXINT_DIS();
        pu_buf->Over_time_count = 0;
        pu_buf->Rx_Status = UartRx_Finished;
        break;

    default:
        break;
    }
}

/*********************************************************************
**功  能：接收核心板（主控制板） 发送过来的串口数据
**参  数：
        @ rx_data 串口接收到的一个字节数据 
        @ pu_buf 协议帧数据存储缓存指针
**返回值:无
*********************************************************************/
void DownUart_RX_INT_Process(uint8_t rx_data, UpCom_Rx_TypDef *pu_buf)
{
    static uint8_t Len_Cnt;
	
    switch (pu_buf->Rx_Status)
    {
    case UartRx_FrameHead:
        if (rx_data == HKFreamHeader)
        {
            pu_buf->Frame_Data[0] = rx_data;
            pu_buf->FrameTotalLen = 1;
            Len_Cnt = 0;
            pu_buf->Over_time_count = 1;
            pu_buf->Rx_Status = UartRx_FrameAddr;
        }
        break;

    case UartRx_FrameAddr:
        pu_buf->Frame_Data[pu_buf->FrameTotalLen] = rx_data;
        pu_buf->FrameTotalLen++;
        if (++Len_Cnt >= LogicAddr_Len)
            pu_buf->Rx_Status = UartRx_FrameSeq;
        break;

    case UartRx_FrameSeq:
        pu_buf->Frame_Data[pu_buf->FrameTotalLen] = rx_data;
        pu_buf->FrameTotalLen++;
        pu_buf->Rx_Status = UartRx_DataCmd;
        break;

    case UartRx_DataCmd:
        pu_buf->Frame_Data[pu_buf->FrameTotalLen] = rx_data;
        pu_buf->FrameTotalLen++;
        pu_buf->Rx_Status = UartRx_Datalen;
        break;

    case UartRx_Datalen:
        if (rx_data > HKData_LenMax)
        {
            pu_buf->Over_time_count = 0;
            pu_buf->Rx_Status = UartRx_FrameHead;
        }
        else
        {
            pu_buf->Frame_Data[pu_buf->FrameTotalLen] = rx_data;
            pu_buf->FrameTotalLen++;
            Len_Cnt = 0;
            if (rx_data == 0)
                pu_buf->Rx_Status = UartRx_FrameCs;
            else
                pu_buf->Rx_Status = UartRx_Data;
        }
        break;

    case UartRx_Data:
        pu_buf->Frame_Data[pu_buf->FrameTotalLen] = rx_data;
        pu_buf->FrameTotalLen++;
        if (++Len_Cnt >= pu_buf->Frame_Data[Region_DataLenNumber])
        {
            Len_Cnt = 0;
            pu_buf->Rx_Status = UartRx_FrameCs;
        }

        break;

    case UartRx_FrameCs:
        pu_buf->Frame_Data[pu_buf->FrameTotalLen] = rx_data;
        pu_buf->FrameTotalLen++;
        if (++Len_Cnt >= 2)
            pu_buf->Rx_Status = UartRx_FrameEnd;
        break;

    case UartRx_FrameEnd:
        if (rx_data == HKFreamEnd) //判断结束符是否正确
        {
            //DownCom_RXINT_DIS(); //关中断
            pu_buf->Frame_Data[pu_buf->FrameTotalLen] = rx_data;
            pu_buf->FrameTotalLen++;
            pu_buf->Over_time_count = 0;
            pu_buf->Rx_Status = UartRx_Finished;
        }
        else
        {
            pu_buf->Over_time_count = 0;
            pu_buf->Rx_Status = UartRx_FrameHead;
        }
        break;
	default:
		break;
    }
}




