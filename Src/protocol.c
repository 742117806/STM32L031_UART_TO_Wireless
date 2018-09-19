#include "protocol.h"
#include "crc16.h"
#include "string.h"
#include "74.h"

//����һ֡����Э���֡����
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

//�����߱���ʹ�õ�У�麯��
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
**��   �ܣ�����Ӧ��Э�����74����(��һ֡���ݵ��û���������ʼ��֡������ǰ�������)
**��   ����
        @srcData Դ����
        @srcLen Դ���ݵĳ��ȣ��ֽ�����
		@outLen	�����һ֡���ݵĳ���
        @mode 1���룬0����
**����ֵ:��
****************************************************************/
void FrameData_74Convert(FRAME_CMD_t *srcData,uint8_t srcLen,uint8_t *outLen,uint8_t mode)
{
	uint8_t frame_len;
	uint8_t temp[256]={0};
	uint16_t crc16;
	uint8_t *p_frame_data;
	
	if(mode==0)	//����
	{
       	frame_len = _74DecodeBytes((uint8_t*)&srcData->userData,temp,srcLen-11);       //�ѱ������ӵ�CRC16(2���ֽ�)ȥ��
	    temp[frame_len] = HKFreamEnd;
		memcpy((uint8_t*)&srcData->userData,temp,frame_len+1);
		*outLen = frame_len+9;		
	}
	else        //����
	{
		frame_len = _74CodeBytes((uint8_t*)&srcData->userData,temp,srcLen-9);    //74����
		
		memcpy((uint8_t*)&srcData->userData,temp,frame_len);					//�ѱ���õ����ݸ��ƻ�ԭ�����ݵĻ�������
		crc16 = CRC16_2((uint8_t*)srcData,frame_len+8);			//����󳤶�+Э��֡ǰ��û�����8���ֽ�֡����
		p_frame_data =  (uint8_t*)&srcData->userData;
		p_frame_data[frame_len]= (crc16 >> 8);
		p_frame_data[frame_len+1]= (crc16 & 0x00ff);
		p_frame_data[frame_len+2] = HKFreamEnd;				                            //��������֡����0x53
		*outLen = frame_len+8+3;
	}
}

/*********************************************************************
**��  �ܣ����յ��Դ��ڷ�����Э������
**��  ����
        @ rx_data ���ڽ��յ���һ���ֽ����� 
        @ pu_buf Э��֡���ݴ洢����ָ��
**����ֵ:��
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
**��  �ܣ����պ��İ壨�����ư壩 ���͹����Ĵ�������
**��  ����
        @ rx_data ���ڽ��յ���һ���ֽ����� 
        @ pu_buf Э��֡���ݴ洢����ָ��
**����ֵ:��
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
        if (rx_data == HKFreamEnd) //�жϽ������Ƿ���ȷ
        {
            //DownCom_RXINT_DIS(); //���ж�
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




