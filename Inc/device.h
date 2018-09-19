#ifndef __DEVICE_H
#define __DEVICE_H
#include "stm32l0xx.h"


#define _74CODE_EN		0	//ͨ��74����
#define _ENCRYPT_EN     0   //ͨ�����ݼ���
/*
�豸��Ϣ
*/
typedef struct DeviceInfo_
{
	uint8_t mac[8];			//�豸MAC��ַ
	uint8_t aes[16];		//��Կ
	uint8_t addr_DA;		//�߼���ַ
	uint8_t addr_GA[3];		//Ⱥ�ڵ�ַ
}DeviceInfo_t;


extern DeviceInfo_t devicInfo;

#endif

