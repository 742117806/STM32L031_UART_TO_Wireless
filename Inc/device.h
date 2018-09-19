#ifndef __DEVICE_H
#define __DEVICE_H
#include "stm32l0xx.h"


#define _74CODE_EN		0	//通信74编码
#define _ENCRYPT_EN     0   //通信数据加密
/*
设备信息
*/
typedef struct DeviceInfo_
{
	uint8_t mac[8];			//设备MAC地址
	uint8_t aes[16];		//密钥
	uint8_t addr_DA;		//逻辑地址
	uint8_t addr_GA[3];		//群众地址
}DeviceInfo_t;


extern DeviceInfo_t devicInfo;

#endif

