#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Key.h"
#include "MyCAN.h"

uint8_t KeyNum;
uint32_t TxID = 0x555;
uint8_t TxLength = 4;
uint8_t TxData[8] = {0x11, 0x22, 0x33, 0x44};

uint32_t RxID;
uint8_t RxLength;
uint8_t RxData[8];

int main(void)
{
	OLED_Init();
	Key_Init();
	MyCAN_Init();
	
	OLED_ShowString(1, 1, "TxID:");
	OLED_ShowHexNum(1, 6, TxID, 3);
	OLED_ShowString(2, 1, "RxID:");
	OLED_ShowString(3, 1, "Leng:");
	OLED_ShowString(4, 1, "Data:");
	
	while (1)
	{
		KeyNum = Key_GetNum();
		
		if (KeyNum == 1)
		{
			TxData[0] ++;
			TxData[1] ++;
			TxData[2] ++;
			TxData[3] ++;
			
			MyCAN_Transmit(TxID, TxLength, TxData);
		}
		
		if (MyCAN_ReceiveFlag())
		{
			MyCAN_Receive(&RxID, &RxLength, RxData);
			
			OLED_ShowHexNum(2, 6, RxID, 3);
			OLED_ShowHexNum(3, 6, RxLength, 1);
			OLED_ShowHexNum(4, 6, RxData[0], 2);
			OLED_ShowHexNum(4, 9, RxData[1], 2);
			OLED_ShowHexNum(4, 12, RxData[2], 2);
			OLED_ShowHexNum(4, 15, RxData[3], 2);
		}
	}
}

//可以在PA12口,CAN_TX引脚,插一个LED灯,LED正极接电源正,负极接PA12,这样再按按键发送,可以看到,每按一次,LED就快速闪烁一下
//这说明,PA12输出了一段波形,如果有示波器或者逻辑分析仪,此时监测PA12的引脚,就能得到CAN数据帧的逻辑电平波形,利用这个
//环回模式TX输出的波形,我们也可以在出问题的时候定位问题位置
//比如,如果OLED屏上无法显示接收内容,就可以通过此LED灯验证,是发送出问题了还是接收出问题了,
//如果按按键,LED可以闪烁,说明发送没问题,这时可以集中找接收部分的问题, 如果按按键,LED灯不闪烁,说明没有发出波形,这时可以
//集中找发送部分的问题,这就是环回模式调试的好处
