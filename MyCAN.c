#include "stm32f10x.h"                  // Device header

void MyCAN_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  //这样PA12就配置成了复用推挽输出模式
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  // PA11号口,初始化为上拉输入模式
	
	CAN_InitTypeDef CAN_InitStructure;
	CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;//CAN_Mode设置CAN外设的测试模式(正常,静默,环回静默)
	
	//配置位时序相关的函数                        波特率=36M/分频系数/(1+BS1 Tq数+ BS2 Tq数)
	CAN_InitStructure.CAN_Prescaler = 48;		//波特率 = 36M / 48 / (1 + 2 + 3) = 125K(高数CAN 125K~1M)
	CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq; //BS1段(Bit Segment 1)    Tq(time quanta)
	CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
	CAN_InitStructure.CAN_SJW = CAN_SJW_2tq; //SIW再同步补偿宽度,SJW值仅用于再同步,与波特率的计算无关
	
	
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_Init(CAN1, &CAN_InitStructure);
	
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	
	// 16位列表模式,这四个参数分别存入一组ID即可,  
	// 16位屏蔽模式,IdHigh,存入第一组ID,MaskIdHigh存入对应的屏蔽位  IDLow,存入第二组ID,MaskIdLow存入对应的屏蔽位
	// 32位列表模式,IdHigh和IdLow组合在一起,存入第一组32位ID,  MaskIdHigh和MaskIdLow组合在一起,存入第二组32位ID
	// 32位屏蔽模式,IdHigh和IdLow组合在一起,存入32位ID, MaskIdHigh和MaskIdLow组合在一起,存入对应的屏蔽位
	//屏蔽位全给0 就是全通状态,前提得选用32位屏蔽模式
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
}


//封装一个发送报文的函数  里面没有CRC,因为CRC由硬件自动生成和校验,不需要管
void MyCAN_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data)
	//定义成32位,方便以后可能会写入扩展ID, 传入数据长度,   传入数据内容
{
	CanTxMsg TxMessage;
	TxMessage.StdId = ID;//StdId 标准ID
	TxMessage.ExtId = ID;// ExtId  扩展ID
	TxMessage.IDE = CAN_Id_Standard;	//IDE 扩展标志位	//CAN_ID_STD
	TxMessage.RTR = CAN_RTR_Data;   //RTR遥控标志位
	TxMessage.DLC = Length;         //数据长度
	for (uint8_t i = 0; i < Length; i ++)
	{
		TxMessage.Data[i] = Data[i]; //Data 数据段内容  这个Data和结构体成员同名,但并不是一个变量
	}
	
	uint8_t TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
	//调用CAN_Transmit,这个结构体指向的报文,就会被写入发送邮箱,并由管理员发送
	
	uint32_t Timeout = 0; //防止程序卡死,用一个超时退出,先定义一个变量
	while (CAN_TransmitStatus(CAN1, TransmitMailbox) != CAN_TxStatus_Ok)
		//CAN_TransmitStatus等待发送完成的函数:检查邮箱的状态   TransmitMailbox代表我们想要检查的那个邮箱的状态
	//	Transmit函数返回值,就表示它选中的邮箱   // CAN_TxStatus_Failed  表示其他状况
	{
		Timeout ++; 
		if (Timeout > 100000)  //一般超时时间大于正常时间的10倍,就肯定不会影响正常等待
			{//这里如果循环等待10万次还没有成功,就直接退出,不等了,避免程序卡死
			break;
		}
	}
}

// 代码第三部分  CAN总线的接收  1,使用查询的方法,这种方式简单   2,使用中断,接收效率更高
//对于查询接收,需要两个函数,1,判断FIFO里是否有报文. 2,读取接收FIFO,把报文内容取出来
uint8_t MyCAN_ReceiveFlag(void)//用来判断接收FIFO里是否有报文
{
	if (CAN_MessagePending(CAN1, CAN_FIFO0) > 0)
	{
		return 1;
	}
	return 0;
}//返回值,定义  有就 1;  没有就 0;

//读取FIFO的过程
void MyCAN_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data)
	//void MyCAN_Receive读取FIFO 0 报文数据
//uint32_t *ID, uint8_t *Length, uint8_t *Data这三个参数,都是指针传递的输出参数,可以当初函数的返回值来理解,因为C 语言
//因为C 语言不支持函数多返回值,所以这里用输出函数,相当于函数返回多个值
{
	CanRxMsg RxMessage;
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	
	if (RxMessage.IDE == CAN_Id_Standard)
	{
		*ID = RxMessage.StdId;
	}
	else
	{
		*ID = RxMessage.ExtId;
	}
	
	if (RxMessage.RTR == CAN_RTR_Data)
	{
		*Length = RxMessage.DLC;
		for (uint8_t i = 0; i < *Length; i ++)
		{
			Data[i] = RxMessage.Data[i];
		}
	}
	else
	{
		//...
	}
}
