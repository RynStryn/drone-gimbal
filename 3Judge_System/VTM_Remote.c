#include <main.h>

uint8_t _USART6_DMA_RX_BUF[BSP_USART6_RX_BUF_LENGTH];
uint8_t _USART6_DMA_TX_BUF[BSP_USART6_TX_BUF_LENGTH];
void usart6_init()
{
    USART_InitTypeDef usart;
    GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
		DMA_InitTypeDef dma;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); 
	
		//串口1对应引脚复用映射
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOC6为USART1
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOC7复用为USART1
		
		//USART1端口配置
		gpio.GPIO_Pin 	= GPIO_Pin_6 | GPIO_Pin_7; //GPIOC6与GPIOC7
		gpio.GPIO_Mode 	= GPIO_Mode_AF;//复用功能
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
		gpio.GPIO_PuPd 	= GPIO_PuPd_UP; //上拉
		GPIO_Init(GPIOC,&gpio); //初始化PC6，PC7

    USART_DeInit(USART6);
//    USART_StructInit(&usart);
    usart.USART_BaudRate = 921600;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART6, &usart);   


		USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
    
		DMA_DeInit(DMA2_Stream1);
    dma.DMA_Channel = DMA_Channel_5;
    dma.DMA_PeripheralBaseAddr	= (uint32_t)(&USART6->DR);
    dma.DMA_Memory0BaseAddr   	= (uint32_t)&_USART6_DMA_RX_BUF[0];
    dma.DMA_DIR 			    = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize			= BSP_USART6_RX_BUF_LENGTH;//sizeof(USART1_DMA_RX_BUF);
    dma.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
    dma.DMA_Mode 				= DMA_Mode_Normal;
    dma.DMA_Priority 			= DMA_Priority_Medium;
    dma.DMA_FIFOMode 			= DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold 		= DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst 		= DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream1, &dma);
    DMA_Cmd(DMA2_Stream1, ENABLE);
		nvic.NVIC_IRQChannel = USART6_IRQn;                          
		nvic.NVIC_IRQChannelPreemptionPriority = 2;   //pre-emption priority 
		nvic.NVIC_IRQChannelSubPriority = 2;		    //subpriority 
		nvic.NVIC_IRQChannelCmd = ENABLE;			
		NVIC_Init(&nvic);	
		USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled


	USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);
	while(DMA_GetCmdStatus(DMA2_Stream6) != DISABLE) {}
	dma.DMA_Channel = DMA_Channel_5;
	dma.DMA_PeripheralBaseAddr	= (uint32_t)(&USART6->DR);
	dma.DMA_Memory0BaseAddr   	= (uint32_t)&_USART6_DMA_TX_BUF[0];
	dma.DMA_DIR 			   				 = DMA_DIR_MemoryToPeripheral;
	dma.DMA_BufferSize					= 0;//sizeof(UART4_DMA_TX_BUF);
	dma.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc 					= DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;
	dma.DMA_Mode 								= DMA_Mode_Normal;
	dma.DMA_Priority 						= DMA_Priority_Medium;
	dma.DMA_FIFOMode 						= DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold 			= DMA_FIFOThreshold_Full;
	dma.DMA_MemoryBurst 				= DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream6,&dma);

	DMA_Cmd(DMA2_Stream6, DISABLE);                           // 关DMA通道
	nvic.NVIC_IRQChannel = DMA2_Stream6_IRQn;   // 发送DMA通道的中断配置
	nvic.NVIC_IRQChannelPreemptionPriority = 3;     // 优先级设置
	nvic.NVIC_IRQChannelSubPriority = 2;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);

	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);
		
		
    USART_Cmd(USART6,ENABLE);
}

//串口接收中断服务函数
void USART6_IRQHandler(void)

{
	static uint32_t this_time_rx_len6 = 0;
	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
	{
		(void)USART6->SR;
		(void)USART6->DR;
		DMA_Cmd(DMA2_Stream1, DISABLE); 
		DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);  //************************************
		this_time_rx_len6 = BSP_USART6_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA2_Stream1);

		VTM_Reomte_Data_Handle(&_USART6_DMA_RX_BUF[0],this_time_rx_len6);
		Equipment_Cnt_Clear(&Peripheral_State.VTM_Remote);
		DMA_SetCurrDataCounter(DMA2_Stream1,BSP_USART6_RX_BUF_LENGTH);
		DMA_Cmd(DMA2_Stream1, ENABLE);
	}       
}

void DMA2_Stream6_IRQHandler(void)
{
  //清除标志
  if(DMA_GetFlagStatus(DMA2_Stream6,DMA_FLAG_TCIF6)!=RESET)//等待DMA1_Steam3传输完成
    {
      DMA_Cmd(DMA2_Stream6, DISABLE);                      //关闭DMA传输
      DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6);//清除DMA1_Steam3传输完成标志
    }
}

 void Uart6DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)

{
    DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输
    DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6);
    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}  //确保DMA可以被设置
  DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量
  DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输
}

void Uart6SendBytesInfoProc(u8* pSendInfo, u16 nSendCount)
{
  u16 i = 0;
  u8 *pBuf = NULL;
  //指向发送缓冲区
  pBuf = _USART6_DMA_TX_BUF;
  for (i=0; i<nSendCount; i++)
    {
      *(pBuf+i) = pSendInfo[i];
    }

  //DMA发送方式

  Uart6DmaSendDataProc(DMA2_Stream6,nSendCount); //开始一次DMA传输！
}



static uint16_t crc16_init = 0xffff;
static const uint16_t crc16_tab[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/**
 * @brief Get the crc16 checksum
 *
 * @param p_msg Data to check
 * @param lenData length
 * @param crc16 Crc16 initialized checksum
 * @return crc16 Crc16 checksum
 */
uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;

    if(p_msg == NULL)
    {
        return 0xffff;
    }

    while(len--)
    {
        data = *p_msg++;
        (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
    }

    return crc16;
}

/**
 * @brief crc16 verify function
 *
 * @param p_msg Data to verify
 * @param len Stream length=data+checksum
 * @return bool Crc16 check result
 */
int32_t verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
    uint16_t w_expected = 0;

    if((p_msg == NULL) || (len <= 2))
    {
//        return false;
		  return 0;
    }
    w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

    return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}


remote_data_t vtm_remote_data;

//Key_Flag_t Key_Flag;
u16 Remote_CRC16;
u16 CRC_Check;
void VTM_Reomte_Data_Handle(uint8_t *pData,u16 rec_len)
{
	
	u16 deal_cnt = 0;
	u16 Frame_length = 0;
	
	Frame_length = 21;		//一帧数据总长度为152
	
	while(rec_len > deal_cnt)		//这个while好像没啥实际用处
	{
		vtm_remote_data.sof_1 = pData[deal_cnt];
		vtm_remote_data.sof_2 = pData[deal_cnt + 1];
		
		CRC_Check= verify_crc16_check_sum(pData,Frame_length);
		Remote_CRC16=Get_CRC16_Check_Sum(pData,Frame_length-2,crc16_init);
		
		if(vtm_remote_data.sof_1==0xA9&&vtm_remote_data.sof_2==0x53)
		{
			if(verify_crc16_check_sum(&pData[deal_cnt],Frame_length))
			{	
				memcpy(&vtm_remote_data,pData,sizeof(remote_data_t));
	//			vtm_remote_data.ch_0 = (pData[3]<<8) | pData[2];
	//			vtm_remote_data.ch_1 = (pData[4]<<5) | (pData[3]>>3);
	//			vtm_remote_data.ch_2 = (pData[6]<<10) | (pData[5]<<2) | (pData[4]>>6);
	//			vtm_remote_data.ch_3 = (pData[7]<<7) | (pData[6]>>1);
	//			
	//			
	//			vtm_remote_data.mode_sw = (pData[7]>>4)&0x03;
	//			vtm_remote_data.pause = (pData[7]>>6)&0x01;
	//			vtm_remote_data.fn_1 = (pData[7]>>7)&0x01;
	//			vtm_remote_data.fn_2 = pData[8]&0x01;
	//			
	//			
	//			vtm_remote_data.wheel = (pData[9]<<7) | (pData[8]>>1);
	//			vtm_remote_data.trigger = (pData[9]>>4)&0x01;
	//			
	//			
	//			vtm_remote_data.mouse_x = (pData[11]<<8) | pData[10];
	//			vtm_remote_data.mouse_y = (pData[13]<<8) | pData[12];
	//			vtm_remote_data.mouse_z = (pData[15]<<8) | pData[14];
	//			vtm_remote_data.mouse_left = pData[16]&0x03;
	//			vtm_remote_data.mouse_right = pData[16]&0x0C;
	//			vtm_remote_data.mouse_middle = pData[16]&0x30;
				
				Remote_data.Remote_Control.ch0   = vtm_remote_data.ch_0;
				Remote_data.Remote_Control.ch1   = vtm_remote_data.ch_1;
				Remote_data.Remote_Control.ch2   = vtm_remote_data.ch_3;
				Remote_data.Remote_Control.ch3   = vtm_remote_data.ch_2; 
				Remote_data.Remote_Control.ch4   = 2048-vtm_remote_data.wheel;//灰控和白控拨轮极性相反，需处理
				Remote_data.Remote_Mouse.x 	     = vtm_remote_data.mouse_x;;
				Remote_data.Remote_Mouse.y       = vtm_remote_data.mouse_y;
				Remote_data.Remote_Mouse.z       = vtm_remote_data.mouse_z;
				Remote_data.Remote_Mouse.press_l = vtm_remote_data.mouse_left;
				Remote_data.Remote_Mouse.press_r = vtm_remote_data.mouse_right;
				Remote_data.Remote_Key.v		 = (pData[18]<<8) | pData[17];
				
				deal_cnt += Frame_length;
        continue; // 继续解析缓冲区里的下一帧
			}
		}

//		keyborad_process(&RC_CtrlData);
		deal_cnt++;
	}
//	VTM_Switch_Action_Get();
//	GetRemoteSwitchAction(&RC_CtrlData);
}

u8 fn_1_flag,fn_1_trigger_flag,pause_flag,pause_trigger_flag;
u8 fn_1_cnt,pause_cnt,fn_2_cnt,trigger_cnt;
u8 fn_2_flag,fn_2_trigger_flag,trigger_flag,trigger_flag1;
void VTM_Switch_Action_Get(void)
{
	if(RemoteCtrl_Mode!=RC_RELAX)
	{
		
		//////////////////////////////////    fn_1触发式标志位    ///////////////////////////////////////////////////	
		if(vtm_remote_data.fn_1 == 1)
		{
			if(fn_1_flag == 0)
			{
				fn_1_cnt++;
				fn_1_flag = 1;
			}
		}
		else
		{
			fn_1_flag = 0;
		}
		if(fn_1_cnt % 2 == 1)
		{
			fn_1_trigger_flag = 1;
		}
		else
		{
			fn_1_trigger_flag = 0;
		}
		
		//////////////////////////////////        ///////////////////////////////////////////////////
		if(vtm_remote_data.pause == 1)
		{
			if(pause_flag == 0)
			{
				pause_cnt++;
				pause_flag = 1;
			}
		}
		else
		{
			pause_flag = 0;
		}
		if(pause_cnt %2 == 0)
		{
			pause_trigger_flag = 0;
		}
		else
		{
			pause_trigger_flag = 1;
		}
		//////////////////////////////////        ///////////////////////////////////////////////////
		if(vtm_remote_data.fn_2 == 1)
		{
			if(fn_2_flag == 0)
			{
				fn_2_cnt++;
				fn_2_flag = 1;
			}
		}
		else
		{
			fn_2_flag = 0;
		}
		if(fn_2_cnt %2 ==0)
		{
			fn_2_trigger_flag = 0;
		}
		else
		{
			fn_2_trigger_flag = 1;
		}
		
		if(vtm_remote_data.trigger == 1)
		{
			if(trigger_flag1 == 0)
			{
				trigger_cnt++;
				trigger_flag1 = 1;
			}
		}
		else
		{
			trigger_flag1 = 0;
		}
		if(trigger_cnt %2 ==0)
		{
			trigger_flag = 0;
		}
		else
		{
			trigger_flag = 1;
		}
	
	}
		
}
