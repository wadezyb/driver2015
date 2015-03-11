/**
  ******************************************************************************
  * @file    encoder.c
  * @author  ZHAN Yubo
  * @version V9.0.0
  * @date    03-May-2014
  * @brief   This file is encoder driver for STM32F405RGT6
  *
*/

/* INCLUDES */
#include "includes.h"
#include "encoder.h"
#include "canopen.h"
#include "vloop.h"
#include "as5047d.h"

/* Private Define */
#define PI (3.1416)

/* Public Varables */
encoderReadType Encoder; 	// for increamental encoder
absObj absEnc;						// for absolute encoder
/*
	* ENCODER INTERFACE:
	* EA->PA6->TIM3-CH1
	* EB->PC7->TIM3-CH2
	* ES->PC12
	*/

/**
  * @brief  Initialize the Encoder parameter to zero.
  * @param  None
  * @retval None
  */
void EncoderObjInit( void )
{
	Encoder.LastP = 0;
	Encoder.LastV = 0;
	Encoder.DP = 0;
	Encoder.V = 0;
	Encoder.CNT32 = 0;
	Encoder.Multi = 0;
	Encoder.Single = 0;
	Encoder.Value = 0;
}

/**
  * @brief  Initialize the TIM6 to a timer which updates every 1ms
  * @param  None
  * @retval None
  */
void TIM6_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE);	       

	TIM_TimeBaseStructure.TIM_Period = (1680-1); 						
	TIM_TimeBaseStructure.TIM_Prescaler =10-1; 	              
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; 		
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); 

	TIM_ClearFlag(TIM6, TIM_FLAG_Update); 	                   
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);                 
	TIM_Cmd(TIM6, ENABLE); 		                                 

	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_SetPriority(TIM6_DAC_IRQn,15);
}
/**
  * @brief  Initialize the Encoder Interface
  * @param  None
  * @retval None
	* @note 	PB0->EA
						PB1->EB
						PC11->EZ
						TIM3 FOR ABZ ENCODER
  */
void Encoder_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	/* Set the clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 ,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 ,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA+RCC_AHB1Periph_GPIOB+ RCC_AHB1Periph_GPIOC,ENABLE);

//	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	NVIC_SetPriority(TIM3_IRQn,15);
	
	/* Set the IO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0+GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM3);

	/* Set the Encoder Mode */
	TIM_SetAutoreload(TIM3,INC_ENC_REVOLUTION-1);
	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Falling,TIM_ICPolarity_Falling);
	TIM_Cmd(TIM3,ENABLE);
	
	//
	TIM3->CNT = (getEncoderValue()+1725+1000)/4*2000/4096%2000;//2116//1725
	
	EncoderObjInit();
	TIM6_Configuration();
	//USART4_Configuration();
	
}	
/**
  * @brief  Configure the UART4 Serial Port as the absolute encoder interface
	* @note   Rx Pin:  PC11
	*					Interupt Priority 15
	*					BaudRate: 9600
  * @param  None
  * @retval None
  */
void USART4_Configuration( void )
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
		USART_ClockInitTypeDef USART_ClockInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); /* 开启对应的IO时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);/* 开启串口1时钟 */

    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;    /* 选择IRQ通道 */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  /* 设置成员NVIC_IRQChannel的先占优先级 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         /* 设置成员NVIC_IRQChannel的从优先级 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            /* 使能中断功能 */
    NVIC_Init(&NVIC_InitStructure);
		NVIC_SetPriority(UART4_IRQn,15);
    

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);
	
    USART_ClockInitStructure.USART_Clock = USART_Clock_Enable;
		USART_ClockInitStructure.USART_CPHA = USART_CPHA_1Edge;
		USART_ClockInitStructure.USART_CPOL = USART_CPOL_High;
		USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
		USART_ClockInit(UART4,&USART_ClockInitStructure);
		
		USART_DeInit(UART4); /* 复位USART4 */
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_Mode = USART_Mode_Rx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
    USART_Init(UART4,&USART_InitStructure);
    USART_ITConfig(UART4,USART_IT_RXNE,ENABLE);/* 开启接收中断 */
		USART_Cmd(UART4,ENABLE);
		absEnc.counter = 0;
		absEnc.num = 0;
		absEnc.flag = 0;
		absEnc.sign = 1;
		absEnc.multi = 0;
		absEnc.single = 0;
		//USART3->DR = 0Xaa;
}	

/**
  * @brief  UART4_IRQHandler for abs encoder data receive
  * @param  None
  * @retval None
  */
void UART4_IRQHandler(void)
{
	char inData;
	
	USART_ClearITPendingBit(UART4,USART_IT_RXNE);
	USART_ClearFlag(UART4,USART_FLAG_TC);
	USART_ClearFlag(UART4,USART_FLAG_RXNE);
	inData = UART4->DR;
	//USART4->DR = inData;
	if(inData == 0x8D)
	{
		absEnc.flag |= 0x01;
	}

	if((absEnc.flag&0x01) == 0x01)
	{
		absEnc.buf[absEnc.num] = inData;
		absEnc.num++;
		if(absEnc.num == 13)
		{
			absEnc.num = 0;
			absEnc.flag &= ~ 0x01;
			if( absEnc.buf[2] == 0x2d )
			{
				absEnc.sign = +1;
			}
			else if( absEnc.buf[2] == 0x2b )
			{
				absEnc.sign = -1;
			}
			if(absEnc.buf[8] == 0xac)
			{
				absEnc.multi = ((absEnc.buf[5]&0x0f)*1000 + (absEnc.buf[5]&0x0f)*100 + (absEnc.buf[6]&0x0f)*10 + (absEnc.buf[7]&0x0f));
				absEnc.single =((absEnc.buf[9]&0x0f)*1000 + (absEnc.buf[10]&0x0f)*100 + (absEnc.buf[11]&0x0f)*10 + (absEnc.buf[12]&0x0f));
				absEnc.value = absEnc.sign*absEnc.multi*4096 - absEnc.single;
			}	
		}
	}		
}


/**
  * @brief  TIM6_DAC_IRQHandler at 1k Hz			
  * @param  None
  * @retval None
  */
void TIM6_DAC_IRQHandler(void)
{
	int temp;
	if(TIM6->SR & TIM_SR_UIF)
	{
		/*清中断标志位*/
		TIM6->SR &= ~TIM_SR_UIF;
		/* 编码器计数 */
		temp = TIM3->CNT;
		Encoder.Single = (temp&0x8000)?(temp|0xffff0000):(temp);//把16位转换成32位
		Encoder.DP = Encoder.Single - Encoder.LastP;			//当前位置与上一次位置做差 求出位置变化量 
		Encoder.LastP = Encoder.Single;									//记录这一次的位置
		if( Encoder.DP > INC_ENC_REVOLUTION/2)												//变化量大于最大变化量 翻转
		{
			Encoder.Multi -= 1;
			Encoder.V = Encoder.DP - INC_ENC_REVOLUTION;
			//USART3->DR = (char)(Encoder.V&0XFF);
		}
		else if( Encoder.DP < -INC_ENC_REVOLUTION/2)									//正转
		{
			Encoder.Multi += 1;
			Encoder.V = Encoder.DP + INC_ENC_REVOLUTION;
			//USART3->DR = (char)(Encoder.V&0XFF);
		}
		else
			Encoder.V = Encoder.DP;											//编码器的变化量就是速度
		
		Encoder.LastV = Encoder.V;
		Encoder.Value = Encoder.Multi*INC_ENC_REVOLUTION + Encoder.Single;
		
		if( (ControlWord.Param&CONTROLWORD_SWITCH_ON) == CONTROLWORD_SWITCH_ON)
		{		
			if( 
					(ModesofOperation.Param == PROFILE_POSITION_MODE) 
				|| (ModesofOperation.Param == VELOCITY_MODE) 
				|| (ModesofOperation.Param == PROFILE_VELOCITY_MODE) 
				|| (ModesofOperation.Param == INTERPOLATED_POSITION_MODE) 
				|| (ModesofOperation.Param == HOMING_MODE) 
			)
				vloopControl();
		}
	}
}
/*========================== END OF FILE =======================================*/
