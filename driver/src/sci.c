/**
  ******************************************************************************
  * @file    sci.c
  * @author  ZHAN Yubo
  * @version V9.0.0
  * @date    03-May-2014
  * @brief   This file is Serial Communication Interface driver for STM32F405RGT6
  *
*/

/* INCLUDES */
#include "includes.h"
#include "sci.h"
#include "cli.h"

/* Public Variable */
xQueueHandle bMessage;

/* Private Variable */
cmdType cmdBuf;

/**
  * @brief  Configuration of Serial Communication Interface
  * @param  None
  * @retval None
  */
void sciConfiguration( void )
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
		USART_ClockInitTypeDef USART_ClockInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); /* 开启对应的IO时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);/* 开启串口1时钟 */
		
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;    /* 选择IRQ通道 */        /* 设置成员NVIC_IRQChannel的从优先级 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            /* 使能中断功能 */
    NVIC_Init(&NVIC_InitStructure);
		NVIC_SetPriority(USART3_IRQn,15);
    
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);		
		
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);
	
    USART_ClockInitStructure.USART_Clock = USART_Clock_Enable;
		USART_ClockInitStructure.USART_CPHA = USART_CPHA_1Edge;
		USART_ClockInitStructure.USART_CPOL = USART_CPOL_High;
		USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
		USART_ClockInit(USART3,&USART_ClockInitStructure);
		
		USART_DeInit(USART3); /* 复位USART3 */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
    USART_Init(USART3,&USART_InitStructure);
    USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);/* 开启接收中断 */
		USART_Cmd(USART3,ENABLE);
		//USART3->DR = 0Xaa;
}


/**
  * @brief  Send a char
	* @param  Data	: unsigned char data to send
	*	@param	messageQueue	: xQueueHandle Type message queue
  * @retval None
  */
void sciSendChar( unsigned char Data, xQueueHandle messageQueue )
{
	unsigned char temp = Data;
	if( messageQueue != 0 )
	{
		if( xQueueSend( messageQueue, ( void * ) &temp, ( TickType_t ) 10 ) != pdPASS )
		{
			// Failed to post the message, even after 10 ticks.
		}		
	}

}

/**
  * @brief  Send a halfword
	* @param  halfword	: data to send
	*	@param	messageQueue	: xQueueHandle Type message queue
  * @retval None
  */
void sciSendHalfword( short halfword, xQueueHandle messageQueue )
{
	unsigned char temp;

	temp = (unsigned char)((halfword&0xff00)>>8);
	sciSendChar( temp, bMessage );
	temp = (unsigned char)(halfword&0xff);
	sciSendChar( temp, bMessage );	
}

/**
  * @brief  Send a word
	* @param  word	: data to send
	*	@param	messageQueue	: xQueueHandle Type message queue
  * @retval None
  */
void sciSendWord ( long word, xQueueHandle messageQueue )
{
	unsigned char temp;

	temp = (unsigned char)((word&0xff000000)>>24);
	sciSendChar( temp, bMessage );
	temp = (unsigned char)((word&0xff0000)>>16);
	sciSendChar( temp, bMessage );
	temp = (unsigned char)((word&0xff00)>>8);
	sciSendChar( temp, bMessage );
	temp = (unsigned char)(word&0xff);
	sciSendChar( temp, bMessage );		
}

/**
  * @brief  Send a string
	* @param  addr	: pointer to the string to send
	*	@param	messageQueue	: xQueueHandle Type message queue
  * @retval None
  */
void sciSendString( char * addr, xQueueHandle messageQueue )
{
	while( *addr != '\0' )
	{
		sciSendChar( *addr, messageQueue );
		addr++;
	}
}

/**
  * @brief  Turn int number to a string.
	* @param  number : The int Number
	* @param 	stringBuf : Pointer to the string
	* @retval char* : Pointer to the string
  */
char* int2String( int number, char* stringBuf )
{
	int temp = 10;
	int n = 1;
	char* stringHead;

	stringHead = stringBuf;
	
	if( number<0 )
	{
		number = -number;
		*stringBuf = '-';
		stringBuf++;
	}
	
	while( (number/temp) != 0 )
	{
		temp = temp*10;
		n++;
	}
	while( n-- )
	{
		temp = temp/10;
		
		switch( (number/temp)%10 )
		{
			case 0: *stringBuf = '0'; break;
			case 1: *stringBuf = '1'; break;
			case 2: *stringBuf = '2'; break;
			case 3: *stringBuf = '3'; break;
			case 4: *stringBuf = '4'; break;
			case 5: *stringBuf = '5'; break;
			case 6: *stringBuf = '6'; break;
			case 7: *stringBuf = '7'; break;
			case 8: *stringBuf = '8'; break;
			case 9: *stringBuf = '9'; break;
			default: break;
		}
		
		stringBuf ++;
	}
	*stringBuf = '\0';

	return stringHead;
}

/**
  * @brief  Turn float number to a string.
	* @param  number : The int Number
	* @param 	resolution: float resolution
	* @param 	stringBuf : Pointer to the string
	* @retval char* : Pointer to the string
  */
char* float2String( float number, unsigned char resolution, char* stringBuf )
{
	char *stringHead = stringBuf;
	char *stringEnd = stringBuf;
	long n = 0;
	long m =0;
	int digit = 0;
	int i=0;
	int temp=0;

	if(number <0)
	{
		number = -number;
		*stringEnd = '-';
		stringEnd++;		
	}
	if(number>65536)
	{
		return 0;
	}
	n=number*(int)pow(10,resolution);
	m=n;
	while(m)
	{
		m=m/10;
		digit++;
	}
	if(digit<=resolution)
		digit = resolution+1;
	for(i=digit;i>0;i--)
	{
		if(i==resolution)
		{
			*stringEnd = '.';
			stringEnd++;
		}
		temp = n%(int)(pow(10,i))/(int)(pow(10,i-1));			
		switch(temp)
		{
			case 0: *stringEnd = '0'; break;
			case 1: *stringEnd = '1'; break;
			case 2: *stringEnd = '2'; break;
			case 3: *stringEnd = '3'; break;
			case 4: *stringEnd = '4'; break;
			case 5: *stringEnd = '5'; break;
			case 6: *stringEnd = '6'; break;
			case 7: *stringEnd = '7'; break;
			case 8: *stringEnd = '8'; break;
			case 9: *stringEnd = '9'; break;
			default: *stringEnd = '-'; break;			
		}
		stringEnd++;
	}
	*stringEnd = '\0';
	return stringHead;
	
}
/**
  * @brief  Turn int number to a hex string.
	* @param  number : The int Number
	* @param 	stringBuf : Pointer to the string
	* @retval char* : Pointer to the string
  */
char* hex2String( long number, char* stringBuf )
{
	int temp = 16;
	int n = 1;
	char* stringHead;

	stringHead = stringBuf;
	
	if( number<0 )
	{
		number = -number;
		*stringBuf = '-';
		stringBuf++;
	}
	
	while( (number/temp) != 0 )
	{
		temp = temp*16;
		n++;
	}
	while( n-- )
	{
		temp = temp/16;
		
		switch( (number/temp)%16 )
		{
			case 0: *stringBuf = '0'; break;
			case 1: *stringBuf = '1'; break;
			case 2: *stringBuf = '2'; break;
			case 3: *stringBuf = '3'; break;
			case 4: *stringBuf = '4'; break;
			case 5: *stringBuf = '5'; break;
			case 6: *stringBuf = '6'; break;
			case 7: *stringBuf = '7'; break;
			case 8: *stringBuf = '8'; break;
			case 9: *stringBuf = '9'; break;
			case 10: *stringBuf = 'A'; break;
			case 11: *stringBuf = 'B'; break;
			case 12: *stringBuf = 'C'; break;
			case 13: *stringBuf = 'D'; break;
			case 14: *stringBuf = 'E'; break;
			case 15: *stringBuf = 'F'; break;

			default: break;
		}
		
		
		stringBuf ++;
	}
	*stringBuf = '\0';

	return stringHead;	
}

/**
  * @brief  USART3_IRQHandler
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void)
{
	char inData;
	
	USART_ClearITPendingBit(USART3,USART_IT_RXNE);
	USART_ClearFlag(USART3,USART_FLAG_TC);
	USART_ClearFlag(USART3,USART_FLAG_RXNE);
	inData = USART3->DR;
	if(inData == '\r')
	{
		while(cmdBuf.head<CMDBUFSIZE)
		{
			cmdBuf.String[cmdBuf.head] = '\0';
			cmdBuf.head++;
		}
		cmdBuf.head = 0;
		cmdBuf.FinishFlag = 1;
	}
	else
	{
		if(inData!='\n')
		{
			cmdBuf.String[cmdBuf.head] = inData;
			cmdBuf.head++;
		}	
		if(cmdBuf.head>=CMDBUFSIZE)
		{
			cmdBuf.BufOverErrorFlag = 1;
			cmdBuf.head = 0;
		}
	}
	
}

/**
  * @brief  Serial Communication Send Data Task
  * @param  None
  * @retval None
  */
void sciSendTask ( void *pvParameters )
{
	unsigned char buf;
	cmdBuf.head =0;
	//sciConfiguration();	
	bMessage = xQueueCreate( 50, sizeof(unsigned char) );

	if( bMessage != 0 )
	{
		for( ;; )
		{
			if( xQueueReceive( bMessage, &buf, 1 ) == pdTRUE )
			{
				while (!(USART3->SR & 0x0080));
				USART3->DR = buf;	
			}
		}
	}
	else
	{
		USART3->DR = 0X55;//队列创建不成功
		vTaskDelete( NULL );
	}
}
/**
  * @brief  Task of Command Line Interface
  * @param  None
  * @retval None
  */
void CLITask ( void *pvParameters )
{
	vTaskDelay(500);
	//sciSendString("\r\nWelcom to Robot Learning System!",bMessage);
	for(;;)
	{
		vTaskDelay(20);
		if(cmdBuf.BufOverErrorFlag == 1)
		{
			cmdBuf.BufOverErrorFlag = 0;
			sciSendString("\r\nCommand is too long!",bMessage);
		}
		/* cmd analysis */
		if( cmdBuf.FinishFlag == 1 )
		{
			cmdBuf.FinishFlag =0;
			findCommandInList(cmdBuf.String);
		}
	}
}
/*========================== END OF FILE =======================================*/
