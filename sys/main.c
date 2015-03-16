/**
  ******************************************************************************
  * @file    main.c
  * @author  ZHAN Yubo
  * @version V9.0.0
  * @date    03-May-2014
  * @brief   This file is the main entrance to the program.
  *
*/

/* INCLUDE */
#include "includes.h"
#include "main.h"
#include "led.h"
#include "pwm.h"
#include "encoder.h"
#include "sci.h"
#include "ad.h"
#include "can.h"
#include "calibration.h"
#include "vloop.h"
#include "ploop.h"
#include "statemonitor.h"
#include "canopen.h"
#include "cloop.h"
#include "interpolation.h"
#include "as5047d.h"
#include "spi.h"
#include "tempture.h"
#include "voltage.h"

static void prvSetupHardware( void );
void startTask ( void *pvParameters );

/* Task Handle */
xTaskHandle ledTaskHandle;
xTaskHandle encoderTaskHandle;
xTaskHandle sciTaskHandle;
xTaskHandle CLITaskHandle;
// Global Task Handle
xTaskHandle ploopTaskHandle;
xTaskHandle interpolationTaskHandle;

/* Task Priority */

int ledTask_PRIORITY				= 1;
int sciSendTask_PRIORITY 		= 6;
int CLITask_PRIORITY				= 5;

//Compile for Debug mode
#ifdef DEBUG_MODE

#endif

/***
	*	Main
	*/  

int main(void)
{
	//SystemCoreClockUpdate();
	prvSetupHardware();

	/* Create the start task */
	xTaskCreate( startTask, "System_start", configMINIMAL_STACK_SIZE*2, NULL, 1, ( TaskHandle_t * ) NULL );

	vTaskStartScheduler();	

	vTaskDelay(100);
	for( ;; )
	{	
	}
	//return 0;
	
}

/**
  * @brief  Initializes the hardware before the RTOS start working.
  * @param  None
  * @retval None
  */
static void prvSetupHardware( void )
{
	// Initialize the Magnetic Encoder
	SPI1_Init();
	Encoder_Configuration();// spi init must before this operation!!!
	
	sciConfiguration();
	//CAN_Configuration();
	pwmConfiguration();
	ADC_Configuration();
	vPI_Init();
	currentLoopInit();
}

/**
  * @brief  Tasks to start at the beginning
  * @param  None
  * @retval None
  */
void startTask ( void *pvParameters )
{
	// These Tasks are run all the way around.
	xTaskCreate( ledTask, "led", configMINIMAL_STACK_SIZE*2, NULL, ledTask_PRIORITY, &ledTaskHandle );
	
	xTaskCreate( sciSendTask, "sciSend", configMINIMAL_STACK_SIZE*2, NULL, sciSendTask_PRIORITY, &sciTaskHandle );
	
	xTaskCreate( CLITask, "CLI", configMINIMAL_STACK_SIZE*5, NULL, CLITask_PRIORITY, &CLITaskHandle );
	
  //xTaskCreate( temptureTestTask, "temptureTestTask", configMINIMAL_STACK_SIZE*2, NULL, 2, NULL );
	
	//xTaskCreate( as5047dTask, "testSVPWMTask", configMINIMAL_STACK_SIZE*2, NULL, 3, NULL );
	
	xTaskCreate( calibrationTask, "Calibration", configMINIMAL_STACK_SIZE, NULL, 7, NULL );	

	xTaskCreate( driveStateManageTask, "driveStateManageTask", configMINIMAL_STACK_SIZE*2, NULL, 2, NULL );

	xTaskCreate( MotionPlanningTask, "MotionPlanningTask", configMINIMAL_STACK_SIZE*3, NULL, 8, NULL );
	
	for( ;; )
	{							
		vTaskDelete(NULL);
	}
}

/*========================== END OF FILE =======================================*/
