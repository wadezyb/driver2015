/**
  ******************************************************************************
  * @file    calibration.c
  * @author  ZHAN Yubo
  * @version V9.0.0
  * @date    03-May-2014
  * @brief   This file is calibration for STM32F405RGT6
  *
*/

#include "includes.h"
#include "calibration.h"
#include "encoder.h"
#include "pwm.h"
#include "ad.h"
#include "canopen.h"
#include "ploop.h"
#include "cloop.h"
#include "as5047d.h"
int pos[10];
int offset[10];
int sum = 0;
int averageOffset = 0;
/* Find Zero Pos */
void calibrateRotorZeroPos( void )
{
	int A = T_PWM/8;
	int angle = 0;
	float theta = 0;
	int i,j;
	int p[]={0,-512,-512*2,-512*3,-512*4,-512*5,-512*6,-512*7};
	// Enable PWM
	PWMON();
	
	// Set 0 SVPWM
	runSVPWM(A,theta);
	vTaskDelay(2000);
	pos[0] = getEncoderValue()/4;
	vTaskDelay(100);
	
	// Record data
	for( j=0;j<8;j++ )
	{
		for(i=0;i<900;i++)
		{
			vTaskDelay(1);
			angle = angle + 1;
			theta = angle*PI/1800;
			runSVPWM(A,theta);
		}
		vTaskDelay(500);
		pos[j] = getEncoderValue()/4;
		vTaskDelay(100);	
	}
	PWMOFF();
	
	// Calc the Offset
	for(i=0;i<8;i++)
	{
		offset[i] = (pos[i]-p[i])%4096;
		sum += offset[i];
	}
	averageOffset = sum/8;
}


/**
  * @brief  Calibrate the encoder
  * @param  None
  * @retval None
  */
void EncCalibrate( void )
{
//	int temp = getEncoderValue();
//	TIM3->CNT = (temp)*INC_ENC_REVOLUTION/4096;
}

/**
  * @brief  Calibration Task
  * @param  None
  * @retval None
  */
void calibrationTask( void *pvParameters )
{
	vTaskDelay(100);
	//EncCalibrate();
	CurrentOffsetCalibration();
	//calibrateRotorZeroPos();
	StatusWord.Param |= STATUSWORD_BIT0;
	for(;;)
	{
		vTaskDelete(NULL);
	}
}

/**
  * @brief  Current Sampling Offset Calibration 
  * @param  None
  * @retval None
  */
void CurrentOffsetCalibration( void )
{
	int i=0;
	int sum1 = 0;
	int sum2 = 0;
	int n = 20;
	for(i=0;i<n;i++)
	{
		sum1 += ADC1->JDR2;
		sum2 += ADC1->JDR1;
		vTaskDelay(1);
	}
	Current.IA_Offset = sum1/n;
	Current.IB_Offset = sum2/n;
}


/*========================== END OF FILE =======================================*/

