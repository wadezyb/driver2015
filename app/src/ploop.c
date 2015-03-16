/**
  ******************************************************************************
  * @file    ploop.c
  * @author  ZHAN Yubo
  * @version V9.0.0
  * @date    03-May-2014
  * @brief   This file is for Position Loop Control
  *
*/
#include "ploop.h"
#include "includes.h"
#include "pid.h"
#include "vloop.h"
#include "encoder.h"
#include "canopen.h"
#include "can.h"
#include "sci.h"
#include "cli.h"
#include "interpolation.h"
#include "pwm.h"

#define Pi (float)(3.1415926)
#define L0 (150.0)
#define L1 (225.0)
#define L2 (125.0)
#define h   (15.0/2/Pi)// mm/rad
#define h3 (4096*0.0032552/2.0/Pi)// mm/rad
#define MaxV (30.0)  // pulse/ms

pLoopObj Position;
int sendFlag = 1;

int Pp[1000];
int Pv[1000];
int Pe[1000];
int Po[1000];
int Pn;
int pTuningStartFlag;
TrajectoryObj tra;

void ploopTuning( void )
{
	
	if( Pn < 1000 )
	{
		Pp[Pn] = Position.targetPosition;	
		Pv[Pn] = Position.targetVelocity;
		Pe[Pn] = Encoder.Value;
		Po[Pn] = Encoder.V;
		Pn++;
	}
	else
	{
		pTuningStartFlag=0;
	}
}

void ploopInit( void )
{
	Position.pPD.kp = 15;
	Position.pPD.ki = 0;
	Position.pPD.kd = 5;
	Position.pPD.scale = 100;
	Position.pPD.output_max = 30;
	Position.pPD.error_sum_max = Position.pPD.output_max*Position.pPD.scale;
}

void ploopControl( void )
{
	// Read Position Value
	Position.P = Encoder.Value;
	// Calc the position error.
	Position.pPD.error = Position.targetPosition - Position.P;
	// Calc the d_error
	Position.pPD.d_error = Position.pPD.error - Position.pPD.last_error;
	// Remember error to last error
	Position.pPD.last_error = Position.pPD.error;
	// Calc the sum of error
	Position.pPD.error_sum += Position.pPD.ki*Position.pPD.error;
	// error sum limite
	if( Position.pPD.error_sum > Position.pPD.error_sum_max )
		Position.pPD.error_sum = Position.pPD.error_sum_max;
	else if( Position.pPD.error_sum < -Position.pPD.error_sum_max )
		Position.pPD.error_sum = -Position.pPD.error_sum_max;
	// Calc the PD output
	Position.pPD.output = (Position.pPD.kp*Position.pPD.error + Position.pPD.error_sum + Position.pPD.kd*Position.pPD.d_error )/Position.pPD.scale+ Position.targetVelocity;//;
	// output limitation
	if( Position.pPD.output > Position.pPD.output_max )
		Position.pPD.output = Position.pPD.output_max;
	else if( Position.pPD.output < -Position.pPD.output_max )
		Position.pPD.output = -Position.pPD.output_max;
	
	// apply the output to vloop
	Velocity.targetVelocity = Position.pPD.output;
	if( pTuningStartFlag == 1 )
		ploopTuning();	
	
}


void ploopTask( void *pvParameters )
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1;	//1KHz
	ploopInit();
	//vTaskSuspend(NULL);
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		if( (ControlWord.Param&CONTROLWORD_SWITCH_ON) == CONTROLWORD_SWITCH_ON)
		{
			ploopControl();
		}
		else
		{
			Position.targetPosition = Encoder.Value;
		}
	}
}


void sendPositionTask( void *pvParameters )
{
	TickType_t xLastWakeTime;
//	char stringBuf[20];
	xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
		vTaskDelayUntil( &xLastWakeTime, 100 );
		//vTaskDelay(1000);
		#if (CAN_NODE != CAN_NODE5)
		{
//			ActualPosition.Param = Encoder.Value;
//			CAN_send(0x05,POSITION_ACTUAL_VALUE_INDEX,0,ActualPosition.Param);
		}
		#else
		{
			if(sendFlag ==1)
			{
				sendFlag = 0;
				sciSendString("A1 ",bMessage);
				sciSendString(int2String(Joint1.ActualPosition.Param,stringBuf),bMessage);
				sciSendString("\r\n",bMessage);
				vTaskDelay(5);
				sciSendString("A2 ",bMessage);
				sciSendString(int2String(Joint2.ActualPosition.Param,stringBuf),bMessage);
				sciSendString("\r\n",bMessage);
				vTaskDelay(5);
				sciSendString("A3 ",bMessage);
				sciSendString(int2String(Joint3.ActualPosition.Param,stringBuf),bMessage);
				sciSendString("\r\n",bMessage);
				vTaskDelay(5);
				sciSendString("A4 ",bMessage);
				sciSendString(int2String(Joint4.ActualPosition.Param,stringBuf),bMessage);
				sciSendString("\r\n",bMessage);
				vTaskDelay(5);
				sciSendString("A5 ",bMessage);
				sciSendString(int2String(-Encoder.Value,stringBuf),bMessage);
				sciSendString("\r\n",bMessage);
				vTaskDelay(5);			
			}

		}
		#endif
		
	}
}

void MotionPlanningTask( void *pvParameters )
{
	TickType_t xLastWakeTime;
	float T = 2000;
	float w = 2*Pi/T;
	float K = 52.0;
	float resolution = 2*Pi/2000/K; // rad per count
	float A = Pi/resolution;
	int offset;
	int t = 0;
	int counter = 0;
	char stringBuf[20];
	int angle = 0;
	ModesofOperation.Param = PROFILE_POSITION_MODE;
	vTaskDelay(500);
	ControlWord.Param|=CONTROLWORD_SWITCH_ON;
	PWMON();	
	vTaskDelay(500);
	xLastWakeTime = xTaskGetTickCount();
	offset = Position.targetPosition;
	for(;;)
	{
		vTaskDelayUntil( &xLastWakeTime, 1 );
		Position.targetPosition = A*cos(w*t)-A+offset;
		//temp = A*sin(w*t)+offset;
		if(t < T)
			t++;
		else
			t = 0;
		if(counter<20)
		{
			counter ++;
		}
		else
		{
			counter = 0;
			angle = (Encoder.Value - offset)*3600/K/2000+1800;
			//sciSendString(int2String(angle,stringBuf),bMessage);
			//sciSendString("\r\n",bMessage);
			sendSerialMessage(angle);
		}
	}		
}
/*========================== END OF FILE =======================================*/

