/**
  ******************************************************************************
  * @file    vloop.c
  * @author  ZHAN Yubo
  * @version V9.0.0
  * @date    03-May-2014
  * @brief   This file is Velocity Loop Control Algorithm
  *
*/

#include "vloop.h"
#include "includes.h"
#include "encoder.h"
#include "cloop.h"

vLoopObj Velocity;

int V[1000];
int Vo[1000];
int Vn=0;
int vloopTuningStartFlag=0;
int Vt=0;

void vPI_Init( void )
{
	Velocity.vPI.Hz = 1000;
	Velocity.vPI.kp = 12000;//
	Velocity.vPI.ki = 300;//
	Velocity.vPI.scale = 100;
	Velocity.vPI.output_max = 2000;
	Velocity.vPI.sum_error_max = Velocity.vPI.output_max*Velocity.vPI.scale;
}

void vloopTuning( void )
{
	if( Vn < 1000 )
	{
		V[Vn] = Velocity.V;
		//Vo[Vn] = Velocity.vPI.output;
		Vn++;
	}
	else
	{
		vloopTuningStartFlag=0;
	}
	if( (Vn<=100) || (Vn>900))
		Velocity.targetVelocity = 0;
	else
		Velocity.targetVelocity = Vt;

}

void vloopControl( void )
{
	// Read Velocity Feedback Value
	Velocity.V = Encoder.V;
	// Calc the error
	Velocity.vPI.error = Velocity.targetVelocity - Velocity.V;
	// Calc the sum of error
	Velocity.vPI.sum_error += Velocity.vPI.error*Velocity.vPI.ki;
	// error sum limitation
	if( Velocity.vPI.sum_error >  Velocity.vPI.sum_error_max)
		Velocity.vPI.sum_error = Velocity.vPI.sum_error_max;
	else if( Velocity.vPI.sum_error < -Velocity.vPI.sum_error_max)
		Velocity.vPI.sum_error = -Velocity.vPI.sum_error_max;
	// Calc the PI output
	Velocity.vPI.output = (Velocity.vPI.kp*Velocity.vPI.error + Velocity.vPI.sum_error )/Velocity.vPI.scale;
	// PI output limitation
	if( Velocity.vPI.output > Velocity.vPI.output_max )
		Velocity.vPI.output = Velocity.vPI.output_max;
	else if( Velocity.vPI.output < -Velocity.vPI.output_max )
		Velocity.vPI.output = -Velocity.vPI.output_max;
	// Send Current Command
	Current.targetCurrent = -Velocity.vPI.output;
	if( vloopTuningStartFlag == 1 )
		vloopTuning();
}

/*========================== END OF FILE =======================================*/
