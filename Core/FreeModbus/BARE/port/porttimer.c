/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "stm32g0xx_hal.h"
#include "tim.h"
#include "adcSampling.h"
#include "MainControl.h"
/* ----------------------- static functions ---------------------------------*/
// static void prvvTIMERExpiredISR( void );
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim16;


extern void adcCallback(void);
extern void dwMotorCallback(void);
/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
	MX_TIM14_Init();
	return TRUE;
}


void
vMBPortTimersEnable(  )
{
	//__HAL_TIM_SET_COUNTER(&htim14, 0);		// ��ռ�����
  //__HAL_TIM_ENABLE(&htim14);				// ʹ�ܶ�ʱ��
	
	__HAL_TIM_CLEAR_IT(&htim14,TIM_IT_UPDATE);	
	//__HAL_TIM_ENABLE_IT(&htim14,TIM_IT_UPDATE);
  __HAL_TIM_SetCounter(&htim14,0);
//	__HAL_TIM_ENABLE(&htim14);
		HAL_TIM_Base_Start_IT(&htim14);
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */

}

void
vMBPortTimersDisable(  )
{
 	HAL_TIM_Base_Stop_IT(&htim14);
	//__HAL_TIM_DISABLE(&htim14);				// ���ܶ�ʱ��
 	__HAL_TIM_SetCounter(&htim14,0);
	//__HAL_TIM_DISABLE_IT(&htim14,TIM_IT_UPDATE);
	__HAL_TIM_CLEAR_IT(&htim14,TIM_IT_UPDATE); 
    /* Disable any pending timers. */
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
static void prvvTIMERExpiredISR( void )
{
    ( void )pxMBPortCBTimerExpired(  );
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{	
  	if(htim->Instance == TIM16) {

		//将步进电机控制放在中断函数中，调高定时器中断的优先级
		adcCallback();
        dwMotorCallback();

	}
	else if(htim->Instance == TIM14)
		prvvTIMERExpiredISR();
}

