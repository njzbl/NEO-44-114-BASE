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

__IO uint32_t mOSTM16_SysTick10us_A = 0;
__IO uint32_t mOSTM16_SysTick10us_K = 0;
__IO uint32_t mOSTM16_SysTick10ms_K = 0;
__IO uint32_t mOSTM16_SysTick10us_M = 0;
__IO uint32_t mOSTM16_SysTick10us_CMD510B_M_A = 0;
__IO uint32_t mOSTM16_SysTick10us_CMD510B_M_B = 0;
__IO uint32_t mOSTM16_SysTick10us_M_C = 0;
__IO uint32_t mAddFgVal = 0;
__IO uint8_t mDebugFlagPowerDownCMD510BA[5];
__IO uint8_t mDebugFlagPowerDownCMD510BB[5];

#define THRESHOLD_LEN                   20
#define THRESHOLD_LEN_DEC_1          (THRESHOLD_LEN - 1)

#if (MOTOR_MODEL == CHENXIN_5840_3650)
// const uint32_t FG_THRESHOLD[THRESHOLD_LEN] = {50000,50000,15000,12000,10000,8000,5000,5000,5000,5000
//                                 ,500,5000,5000,5000,5000,5000,5000,5000,5000,500};      //400阈值对应电流： 230 ~ 342 （438mA ~ 651mA）
const uint32_t FG_THRESHOLD[THRESHOLD_LEN] = {2400,1800,1500,1200,1000,800,500,500,500,500
                                ,500,500,500,500,500,500,500,500,500,500};      //400阈值对应电流： 230 ~ 342 （438mA ~ 651mA）
#endif
extern void adcCallback(void);
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
		mOSTM16_SysTick10us_A++;
		mOSTM16_SysTick10us_M++;
		mOSTM16_SysTick10us_K++;
		mOSTM16_SysTick10us_CMD510B_M_A++;
		mOSTM16_SysTick10us_CMD510B_M_B++;
		mOSTM16_SysTick10us_M_C++;

		//将步进电机控制放在中断函数中，调高定时器中断的优先级
		adcCallback();
        
		if(mMachineSta.activation == RUN_STA) {
            mAddFgVal = 0;   //50    //(采用PWM方式供电) 数组 ： 350 ， mAddFgVal = 150 ，电机运行约 300 次，出现1台进风电机打齿。
            
            // if(mDoorRunNumSta >= 2) {       //已经在常温下回到了正常关闭百叶的状态
            //     if(mKeySta.nowKeySta == OPEN_DOOR) {
            //         if(mDoorSta.nowDoorPositionCMD510BMA < (FARTHEST_POSITION_DC_B_MOTOR - 100))
            //             mAddFgVal = 5000;//mAddFgVal = 0;//
            //         else
            //             mAddFgVal = 0;
            //     }
            //     else {
            //         if(mDoorSta.nowDoorPositionCMD510BMA < 50)
            //             mAddFgVal = 0;
            //         else 
            //             mAddFgVal = 5000;//mAddFgVal = 0;//
            //     }
            // }

			if((mCount.motorCMD510BRunStaA >= THRESHOLD_LEN_DEC_1 && mOSTM16_SysTick10us_CMD510B_M_A >= (FG_THRESHOLD[THRESHOLD_LEN_DEC_1] + mAddFgVal)) 
            || (mOSTM16_SysTick10us_CMD510B_M_A >= (FG_THRESHOLD[mCount.motorCMD510BRunStaA] + mAddFgVal) && (mCount.motorCMD510BRunStaA < THRESHOLD_LEN_DEC_1 && mCount.motorCMD510BRunStaA > 0)) 
            || (mCount.motorCMD510BRunStaA == 0 && mOSTM16_SysTick10us_CMD510B_M_A > 2600)) {	//10ms  If the pulse is not received for a long time, it means that the motor is stuck.
                if(mDebugFlagPowerDownCMD510BA[0] == 0) {
#if (MOTOR_MODEL == CHENXIN_5840_3650)
                    setBDCMotorStop(0);	//堵转的时候，可能也有脉冲。要测试  //   ZBL  20250418
#endif
                    mDebugFlagPowerDownCMD510BA[0] = 1;
                    // printf("PowerDownShadesMotorAAA() mCount.motorCMD510BRunStaA = %d mOSTM16_SysTick10us_CMD510B_M_A = %d\r\n",mCount.motorCMD510BRunStaA,mOSTM16_SysTick10us_CMD510B_M_A);
                }
				mOSTM16_SysTick10us_CMD510B_M_A = 0;
				if(mKeySta.nowKeySta == CLOSE_DOOR) {		// 堵转停止说明是到了关门极限位置，如果是门被卡住，则有漏洞，但不加此逻辑结构件会出现百叶越开越小的问题
					mDoorSta.nowDoorPositionCMD510BMA = 0;   //ZBL 20250826   低温实验用的临时屏蔽代码
				}
			}
			if((mCount.motorCMD510BRunStaB >= THRESHOLD_LEN_DEC_1 && mOSTM16_SysTick10us_CMD510B_M_B >= (FG_THRESHOLD[THRESHOLD_LEN_DEC_1] + mAddFgVal))
            || (mOSTM16_SysTick10us_CMD510B_M_B >= (FG_THRESHOLD[mCount.motorCMD510BRunStaB] + mAddFgVal) && (mCount.motorCMD510BRunStaB < THRESHOLD_LEN_DEC_1 && mCount.motorCMD510BRunStaB > 0))
            || (mCount.motorCMD510BRunStaB == 0 && mOSTM16_SysTick10us_CMD510B_M_B > 2600)) {	//power up 25ms，FG is 0，

                if(mDebugFlagPowerDownCMD510BB[0] == 0) {
#if (MOTOR_MODEL == CHENXIN_5840_3650)
				    setBDCMotorStop(1);  //   ZBL  20250418
#endif
                    mDebugFlagPowerDownCMD510BB[0] = 1;
                    // printf("PowerDownShadesMotorBBB()  mDebugFlagPowerDownB[0] = 1  mCount.motorCMD510BRunStaB = %d\r\n",mCount.motorCMD510BRunStaB);
                }

				mOSTM16_SysTick10us_CMD510B_M_B = 0;
				if(mKeySta.nowKeySta == CLOSE_DOOR) {		// 堵转停止说明是到了关门极限位置，如果是门被卡住，则有漏洞，但不加此逻辑结构件会出现百叶越开越小的问题
					mDoorSta.nowDoorPositionCMD510BMB = 0;   //ZBL 20250826   低温实验用的临时屏蔽代码
				}
			}
		}


	}
	else if(htim->Instance == TIM14)
		prvvTIMERExpiredISR();
}

