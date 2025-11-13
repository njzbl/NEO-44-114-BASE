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

__IO uint32_t mOSTM16_SysTick20us_A = 0;
__IO uint32_t mOSTM16_SysTick20us_K = 0;
__IO uint32_t mOSTM16_SysTick10ms_K = 0;
__IO uint32_t mOSTM16_SysTick20us_CMD510B_M_A = 0;
__IO uint32_t mOSTM16_SysTick20us_CMD510B_M_B = 0;
__IO uint32_t mAddFgVal = 0;
__IO uint8_t mDebugFlagPowerDownCMD510BA[5];
__IO uint8_t mDebugFlagPowerDownCMD510BB[5];

#define THRESHOLD_LEN                   30
#define THRESHOLD_LEN_DEC_1          (THRESHOLD_LEN - 1)

#if (MOTOR_MODEL == CHENXIN_5840_3650)
// const uint32_t FG_THRESHOLD[THRESHOLD_LEN] = {25000,25000,7500,6000,5000,4000,2500,2500,2500,2500
//                                 ,250,250,250,250,250,250,250,250,250,250};      //400阈值对应电流： 230 ~ 342 （438mA ~ 651mA）
// const uint32_t FG_THRESHOLD[THRESHOLD_LEN] = {1200,900,750,600,500,400,250,250,250,250
//                                 ,250,250,250,250,250,250,250,250,250,250};      //400阈值对应电流： 230 ~ 342 （438mA ~ 651mA）
const uint32_t FG_THRESHOLD[THRESHOLD_LEN] = {1200,1200,1200,1200,1200,1200,1200,1200,1200,1200
                                ,1200,1200,1200,1200,1200,1200,1200,1200,1200,1200
								,1200,1200,1200,1200,1200,1200,1200,1200,1200,300};
#endif

//行程20mm 速度9mm/s  FG信号不用上拉电阻，5V输出
// 迪洛克防爆电机 一个周期约 0.95ms ~ 1.0 ms   //第1个周期13ms ,第2个周期12ms ,第3个周期11ms ,第4个周期9ms ,第5个周期5ms ,
#if (MOTOR_MODEL == DLK_YLSZ23)   
const uint32_t FG_THRESHOLD[THRESHOLD_LEN] = {1000,800,800,700,500,400,350,250,150,125         // 400350DW 使用万融的电机，一个周期约1.75ms  , 第一个周期约5.4ms ，第二个周期约4.8ms
                                 	,125,125,125,125,125,125,125,125,125,125
									,125,125,125,125,125,125,125,125,125,125};           //220
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
		mOSTM16_SysTick20us_A++;
		mOSTM16_SysTick20us_K++;
		mOSTM16_SysTick20us_CMD510B_M_A++;
		mOSTM16_SysTick20us_CMD510B_M_B++;

		//将步进电机控制放在中断函数中，调高定时器中断的优先级
		adcCallback();

#if (MOTOR_MODEL == CHENXIN_5840_3650 || MOTOR_MODEL == DLK_YLSZ23)

		if(mMachineSta.activation == RUN_STA) {
            mAddFgVal = 0;   //50    //(采用PWM方式供电) 数组 ： 350 ， mAddFgVal = 150 ，电机运行约 300 次，出现1台进风电机打齿。
            
//>>>>>>>>>>>>>>>>>>>  中途提高驱动力的处理  >>>>>>>>>>>>>>>>>>>
            if(mKeySta.nowKeySta == OPEN_DOOR) {
                if(mCount.motorCMD510BRunStaA < (FARTHEST_POSITION_DC_B_MOTOR - 100))
                    mAddFgVal = 100;    //庆城是400关紧  600动作，所以 mAddFgVal = 200;   增强齿轮是500关紧 800动作，所以 mAddFgVal = 300;
                else
                    mAddFgVal = 0;
            }
            else {
                if(mCount.motorCMD510BRunStaA < 500 && mDoorSta.nowDoorPositionCMD510BMA > 100)
                    mAddFgVal = 100;    //庆城是400关紧  600动作，所以 mAddFgVal = 200;   增强齿轮是500关紧 800动作，所以 mAddFgVal = 300;
                else 
                    mAddFgVal = 0;
            }
//<<<<<<<<<<<<<<<<<<<  中途提高驱动力的处理  <<<<<<<<<<<<<<<<<<<

			if((mCount.motorCMD510BRunStaA >= THRESHOLD_LEN_DEC_1 && mOSTM16_SysTick20us_CMD510B_M_A >= (FG_THRESHOLD[THRESHOLD_LEN_DEC_1] + mAddFgVal)) 
            || (mOSTM16_SysTick20us_CMD510B_M_A >= (FG_THRESHOLD[mCount.motorCMD510BRunStaA] + mAddFgVal) && (mCount.motorCMD510BRunStaA < THRESHOLD_LEN_DEC_1 && mCount.motorCMD510BRunStaA > 0)) 
            || (mCount.motorCMD510BRunStaA == 0 && mOSTM16_SysTick20us_CMD510B_M_A > 1300)) {	//10ms  If the pulse is not received for a long time, it means that the motor is stuck.
                if(mDebugFlagPowerDownCMD510BA[0] == 0) {
#if (MOTOR_MODEL == CHENXIN_5840_3650)
                    setBDCMotorStop(0);	//堵转的时候，可能也有脉冲。要测试  //   ZBL  20250418
#endif
                    mDebugFlagPowerDownCMD510BA[0] = 1;
                    // printf("PowerDownShadesMotorAAA() mCount.motorCMD510BRunStaA = %d mOSTM16_SysTick20us_CMD510B_M_A = %d\r\n",mCount.motorCMD510BRunStaA,mOSTM16_SysTick20us_CMD510B_M_A);
                }
				mOSTM16_SysTick20us_CMD510B_M_A = 0;
				if(mKeySta.nowKeySta == CLOSE_DOOR) {		// 堵转停止说明是到了关门极限位置，如果是门被卡住，则有漏洞，但不加此逻辑结构件会出现百叶越开越小的问题
					mDoorSta.nowDoorPositionCMD510BMA = 0;
				}
			}			
//>>>>>>>>>>>>>>>>>>>  中途提高驱动力的处理  >>>>>>>>>>>>>>>>>>>

            if(mKeySta.nowKeySta == OPEN_DOOR) {
                if(mCount.motorCMD510BRunStaB < (FARTHEST_POSITION_DC_B_MOTOR - 100))
                    mAddFgVal = 100;    //庆城是400关紧  600动作，所以 mAddFgVal = 200;   增强齿轮是500关紧 800动作，所以 mAddFgVal = 300;
                else
                    mAddFgVal = 0;
            }
            else {
                if(mCount.motorCMD510BRunStaB < 500 && mDoorSta.nowDoorPositionCMD510BMB > 100)
                    mAddFgVal = 100;    //庆城是400关紧  600动作，所以 mAddFgVal = 200;   增强齿轮是500关紧 800动作，所以 mAddFgVal = 300;
                else 
                    mAddFgVal = 0;
            }

//<<<<<<<<<<<<<<<<<<<  中途提高驱动力的处理  <<<<<<<<<<<<<<<<<<<
			if((mCount.motorCMD510BRunStaB >= THRESHOLD_LEN_DEC_1 && mOSTM16_SysTick20us_CMD510B_M_B >= (FG_THRESHOLD[THRESHOLD_LEN_DEC_1] + mAddFgVal))
            || (mOSTM16_SysTick20us_CMD510B_M_B >= (FG_THRESHOLD[mCount.motorCMD510BRunStaB] + mAddFgVal) && (mCount.motorCMD510BRunStaB < THRESHOLD_LEN_DEC_1 && mCount.motorCMD510BRunStaB > 0))
            || (mCount.motorCMD510BRunStaB == 0 && mOSTM16_SysTick20us_CMD510B_M_B > 1300)) {	//power up 25ms，FG is 0，

                if(mDebugFlagPowerDownCMD510BB[0] == 0) {
#if (MOTOR_MODEL == CHENXIN_5840_3650)
				    setBDCMotorStop(1);  //   ZBL  20250418
#endif
                    mDebugFlagPowerDownCMD510BB[0] = 1;
                    // printf("PowerDownShadesMotorBBB()  mDebugFlagPowerDownB[0] = 1  mCount.motorCMD510BRunStaB = %d\r\n",mCount.motorCMD510BRunStaB);
                }

				mOSTM16_SysTick20us_CMD510B_M_B = 0;
				if(mKeySta.nowKeySta == CLOSE_DOOR) {		// 堵转停止说明是到了关门极限位置，如果是门被卡住，则有漏洞，但不加此逻辑结构件会出现百叶越开越小的问题
					mDoorSta.nowDoorPositionCMD510BMB = 0;
				}
			}
		}

#endif

	}
	else if(htim->Instance == TIM14)
		prvvTIMERExpiredISR();
}

