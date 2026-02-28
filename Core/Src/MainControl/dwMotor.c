/*****************************************************************************************************************************
 * \file   : dwMotor.c    
 * \module : MainControl
 * \brief  : MainControl module  
 * \project: NEO-400350-DW
 * \author : ZBL
 *============================================================================================================================
 *                                                   Revision control History
 *============================================================================================================================
 * V1.0.0: 2026-02-28: Initial Version
 *
 *
 *
 ****************************************************************************************************************************/

/*****************************************************************************************************************************
 * Other Header File Inclusion
 ****************************************************************************************************************************/
#include "stm32g0xx_hal.h"
#include "tim.h"
#include "adcSampling.h"
#include "MainControl.h"

__IO uint32_t mOSTM16_SysTick20us_A = 0;
__IO uint32_t mOSTM16_SysTick20us_K = 0;
__IO uint32_t mOSTM16_SysTick10ms_K = 0;
__IO uint32_t mOSTM16_SysTick1ms_S = 0;
__IO uint32_t mOSTM16_SysTick20us_CMD510B_M[MOTOR_BDC_NUMBER_MAX] = {0};
__IO uint8_t mDebugFlagPowerDownCMD510B[MOTOR_BDC_NUMBER_MAX][5];
__IO uint32_t mAddFgVal = 0;

#define THRESHOLD_LEN                   30
#define THRESHOLD_LEN_DEC_1          (THRESHOLD_LEN - 1)

#if (MOTOR_MODEL == CHENXIN_5840_3650)              // 400350DW 使用辰鑫的电机，一个周期约2.5ms  , 第一个周期约12ms ，第二个周期约9ms
// const uint32_t FG_THRESHOLD[THRESHOLD_LEN] = {25000,25000,7500,6000,5000,4000,2500,2500,2500,2500
//                                 ,250,250,250,250,250,250,250,250,250,250};      //400阈值对应电流： 230 ~ 342 （438mA ~ 651mA）
// const uint32_t FG_THRESHOLD[THRESHOLD_LEN] = {1200,900,750,600,500,400,250,250,250,250
//                                 ,250,250,250,250,250,250,250,250,250,250};      //400阈值对应电流： 230 ~ 342 （438mA ~ 651mA）
const uint32_t FG_THRESHOLD[THRESHOLD_LEN] = {1200,1200,1200,1200,1200,1200,1200,1200,1200,1200         //-30°C辰鑫电机可以一次性开启（电流值和常温差不多），-35°C只能走1个FG（1200参数）电流并没有超过-30°C的最大值，是超过1200时间，断的电如果改大1200，电流会激增，导致堵转时齿轮断裂。说明油脂在低于-30°C时发生了剧烈的变化。
                                ,1200,1200,1200,1200,1200,1200,1200,1200,1200,1200
								,1200,1200,1200,1200,1200,1200,1200,1200,1200,400};                 //许昌现场的状态： 600 关紧，700开启（配1.5mm百叶）。现在是： 600关紧，800开启（配1.0mm百叶）
                                                                                                    //20us：400 关紧参数，-40°C一次性开启。20000+次常温老化正常。
#endif

//行程20mm 速度9mm/s  FG信号不用上拉电阻，5V输出
// 迪洛克防爆电机 一个周期约 0.95ms ~ 1.0 ms   //第1个周期13ms ,第2个周期12ms ,第3个周期11ms ,第4个周期9ms ,第5个周期5ms ,
#if (MOTOR_MODEL == DLK_YLSZ23)   
const uint32_t FG_THRESHOLD[THRESHOLD_LEN] = {1000,800,800,700,500,400,350,250,150,125         // 400350DW 使用万融的电机，一个周期约1.75ms  , 第一个周期约5.4ms ，第二个周期约4.8ms
                                 	,125,125,125,125,125,125,125,125,125,125
									,125,125,125,125,125,125,125,125,125,125};           //220
#endif

void dwMotorCtrl(uint8_t motorCh)
{
#if (MOTOR_MODEL == CHENXIN_5840_3650 || MOTOR_MODEL == DLK_YLSZ23)
    if(mMachineSta.activation == RUN_STA) {
        mAddFgVal = 0;   //50    //(采用PWM方式供电) 数组 ： 350 ， mAddFgVal = 150 ，电机运行约 300 次，出现1台进风电机打齿。
        
//>>>>>>>>>>>>>>>>>>>  中途提高驱动力的处理  >>>>>>>>>>>>>>>>>>>
        if(mKeySta.nowKeySta == OPEN_DOOR) {
            if(mCount.motorCMD510BRunSta[motorCh] < (FARTHEST_POSITION_DC_B_MOTOR - 100))
                mAddFgVal = 100;    //庆城是400关紧  600动作，所以 mAddFgVal = 200;   增强齿轮是500关紧 800动作，所以 mAddFgVal = 300;
            else
                mAddFgVal = 0;
        }
        else {
            if(mCount.motorCMD510BRunSta[motorCh] < 500 && mDoorSta.nowDoorPositionCMD510BM[motorCh] > 100)
                mAddFgVal = 100;    //庆城是400关紧  600动作，所以 mAddFgVal = 200;   增强齿轮是500关紧 800动作，所以 mAddFgVal = 300;
            else 
                mAddFgVal = 0;
        }
//<<<<<<<<<<<<<<<<<<<  中途提高驱动力的处理  <<<<<<<<<<<<<<<<<<<

        if((mCount.motorCMD510BRunSta[motorCh] >= THRESHOLD_LEN_DEC_1 && mOSTM16_SysTick20us_CMD510B_M[motorCh] >= (FG_THRESHOLD[THRESHOLD_LEN_DEC_1] + mAddFgVal)) 
        || (mOSTM16_SysTick20us_CMD510B_M[motorCh] >= (FG_THRESHOLD[mCount.motorCMD510BRunSta[motorCh]] + mAddFgVal) && (mCount.motorCMD510BRunSta[motorCh] < THRESHOLD_LEN_DEC_1 && ((mCount.motorCMD510BRunSta[motorCh] > 0)))) 
        || (mCount.motorCMD510BRunSta[motorCh] == 0 && mOSTM16_SysTick20us_CMD510B_M[motorCh] > 1300)) {	//10ms  If the pulse is not received for a long time, it means that the motor is stuck.
            if(mDebugFlagPowerDownCMD510B[motorCh][0] == 0) {
#if (MOTOR_MODEL == CHENXIN_5840_3650)
                setBDCMotorStop(motorCh);	//堵转的时候，可能也有脉冲。要测试  //   ZBL  20250418
#endif
                mDebugFlagPowerDownCMD510B[motorCh][0] = 1;
                // printf("PowerDownShadesMotorAAA() mCount.motorCMD510BRunSta[0] = %d mOSTM16_SysTick20us_CMD510B_M[0] = %d\r\n",mCount.motorCMD510BRunSta[0],mOSTM16_SysTick20us_CMD510B_M[0]);
            }
            mOSTM16_SysTick20us_CMD510B_M[motorCh] = 0;
            if(mKeySta.nowKeySta == CLOSE_DOOR) {		// 堵转停止说明是到了关门极限位置，如果是门被卡住，则有漏洞，但不加此逻辑结构件会出现百叶越开越小的问题
                mDoorSta.nowDoorPositionCMD510BM[motorCh] = 0;
            }
        }
    }
#endif
}

void dwMotorCallback(void)
{
    mOSTM16_SysTick20us_A++;
    mOSTM16_SysTick20us_K++;
    mOSTM16_SysTick1ms_S++;
    mOSTM16_SysTick20us_CMD510B_M[0]++;
    mOSTM16_SysTick20us_CMD510B_M[1]++;
    mOSTM16_SysTick20us_CMD510B_M[2]++;
    dwMotorCtrl(0);
    dwMotorCtrl(1);
    dwMotorCtrl(2);
}
/*****************************************************************************************************************************
 * END OF FILE: dwMotor.c
 ****************************************************************************************************************************/
