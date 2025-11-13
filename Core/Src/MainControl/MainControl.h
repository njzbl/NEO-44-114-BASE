/*****************************************************************************************************************************
 * \file   : MainControl.h    
 * \module : MainControl
 * \brief  : 
 * \project: NEO-180160-SM
 * \author : ZBL
 *============================================================================================================================
 *                                                   Revision control History
 *============================================================================================================================
 * V1.0.0: 2023-09-19: Initial Version
 *
 *
 *
 ****************************************************************************************************************************/
#ifndef MAINCONTROL_H
#define	MAINCONTROL_H

/*****************************************************************************************************************************
 * Other Header File Inclusion 
 ****************************************************************************************************************************/
#include "stm32g0xx_hal.h"
#include "gpio.h"
#include "usart.h"
#include "string.h"
#include "stdarg.h"
#include "stdio.h"
#include "main.h"
#include "demo.h"


/*****************************************************************************************************************************
 * Compile Option or configuration Section (for/debug)                                                
 ****************************************************************************************************************************/

/*****************************************************************************************************************************
 * Macro Definition
 ****************************************************************************************************************************/


#define INVALID                         0
#define VALID                           1

#define PARAM_DISENABLED                0
#define PARAM_ENABLED                   1

#define TRUE                            1
#define FALSE                           0

#define RUN                             1
#define STOP                            0

#define DELAY_50MS                      5
#define DELAY_100MS                     10
#define DELAY_150MS                     15
#define DELAY_300MS                     30
#define DELAY_600MS                     60
#define DELAY_1S                        100
#define DELAY_3S                        300
#define DELAY_4S                        400
#define DELAY_5S                        500
#define DELAY_6S                        600
#define DELAY_7S                        700
#define DELAY_14S                       1400
#define DELAY_15S                       1500
#define DELAY_16S                       1600
#define DELAY_30S                       3000


#define MOTOR_RUN_PLUSE_INDEX_MAX               2         //2个周期的脉冲值。参与阈值计算。
#define MOTOR_RUN_PLUSE_10MS_MAX                1000      //10ms内最多收到的脉冲数
#define MOTOR_RUN_PLUSE_10MS_MIN_1              6         //10ms 内最少收到的脉冲数1
#define MOTOR_RUN_PLUSE_10MS_THRESHOLD_1        14         //10ms 内收到的脉冲数阈值1
#define MOTOR_RUN_PLUSE_10MS_MIN_2              5         //10ms 内最少收到的脉冲数2
#define MOTOR_RUN_PLUSE_10MS_THRESHOLD_2        10         //10ms 内收到的脉冲数阈值2
#define MOTOR_RUN_PLUSE_10MS_MIN_3              3         //10ms 内最少收到的脉冲数3
#define MOTOR_RUN_PLUSE_10MS_THRESHOLD_3        8         //10ms 内收到的脉冲数阈值3

#define MOTOR_BDC_NUMBER_MAX                    3         //一块主板最多带4个直流无刷电机
#define MOTOR_BDC_CURRENT_BUF_MAX               3        //直流无刷电机电流采集buf的最大深度，即最大周期数。暂定50us一个周期
#define MOTOR_BDC_CURRENT_MAX_FARWORD			      2000000		//最大的堵转电流，实际值测量。开门2.0A截止
#define MOTOR_BDC_CURRENT_MAX_BACK				      -1650000		//最大的堵转电流，实际值测量。关门2.8~3.0A截止，关门时需要大力矩锁紧，合理

#define MAINCONTROL_20US_TO_10MS                500

#define KEY_VAL_OPEN_WIN                            0
#define KEY_VAL_CLOSE_WIN                           1
#define KEY_VAL_INVALID                             0xff
/*****************************************************************************************************************************
 * Enumeration Definition
 ****************************************************************************************************************************/

/*****************************************************************************************************************************
 * Typedef Definition
 ****************************************************************************************************************************/
typedef struct COUNT
{
  uint32_t delay_20us;
  uint32_t delay10ms;
  uint32_t key;
  uint32_t agingCheck;
  uint32_t chk;
  uint32_t motor;
  uint32_t motorRunSta;   //记录百叶在一个检测周期内合理电流值的总数量
  uint32_t motorRunStaA;   //记录直流无刷电机的百叶在一个检测周期内合理电流值的总数量
  uint32_t motorRunStaB;   //记录直流无刷电机的百叶在一个检测周期内合理电流值的总数量
  uint32_t motorRunPluseA[MOTOR_RUN_PLUSE_INDEX_MAX];   //记录直流无刷电机的百叶在一个定时周期内合理电流值的总数量, 或FG 信号的次数，用于检测堵转
  uint32_t motorRunPluseB[MOTOR_RUN_PLUSE_INDEX_MAX];   //记录直流无刷电机的百叶在一个定时周期内合理电流值的总数量, 或FG 信号的次数，用于检测堵转
  uint32_t motorRunPluseIndex;                          //直流无刷电机的 Pluse 的下标
  uint32_t motorBDCRunSta[MOTOR_BDC_NUMBER_MAX];        //直流有刷电机的百叶在一定周期内合理电流的总数
  uint32_t fan;
  uint32_t fanRunSta;     //记录风机在一个检测周期内合理电流值的总数量
  uint32_t initBackDoor;  //上电初始化先回home position
  uint32_t loopTestCount;     //老化循环测试时间记录
  uint32_t keyDownQG;               //气感信号的按键去抖，单独计时
  uint32_t keyUpQG;               //气感信号的按键去抖，单独计时
  uint32_t power;              //用于180160DW风扇全速运行时，百叶反转时，FG信号异常，等待电机停止后再启动的软件处理。
  uint32_t motorCMD510BRunStaA;   //记录CMD510B直流无刷电机的百叶在一个检测周期内合理电流值的总数量
  uint32_t motorCMD510BRunStaB;   //记录CMD510B直流无刷电机的百叶在一个检测周期内合理电流值的总数量
}stCOUNT;

typedef struct DOOR_STA
{
  uint16_t nowDoorPosition;
  uint16_t nextDoorPosition;
  uint8_t toggleDirSta;   //正向反向运动切换完成的标志 //toggleDirection
  uint8_t toggleDirPreSta;   //正向反向运动切换预处理的标志
  uint8_t runSta;               //open or close
  uint8_t speedSta;             //ACC DEC HOLD ,上电默认是HOLD，然后根据按键来运动
  uint8_t slowStep;             //ACC DEC 的步数
  uint8_t phase;                //当前的相位
  uint8_t motorCh;               //当前电机的通道号
  uint32_t moreStep;             //400350为了压紧胶条，碰到接近开关后多走的step
  uint32_t msStopDelay;          //电机STOP状态已经保持的时间。1 = 10ms
  int32_t nowDoorPositionDCM[MOTOR_BDC_NUMBER_MAX];  //for DC brushless motor  A/B/C
  // int32_t nowDoorPositionDCMA;  //for DC brushless motor  A
  // int32_t nowDoorPositionDCMB;  //for DC brushless motor  B 
  // int32_t nowDoorPositionDCMC;  //for DC brushless motor  C
  int32_t motorFG[MOTOR_BDC_NUMBER_MAX];    //隔爆电机，采用直流无刷，输出FG信号，用于角度计算。本变量用于计算，一次开/闭动作会发出的FG信号总数。
  int32_t countMidwayOpenDoor;  //
  int32_t lastPositionCLoseDoor[MOTOR_BDC_NUMBER_MAX];  //这个变量其实可以用nowDoorPositionDCMA/C 来实现的，但为了避免某一次电机异常而导致的永远不能找到关门的0位置。这样每次关百叶就可以清零了。
  //1、完整的开百叶行程是5935个脉冲 ，完整的关百叶行程是5945个脉冲，所以关比开百叶多10个脉冲
  //2、百叶未到极限位置时，中途开百叶次数，每次开百叶（不到堵转动的极限位置），会增加10个脉冲。
  //3、每次开百叶中间有一次关未到位，再开到堵转的极限位置会多加50个脉冲，每次关百叶中间有一次开未到位，再关到极限位置也会多减50个脉冲
  int32_t motorCurNum[MOTOR_BDC_NUMBER_MAX];       //记录直流有刷电机运动过程中电流超过阈值的次数，对于没有FG信号的电机可以通过此方式确定电机旋转角度的近似值。
  int32_t nowDoorPositionCMD510BMA;  //CMD510B for DC brushless motor  A 
  int32_t nowDoorPositionCMD510BMB;  //CMD510B for DC brushless motor  B 
  int32_t doorPositionFinalA;
  int32_t doorPositionFinalB;

}stDOOR_STA;

typedef struct MOTOR_BDC_CUR    //此结构图只针对采用INA219芯片的模式，不适用于ADC采样模式的BDC电机。这样ADC采样代码结构更清晰。移植更方便。
{
  signed long current[4][MOTOR_BDC_CURRENT_BUF_MAX];       //4个直流有刷电机（BDC）的电流采集值
  uint8_t BDCIndex;                                                           //4个直流有刷电机（BDC）的电流采集序号（下标）
  uint8_t currentIndex;
  uint8_t sampleFlag[4];   //0:本通道在自己的采样周期内还没采样  1：本通道在自己的采样周期内已经采样完成。
  uint8_t motorBDCValid[4];         //直流有刷电机的百叶有效值状态
  uint8_t Doneflag;   //0: 大循环采样周期为结束  1:本次大循环采样周期 MOTOR_BDC_NUMBER_MAX X MOTOR_BDC_CURRENT_BUF_MAX 已经结束，
}stMOTOR_BDC;

typedef struct KEY_STA
{
  uint8_t isNewKeyCmd;
  uint8_t nowKeySta;
  uint8_t nextKeySta;
  uint8_t nextChkSta;
}stKEY_STA;

typedef struct MACHINE_STA
{
  uint8_t activation;         //0： run    1： stop
  uint8_t faultSta;           //1：error   0:  OK
  uint8_t hBridgeSta;         //0: 将上下管都置于关闭状态   1:等待10ms保证上下管都完全关闭   2： 已打开一侧的下管   3： 异测的上下管都打开
}stMACHINE_STA;

typedef struct FAN_STA
{
  uint32_t fanCur;          // 风扇电流
  uint32_t fanFg;           // 风扇FG
}stFAN_STA;

typedef struct ATH20DATA{
    int32_t RH;
    int32_t temperature;
    uint8_t status;
}stATH20DATA;

/*****************************************************************************************************************************
 * Table Definition
 ****************************************************************************************************************************/

/*****************************************************************************************************************************
 * Global Function Prototypes
 ****************************************************************************************************************************/
extern stINPUT_STA mInputSta;
extern __IO stCOUNT mCount;
extern __IO stDOOR_STA mDoorSta;
extern __IO stKEY_STA mKeySta;
extern stMACHINE_STA mMachineSta;
extern stMOTOR_BDC mMotorBDC;
extern __IO uint32_t mOSTM16_SysTick20us_K;
extern __IO uint32_t mOSTM16_SysTick10ms_K;
extern __IO uint32_t mOSTM16_SysTick20us_CMD510B_M_A;
extern __IO uint32_t mOSTM16_SysTick20us_CMD510B_M_B;
extern int mDoorRunNumSta;
extern stMACHINE_MDBUS_STA mMachineModbusSta;
// extern uint16_t mMotorCurTemp[1024];
extern uint16_t mMotorCurCount;
extern uint16_t mMotorCurCountFlag;

extern void PrintfVersion(void);
extern int MainControl(void);
extern void InitVar(void);

void OpenBDCShares(uint8_t chn);
void CloseBDCShares(uint8_t chn);
void StopBDCShares(uint8_t chn);
void OpenExShades(void);
void CloseExShades(void);
void StopExShades(void);
void getBDCMotorCur(void);
void setBLDCMotor(uint8_t chn, uint8_t sta);
extern void StartFan(void);
extern void StopFan(void);
// extern HAL_StatusTypeDef  AHT20GetStatusTempRH(stATH20DATA *ath20data);
// extern HAL_StatusTypeDef  AHT20TransCmd(void);
// extern unsigned char Calc_CRC8(unsigned char *message,unsigned char Num);
#endif
/*****************************************************************************************************************************
 * END OF FILE: StepMotor.h
 ****************************************************************************************************************************/
