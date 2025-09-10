/*****************************************************************************************************************************
 * \file   : MainControl.c    
 * \module : MainControl
 * \brief  : MainControl module  
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

/*****************************************************************************************************************************
 * Other Header File Inclusion
 ****************************************************************************************************************************/
#include "stm32g0xx_hal.h"
#include "tim.h"
#include "MainControl.h"
#include "adcSampling.h"
#include "common.h"
#include "stm32_flash.h"
#include "i2c.h"

stINPUT_STA mInputSta = {GPIO_PIN_RESET};
stOUTPUT_STA mOutputSta = {0};
__IO stCOUNT mCount = {0};
__IO stDOOR_STA mDoorSta = {FARTHEST_POSITION, HOME_POSITION, UNCOMPLETE, UNCOMPLETE, HOLD, HOLD, 0, PHASE_0,0,HOME_POSITION,HOME_POSITION,HOME_POSITION,0,0,0};
__IO stKEY_STA mKeySta = {INVALID, UNCERTAIN, UNCERTAIN, UNCERTAIN};
stMACHINE_STA mMachineSta = {0};
stMOTOR_BDC mMotorBDC = {0};
__IO stFAN_STA mFanSta = {0};
int mDoorRunNumSta = 0; 	//0: 初始化状态， 1：开关门至少1次。//考虑复位状态下，检测 homeposition.
stMACHINE_MDBUS_STA mMachineModbusSta;
stSystemParamTypeDef mSystemParamInfo;
uint8_t mManualFlag = FALSE;
uint8_t mKeyValQG = KEY_VAL_INVALID, mKeyVal485 = KEY_VAL_INVALID, mKeyValTemp = KEY_VAL_INVALID, mAutoVal485 = KEY_VAL_INVALID, mTestVal485 = KEY_VAL_INVALID;

int32_t motorCurTemp[MOTOR_BDC_NUMBER_MAX]; 
uint8_t mBDCSta[2] = {0};           //0: 初始状态 1：允许开启状态  2：允许关闭状态
uint8_t mMotorPowerEnabled = 0;     //刚开机时禁止推杆电机供电，这样可以采用0电流时的电平。
__IO uint32_t mCMD510BPowerFlagA = 0;
__IO uint32_t mCMD510BPowerFlagB = 0;
extern __IO uint8_t mDebugFlagPowerDownCMD510BA[5];
extern __IO uint8_t mDebugFlagPowerDownCMD510BB[5];

extern __IO uint32_t mPtMotorCurrentMax[30];
extern __IO uint8_t mPtMotorCurrentCount[3];
extern __IO uint8_t mPtMotorPushCurrentCount[3];
// uint16_t mMotorCurTemp[1024] = {0};
uint16_t mMotorCurCount = 0;
uint16_t mMotorCurCountFlag = 0;
/*****************************************************************************************************************************
 * Macro Definition
 ****************************************************************************************************************************/

/*****************************************************************************************************************************
 * Enumeration Definition
 ****************************************************************************************************************************/

/*****************************************************************************************************************************
 * Typedef Definition
 ****************************************************************************************************************************/

/*****************************************************************************************************************************
 * Static Local Variables Declaration
 ****************************************************************************************************************************/

/*****************************************************************************************************************************
 * Table Constant Definition
 ****************************************************************************************************************************/


/*****************************************************************************************************************************
 * Static Local Functions Declaration
 ****************************************************************************************************************************/


/*****************************************************************************************************************************
 *                                                                  Function Source code
 ****************************************************************************************************************************/

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Falling_Callback could be implemented in the user file
   */
#if (MOTOR_MODEL == CHENXIN_5840_3650 || MOTOR_MODEL == TZC36_5840_3650)
  	if(GPIO_Pin == ON_STATE1_Pin) {
        if(mCMD510BPowerFlagA == 1) {       //增加了下拉电阻，这条限制语句就可以不用增加了，如果没有下拉电阻，那么在电机掉电后因为内部电容的存在，FG信号会缓慢的从高电平掉落到低电平，就是触发成千上万个FG脉冲信号。增加了下拉电阻可以非常有效的解决这个问题。
            return ;
        }

#if (MOTOR_MODEL == TZC36_5840_3650)
        if(mCount.motor > 200) {      //因为万融电机,前500ms内FG就没有
            mCount.motorCMD510BRunStaA++;
        }
#else
        if(mCount.motor > 1) {      //因为14米长线时,前50ms内FG不稳定
            mCount.motorCMD510BRunStaA++;
        }
#endif
		mOSTM16_SysTick10us_CMD510B_M_A = 0;	//clear Motor A pluse count, if the number over 500ms is NG.
		if(mKeySta.nowKeySta == OPEN_DOOR) {
			mDoorSta.nowDoorPositionCMD510BMA++;
			if(mDoorSta.nowDoorPositionCMD510BMA >= FARTHEST_POSITION_DC_B_MOTOR) {

                if(mDebugFlagPowerDownCMD510BA[1] == 0) {
#if (MOTOR_MODEL == CHENXIN_5840_3650)
				    setBDCMotorStop(0);      //   ZBL  20250418
                     printf("PowerDownShadesMotorAAA() mCount.motorCMD510BRunStaA = %d mOSTM16_SysTick10us_CMD510B_M_A = %d mDoorSta.nowDoorPositionCMD510BMA = %d \r\n",mCount.motorCMD510BRunStaA,mOSTM16_SysTick10us_CMD510B_M_A,mDoorSta.nowDoorPositionCMD510BMA);
                
#endif
                    mDebugFlagPowerDownCMD510BA[1] = 1;
                }
			}
		}
		else if(mKeySta.nowKeySta == CLOSE_DOOR) {	//关百叶时 通过堵转保护限制位置。
			mDoorSta.nowDoorPositionCMD510BMA--;
			if(mDoorSta.nowDoorPositionCMD510BMA <= 0) {
				mDoorSta.nowDoorPositionCMD510BMA = 0;
                // setBDCMotorStop(0);   //ZBL 20250826   低温实验用的临时增加代码
			}
		}
	}
	else if(GPIO_Pin == ON_STATE2_Pin) {
        if(mCMD510BPowerFlagB == 1) {       //增加了下拉电阻，这条限制语句就可以不用增加了
            return ;
        }
#if (MOTOR_MODEL == TZC36_5840_3650)
        if(mCount.motor > 200) {      //因为万融电机,前500ms内FG就没有
            mCount.motorCMD510BRunStaB++;
        }
#else
        if(mCount.motor > 1) {      //因为14米长线时,前50ms内FG不稳定
            mCount.motorCMD510BRunStaB++;
        }
#endif
		mOSTM16_SysTick10us_CMD510B_M_B = 0;	//clear Motor B pluse count, if the number over 500ms is NG.
		if(mKeySta.nowKeySta == OPEN_DOOR) {
			mDoorSta.nowDoorPositionCMD510BMB++;
			if(mDoorSta.nowDoorPositionCMD510BMB >= FARTHEST_POSITION_DC_B_MOTOR) {
                if(mDebugFlagPowerDownCMD510BB[1] == 0) {
                    
#if (MOTOR_MODEL == CHENXIN_5840_3650)
				    setBDCMotorStop(1);    //   ZBL  20250418
#endif
                    mDebugFlagPowerDownCMD510BB[1] = 1;
                }
			}
		}
		else if(mKeySta.nowKeySta == CLOSE_DOOR) {
			mDoorSta.nowDoorPositionCMD510BMB--;
			if(mDoorSta.nowDoorPositionCMD510BMB <= 0) {
				mDoorSta.nowDoorPositionCMD510BMB = 0;
                // setBDCMotorStop(1);   //ZBL 20250826   低温实验用的临时增加代码
			}
				
		}
	}
#endif
}

void InitVar(void)
{
	mDoorSta.nowDoorPosition = FARTHEST_POSITION;
	mDoorSta.nextDoorPosition = HOME_POSITION;
	mDoorSta.toggleDirSta = UNCOMPLETE;
	mDoorSta.toggleDirPreSta = UNCOMPLETE;
	mDoorSta.runSta = HOLD;
	mDoorSta.speedSta = HOLD;
	mDoorSta.slowStep = 0;
	mDoorSta.phase = PHASE_0;
	mDoorSta.motorCh = 0;
	mDoorSta.moreStep = 100;
	mDoorSta.countMidwayOpenDoor = 0;
	mDoorSta.msStopDelay = 0;
	for(int i = 0; i < MOTOR_BDC_NUMBER_MAX; i++) {
		mDoorSta.nowDoorPositionDCM[i] = HOME_POSITION;
		mDoorSta.motorFG[i] = 0;
		mDoorSta.lastPositionCLoseDoor[i] = 0;
        mDoorSta.motorCurNum[i] = 0;
        motorCurTemp[i] = 0;
	}
	mKeySta.isNewKeyCmd = INVALID;
	mKeySta.nowKeySta = UNCERTAIN;
	mKeySta.nextKeySta = UNCERTAIN;
	mKeySta.nextChkSta = UNCERTAIN;

    mMachineModbusSta.machineWorkMode = CHECK_SELF_MODE;            //上电时默认处于自检状态，后面初始化结束显示待机状态；
    mMachineModbusSta.outWindowsSta = WINDOWS_CLOSE_MODE;           //上电时默认百叶处于关闭状态；
    mMachineModbusSta.inWindowsSta = WINDOWS_CLOSE_MODE;            //上电时默认百叶处于关闭状态；
    mMachineModbusSta.fanSta = FAN_CLOSE_MODE;                      //上电时默认风扇处于关闭状态；
    mMachineModbusSta.fanCurrent = 0;                               //上电时默认风扇转速处于0状态；
    mMachineModbusSta.communicationSta = CONNECT_MODE;              //上电时默认通信处于连接状态；
    mMachineModbusSta.softwareVer = SOFTWARE_VERSION;               //
    mMachineModbusSta.fanRPMTotal = 0;                              //上电时默认风扇转速处于0状态；
    mMachineModbusSta.fan1RPM = 0;                                  //上电时默认风扇转速处于0状态；
    mMachineModbusSta.fan2RPM = 0;                                  //上电时默认风扇转速处于0状态；
    mMachineModbusSta.temperature = mTemplateAdc.NowVal + 273;      //开氏温度  MCU温度
    mMachineModbusSta.FanSta2 = NORMAL_MODE;                        //上电时默认百叶处于关闭状态；
}

void InitNewKeyVar(uint8_t keySta)
{
    setBDCMotorStop(0);  //   ZBL  20250418
    setBDCMotorStop(1);  //   ZBL  20250418
	mCount.key = 0;
	mKeySta.nextKeySta = keySta;
#if (MOTOR_TYPE == 1)
	mKeySta.nowKeySta = keySta;
#endif
	mKeySta.isNewKeyCmd = TRUE;
	mCount.agingCheck = 0;
	mCount.fan = 0;
	mCount.fanRunSta = 0;
    mFanSta.fanFg = 0;
	mCount.motor = 0;
	mCount.motorRunSta = 0;
	mCount.motorRunStaA = 0;
	mCount.motorRunStaB = 0;
	mCount.motorCMD510BRunStaA = 0;
	mCount.motorCMD510BRunStaB = 0;
	mCount.motorRunPluseIndex = 0;
	for(int i = 0;i < MOTOR_RUN_PLUSE_INDEX_MAX;i++) {
		mCount.motorRunPluseA[i] = 0;
		mCount.motorRunPluseB[i] = 0;
	}
	if(keySta == OPEN_DOOR) {
		printf("FG A,B,C = %d, %d, %d-  \r\n",mDoorSta.motorFG[0],mDoorSta.motorFG[1],mDoorSta.motorFG[2]);

#if (MACHINE_MODE == MACHINE_YGDY_MODE || MACHINE_MODE == MACHINE_HW_MODE || MACHINE_MODE == MACHINE_NO_MODE)
        mMachineModbusSta.machineWorkMode = RUNNINT_MODE;
#endif
#if (MACHINE_MODE == MACHINE_HY_MODE)
        mMachineModbusSta.machineWorkMode = RUNNINT_MODE - 1;
#endif
        mMachineModbusSta.fanSta = FAN_OPEN_MODE;
        mMachineModbusSta.outWindowsSta = WINDOWS_OPEN_MODE;
        mMachineModbusSta.inWindowsSta = WINDOWS_OPEN_MODE;
	}
	else if(keySta == CLOSE_DOOR) {
		printf("FG A,B,C = %d, %d, %d+  \r\n",mDoorSta.motorFG[0],mDoorSta.motorFG[1],mDoorSta.motorFG[2]);
#if (MACHINE_MODE == MACHINE_YGDY_MODE || MACHINE_MODE == MACHINE_HW_MODE || MACHINE_MODE == MACHINE_NO_MODE)
        mMachineModbusSta.machineWorkMode = WAITTING_MODE;
#endif
#if (MACHINE_MODE == MACHINE_HY_MODE)
        mMachineModbusSta.machineWorkMode = WAITTING_MODE - 1;
#endif
        mMachineModbusSta.fanSta = FAN_CLOSE_MODE;                      //上电时默认风扇处于关闭状态；
        mMachineModbusSta.outWindowsSta = WINDOWS_CLOSE_MODE;
        mMachineModbusSta.inWindowsSta = WINDOWS_CLOSE_MODE;
	}
    printf("mPtMotorCurrentMax[0:3] = %d,%d,%d _ %d,%d\r\n",mPtMotorCurrentMax[0],mPtMotorCurrentMax[1],mPtMotorCurrentMax[2],mPtMotorCurrentMax[4],mPtMotorCurrentMax[5]);
	for(int i = 0;i < MOTOR_BDC_NUMBER_MAX;i++) {
		mCount.motorBDCRunSta[i] = 0;
		mMotorBDC.sampleFlag[i] = 0;
		for(int j = 0; j < MOTOR_BDC_CURRENT_BUF_MAX; j++) {
			mMotorBDC.current[i][j] = 0;
		}
		mDoorSta.motorFG[i] = 0;
        mDoorSta.motorCurNum[i] = 0;
        motorCurTemp[i] = 0;
		if(keySta == CLOSE_DOOR) {
			if(mDoorSta.lastPositionCLoseDoor[i] > NEAREST_POSITION_DC_MOTOR_GB) {
				mDoorSta.nowDoorPositionDCM[i] += (mDoorSta.lastPositionCLoseDoor[i] - (NEAREST_POSITION_DC_MOTOR_GB >> 1));  //这里是实测出的值。
			}
		}
        mPtMotorCurrentMax[i] = 0;
        mPtMotorCurrentMax[i+3] = 0;
        mPtMotorCurrentMax[i+6] = 0;
        mPtMotorCurrentMax[i+9] = 0;
        mPtMotorCurrentCount[i] = 0;
        mPtMotorPushCurrentCount[i] = 0;
	}

	mMotorBDC.motorBDCValid[0] = DC_MOTOR0_MA;
	mMotorBDC.motorBDCValid[1] = DC_MOTOR1_MA;
	mMotorBDC.motorBDCValid[2] = DC_MOTOR2_MA;
	mMotorBDC.motorBDCValid[3] = DC_MOTOR3_MA;
	mMotorBDC.BDCIndex = 0;
	mMotorBDC.currentIndex = 0;
	mMotorBDC.Doneflag = 0;
    
    mMotorPowerEnabled = 1;
	mMachineSta.activation = RUN_STA;	//in run state
	StopExShades();
	mOutputSta.motorS1 = MACHINE_OK;
	// mOutputSta.fanS2 = MACHINE_OK;		//关风机时不要清除风机故障
    HAL_StatusTypeDef ret;
    stATH20DATA ath20data;
    ret = AHT20GetStatusTempRH(&ath20data);
    if(ret != HAL_ERROR) {
        printf("AHT20GetStatusTempRH = %d\r\n",ret);
        AHT20TransCmd();
    }
    // for(int i = 0; i < sizeof(mMotorCurTemp) / sizeof(uint16_t); i++)
    //     mMotorCurTemp[i] = 0;
    mMotorCurCount = 0;
    mMotorCurCountFlag = 0;
}

/*****************************************************************************************************************************
 * MainControl
 ****************************************************************************************************************************/
int MainControl(void)
{
	//do something
	int retn = 0, i = 0;
	// int8_t pluseSta1 = 0;
	// int8_t pluseSta2 = 0;
	uint32_t delayKey = 0;
	if(mOSTM16_SysTick10us_K >= MAINCONTROL_10US_TO_10MS) {	// 10ms
		mCount.delay10ms++;
		mOSTM16_SysTick10us_K = 0;
		mCount.fan++;
		mCount.motor++;
		if (mCount.initBackDoor < 600000) {  //600s
			mCount.initBackDoor++;
		}
		mCount.loopTestCount++;
		// mMotorBDC.Doneflag = 0;  //允许下个周期开始检测电机电流		本硬件版本没有采用INA219采集电流
	}
	else {
		//每10ms检测一次推杆电机电流，并且只在开百叶和关百叶的过程中才检测电流。这里要测试I2C总线的速度是否能跟得上
		if(mMachineSta.activation == RUN_STA) {
			getBDCMotorCur();	//软件限流的参数和策略还没改好
		}
		return retn;
	}
    UpdataModbusAddr();
	mInputSta.startQG = getChkSta();
		// printf("mOSTM16_SysTick10us_K = %d mInputSta.startQG = %d\r\n",mOSTM16_SysTick10us_K,mInputSta.startQG);
#if (LOOP_TEST == 1)
	if(mCount.loopTestCount <= 650) {	//if(mCount.loopTestCount <= 3000) {
		mInputSta.startQG = GPIO_PIN_SET;
		// printf("mInputSta.startQG = GPIO_PIN_SET\r\n");
	}
	else if(mCount.loopTestCount <= 1300) {  //else if(mCount.loopTestCount <= 6000) {
		mInputSta.startQG = GPIO_PIN_RESET;
		// printf("mInputSta.startQG = GPIO_PIN_RESET\r\n");
	}
	else
		mCount.loopTestCount = 0;
#endif
	if(mDoorRunNumSta == 0) {
		if(mCount.initBackDoor < DELAY_1S)	{	//前1秒开门，后6秒关门，为了避开关门初期接近开关不可靠接触，采用惯性关门。
			mInputSta.startQG = GPIO_PIN_RESET;
		}
		else if(mCount.initBackDoor <= DELAY_7S) {  //第一次上电，就立刻关门。如果此时已经检测到关门信号，则不执行此关门动作。只置位mDoorRunNumSta = 1
			mInputSta.startQG = GPIO_PIN_SET;
		}
	}
	// printf("mOSTM16_SysTick10us_K:%d,%d;%d\r\n",mOSTM16_SysTick10us_K,mOSTM16_SysTick10us_M,mInputSta.startQG);
	
	// if (mDoorRunNumSta == 0) {
	// 	delayKey = DELAY_150MS;
	// }
	// else {
	// 	delayKey = DELAY_1S;		//只有用机械式继电器才会要求间隔1秒钟。
	// }
	delayKey = DELAY_150MS;
	if(mMachineSta.hBridgeSta == H_BRIDGE_STATE_OFF) {	//这条语句必须要在InitNewKeyVar()函数之前，否则会造成H桥上下管同时导通的问题。
		mDoorSta.msStopDelay++;
		if(mDoorSta.msStopDelay >= 50)		//500ms
			mMachineSta.hBridgeSta = H_BRIDGE_STATE_WAIT;
	}
//>>>>>>>>>>>>>>>>  气感信号按键单独去抖计算，为了实现阳光电源变态的控制逻辑 >>>>>>>>>>>>>>>>>>>>
    if(mInputSta.startQG == GPIO_PIN_RESET) {   //闭合状态，为了开百叶
        mCount.keyUpQG = 0;
        mCount.keyDownQG++;
        if(mCount.keyDownQG > delayKey) {	 //600ms去抖
            mCount.keyDownQG = delayKey + 1;
            mKeyValQG = KEY_VAL_OPEN_WIN;
        }
    }
    else {   //断开状态，为了关百叶
        mCount.keyDownQG = 0;
        mCount.keyUpQG++;
        if(mCount.keyUpQG > delayKey) {	 //600ms去抖
            mCount.keyUpQG = delayKey + 1;
            mKeyValQG = KEY_VAL_CLOSE_WIN;
        }
    }
//<<<<<<<<<<<<<<<<  气感信号按键单独去抖计算，为了实现阳光电源变态的控制逻辑 <<<<<<<<<<<<<<<<<<<<

#if (MODBUS_CTRL == PARAM_ENABLED)
    // if(mManualFlag == FALSE) {     //气感信号优先级高于485信号。只有在气感信号手动按钮状态为关闭百叶时，485信号才有效。
    //     if(getCtrlStaModbus() == ON_MODE_PARAMS)
    //         mKeyVal485 = KEY_VAL_OPEN_WIN;
    //     else if(getCtrlStaModbus() == OFF_MODE_PARAMS)
    //         mKeyVal485 = KEY_VAL_CLOSE_WIN;
    //     else
    //         mKeyVal485 = KEY_VAL_INVALID;
    // }
    // else {
    //     mKeyVal485 = mKeyValQG;
    // }
    if(getCtrlStaModbus() == ON_MODE_PARAMS)
        mKeyVal485 = KEY_VAL_OPEN_WIN;
    else if(getCtrlStaModbus() == OFF_MODE_PARAMS)
        mKeyVal485 = KEY_VAL_CLOSE_WIN;
    else
        mKeyVal485 = KEY_VAL_INVALID;
#endif

#if (MODBUS_CTRL == 0)
    mManualFlag == TRUE;
#endif

#if (MACHINE_MODE == MACHINE_NO_MODE || MACHINE_MODE == MACHINE_YGDY_MODE || MACHINE_MODE == MACHINE_HW_MODE)

// //气感信号优先级高于485信号。只有在气感信号手动按钮状态为关闭百叶时，485信号才有效。
//     if(mKeyValQG == KEY_VAL_OPEN_WIN) { // 气感要求开百叶
//         mInputSta.start = KEY_VAL_OPEN_WIN;
//     }
//     else if(mKeyValQG == KEY_VAL_CLOSE_WIN && mManualFlag == TRUE) { //还在气感 控制周期内
//         mInputSta.start = KEY_VAL_CLOSE_WIN;
//     }
//     else if(mKeyVal485 == KEY_VAL_OPEN_WIN) { //485要求开百叶
//         mInputSta.start = KEY_VAL_OPEN_WIN;
//     }
//     else if(mKeyVal485 == KEY_VAL_CLOSE_WIN) {   //485要求关百叶
//         mInputSta.start = KEY_VAL_CLOSE_WIN;
//     }



//ZBL 20250826 低温测试临时屏蔽
    // if(mPowerAdcB.NowVal < mPowerAdcB.ThresholdMin && mCount.initBackDoor > DELAY_7S) {      //上电7秒以后  ,断电自动关闭百叶。 //ZBL 20250826 低温测试临时屏蔽
    //     mKeyValQG = KEY_VAL_CLOSE_WIN;
    // }
	if(mKeyValQG == KEY_VAL_OPEN_WIN) {//if(mInputSta.start == KEY_VAL_OPEN_WIN) {
        // if(mKeyValQG == KEY_VAL_OPEN_WIN)       //手工启动模式。
        //     mManualFlag = TRUE;
        if(mKeySta.nowKeySta != OPEN_DOOR) {
            InitNewKeyVar(OPEN_DOOR);
            printf("mKeySta.isNewKeyCmd = TRUE  OPEN_DOOR  %d,%d \r\n",mKeyValQG, mKeyVal485);
        }
    }
	else if(mKeyValQG == KEY_VAL_CLOSE_WIN) {
        if(mKeySta.nowKeySta != CLOSE_DOOR) {
            InitNewKeyVar(CLOSE_DOOR);
            printf("mKeySta.isNewKeyCmd = TRUE  CLOSE_DOOR  %d,%d \r\n",mKeyValQG, mKeyVal485);
        }
    }

#endif
#if (MACHINE_MODE == MACHINE_HY_MODE)

    mAutoVal485 = getAutoStaModbus();
    mTestVal485 = getTestStaModbus();

    if(mNtc10KAdc.NowVal >= 35) {
        mKeyValTemp = KEY_VAL_OPEN_WIN;
    }
    else if(mNtc10KAdc.NowVal <= 30) {
        mKeyValTemp = KEY_VAL_CLOSE_WIN;
    }

    if(mTestVal485 == 0x01) {       //测试模式，只接受气感信号的命令
        mInputSta.start = mKeyValQG;
    }
    else {
        if(mAutoVal485 == 0x00) {     //自动模式，温度自动控制
            if(mKeyValTemp == 0xff) {   //这里mKeyValTemp 有可能是 0xff的值
                mInputSta.start = KEY_VAL_CLOSE_WIN;
            }
            else {
                mInputSta.start = mKeyValTemp;
            }
        }
        else {      //手动状态
            mInputSta.start = mKeyVal485;
        }
    }

	if(mInputSta.start == KEY_VAL_OPEN_WIN) {
        if(mKeySta.nowKeySta != OPEN_DOOR) {
            InitNewKeyVar(OPEN_DOOR);
            printf("mKeySta.isNewKeyCmd = TRUE  OPEN_DOOR  %d,%d \r\n",mKeyValQG, mKeyVal485);
        }
    }
	else if(mInputSta.start == KEY_VAL_CLOSE_WIN) {
        if(mKeySta.nowKeySta != CLOSE_DOOR) {
            InitNewKeyVar(CLOSE_DOOR);
            printf("mKeySta.isNewKeyCmd = TRUE  CLOSE_DOOR  %d,%d \r\n",mKeyValQG, mKeyVal485);
        }
    }
#endif
	// printf(" --- mCount.loopTestCount = %d mInputSta.startQG= %d mKeySta.nextKeySta = %d\r\n",mCount.loopTestCount,mInputSta.startQG,mKeySta.nextKeySta);


	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>风机控制>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	if(mKeySta.nextKeySta == OPEN_DOOR) {	//闭合气感信号，要求开窗、开风机
		// if(mCount.initBackDoor > 400)		//继电器控制百叶，上电的时候不能频繁开关继电器，小于1S 间隔会导致继电器不能有效释放。
			StartFan();
		OpenExShades();
		// mKeySta.nextKeySta = UNCERTAIN;
	}
	else if(mKeySta.nextKeySta == CLOSE_DOOR){
		StopFan();
		CloseExShades();
		// mKeySta.nextKeySta = UNCERTAIN;	//华为版本通过mMachineSta.hBridgeSta变量控制只开一次。其实多次发出开命令也可以的。
	}
	if(mMachineSta.hBridgeSta == H_BRIDGE_STATE_OPEN_H) {	//华为版本通过mMachineSta.hBridgeSta变量控制
			mKeySta.nextKeySta = UNCERTAIN;
	}

	if(mCount.fan >= DELAY_4S) {	//风机开启或者关闭信号发出后延时4秒钟，检测电流
		if(mFanAdc.CurFlag == 1) {	//mFanCurFalg：采集数据有效
			mFanAdc.CurFlag = 0;
            //关百叶的时候也检测风机电流，如果关闭百叶时风机电流依然很大，那就说明风机故障
#if (STOP_FAN_CUR_CHECK == 0)       //
			if(mKeySta.nowKeySta == OPEN_DOOR) {	//闭合气感信号，要求开窗、开风机
#endif
				if(mFanAdc.NowVal >= mFanAdc.ThresholdMin && mFanAdc.NowVal <= mFanAdc.ThresholdMax) {
					mCount.fanRunSta++;		//记录风机正常电流值在一个检测周期内的步数
					// printf("mFanAdc.NowVal = %d mFanAdc.Threshold = %d mCount.fanRunSta = %d\r\n",mFanAdc.NowVal,mFanAdc.Threshold,mCount.fanRunSta);
				}
#if (STOP_FAN_CUR_CHECK == 0)
			}
#endif
		}
	}

	if(mCount.motor >= DELAY_6S && mCount.motor < DELAY_7S)	{//6000 ms 要出输出报告
		mMachineSta.activation = STOP_STA;
		// PowerDownInShadesMotorA();
		// PowerDownInShadesMotorB();
		StopExShades();  //临时测试把这条语句删除，实际使用过程中不可删除。  //zbl 20240522
		if(mDoorRunNumSta == 0) {	//上电后第一次开/关门命令，不理会电机状态，因为不知道上电时门状态。
			mOutputSta.motorS1 = MACHINE_OK;  //默认OK
		}
		else {
            
#if (FG_CUR_TYPE == 0)  //0:直流有刷电机用FG信号判定电机工作转态；  1:直流有刷电机用电流信号判定电机工作转态；
			uint8_t motorAllSta = 0;
			for(i = 0; i < MOTOR_BDC_NUMBER_MAX; i++) {
				if(mMotorBDC.motorBDCValid[i] == VALID) {
					if(mDoorSta.motorFG[i] < 100) {
						motorAllSta |= 1;
					}
					if(mKeySta.nowKeySta == OPEN_DOOR) {
						if((mDoorSta.nowDoorPositionDCM[i] + mDoorSta.lastPositionCLoseDoor[i]) < FARTHEST_POSITION_DC_MOTOR_GB - FAREST_POSITION_DC_MOTOR_GB)		//开百叶报警的裕量留大一点。
							motorAllSta |= 1;
					}
					else if(mKeySta.nowKeySta == CLOSE_DOOR) {
						int32_t count = 1;
						count = mDoorSta.countMidwayOpenDoor;
						if(count > 0) {
							count = count - 1;
						}
						if(mDoorSta.nowDoorPositionDCM[i] > NEAREST_POSITION_DC_MOTOR_GB + (count * 10)) {
							motorAllSta |= 1;
						}
					}
				}
			}
			// if(mCount.motorRunStaA < MOTOR_EFFICACY_NUM_MAX || mCount.motorRunStaB < MOTOR_EFFICACY_NUM_MAX) {
			// 	motorAllSta |= 1;
			// }
			if(motorAllSta != 0) {
				mOutputSta.motorS1 = MACHINE_ERR;
			}
			else {
				mOutputSta.motorS1 = MACHINE_OK;
			}
#endif
#if (FG_CUR_TYPE == 1)  //0:直流有刷电机用FG信号判定电机工作转态；  1:直流有刷电机用电流信号判定电机工作转态；

			uint8_t motorAllSta = 0;
			for(i = 0; i < MOTOR_BDC_NUMBER_MAX; i++) {
				if(mMotorBDC.motorBDCValid[i] == VALID) {
					if(mDoorSta.motorCurNum[i] < 100) {
						motorAllSta |= 1;
					}
				}
			}
			if(motorAllSta != 0) {
				mOutputSta.motorS1 = MACHINE_ERR;
			}
			else {
				mOutputSta.motorS1 = MACHINE_OK;
			}
            printf("mDoorSta.motorCurNum = %d,%d,%d motorCurTemp = %d,%d,%d\r\n",mDoorSta.motorCurNum[0],mDoorSta.motorCurNum[1],mDoorSta.motorCurNum[2],motorCurTemp[0],motorCurTemp[1],motorCurTemp[2]);
#endif
		}
		// printf("mCount.motorBDCRunSta[0-2] = %d_%d_%d\r\n",mCount.motorBDCRunSta[0],mCount.motorBDCRunSta[1],mCount.motorBDCRunSta[2]);
		// printf("setOUT_C0(%d);  = %d, A,B = %d, %d\r\n",mOutputSta.motorS1, mCount.motorRunSta,mCount.motorRunStaA, mCount.motorRunStaB);
		
		if(mKeySta.nowKeySta == CLOSE_DOOR) { //应该在关百叶的角度信号小于200时清零。
			printf("Close mDoorSta.motorFG A,B,C = %d, %d, %d ; DCMA,B = %d, %d, %d ; LAST,B = %d, %d, %d\r\n",mDoorSta.nowDoorPositionCMD510BMA,mDoorSta.nowDoorPositionCMD510BMB, mDoorSta.motorFG[2],mDoorSta.nowDoorPositionDCM[0],mDoorSta.nowDoorPositionDCM[1],mDoorSta.nowDoorPositionDCM[2],mDoorSta.lastPositionCLoseDoor[0],mDoorSta.lastPositionCLoseDoor[1],mDoorSta.lastPositionCLoseDoor[2]);
		}
		else if(mKeySta.nowKeySta == OPEN_DOOR){
			printf("Open  mDoorSta.motorFG A,B,C = %d, %d, %d ; DCMA,B = %d, %d, %d ; LAST,B = %d, %d, %d\r\n",mDoorSta.nowDoorPositionCMD510BMA,mDoorSta.nowDoorPositionCMD510BMB, mDoorSta.motorFG[2],mDoorSta.nowDoorPositionDCM[0],mDoorSta.nowDoorPositionDCM[1],mDoorSta.nowDoorPositionDCM[2],mDoorSta.lastPositionCLoseDoor[0],mDoorSta.lastPositionCLoseDoor[1],mDoorSta.lastPositionCLoseDoor[2]);
		}
		if (mKeySta.nowKeySta == CLOSE_DOOR) { //应该在关百叶的角度信号小于200时清零。
			for(i = 0; i < MOTOR_BDC_NUMBER_MAX; i++) {
				if(mDoorSta.nowDoorPositionDCM[i] >= NEAREST_POSITION_DC_MOTOR_GB)
					mDoorSta.lastPositionCLoseDoor[i] = mDoorSta.nowDoorPositionDCM[i];
				else
					mDoorSta.lastPositionCLoseDoor[i] = 0;
				mDoorSta.nowDoorPositionDCM[i] = 0;
			}
		}
		// if(mDoorRunNumSta >= 0) {
		// 	mDoorRunNumSta = 2;	//mDoorRunNumSta = 0; // 用于自我循环测试时使用的，赋值0 可以在及时大于5秒后也能一直使用虚拟按键。
		// }
		mCount.motor = DELAY_7S + 1;
		mDoorSta.countMidwayOpenDoor = 0;
        // for(int i = 0; i < 50; i++) {
        //     printf("L%d-%d %d %d %d %d_%d %d %d %d %d__%d %d %d %d %d_%d %d %d %d %d\r\n",(i + 1)
        //         ,mMotorCurTemp[0 + i * 20],mMotorCurTemp[1 + i * 20],mMotorCurTemp[2 + i * 20],mMotorCurTemp[3 + i * 20],mMotorCurTemp[4 + i * 20]
        //         ,mMotorCurTemp[5 + i * 20],mMotorCurTemp[6 + i * 20],mMotorCurTemp[7 + i * 20],mMotorCurTemp[8 + i * 20],mMotorCurTemp[9 + i * 20]
        //         ,mMotorCurTemp[10 + i * 20],mMotorCurTemp[11 + i * 20],mMotorCurTemp[12 + i * 20],mMotorCurTemp[13 + i * 20],mMotorCurTemp[14 + i * 20]
        //         ,mMotorCurTemp[15 + i * 20],mMotorCurTemp[16 + i * 20],mMotorCurTemp[17 + i * 20],mMotorCurTemp[18 + i * 20],mMotorCurTemp[19 + i * 20]
        //         );
        // }
	}
	else if(mCount.motor >= DELAY_7S) {
		mCount.motor = DELAY_7S + 1;
	}
	if(mCount.fan >= DELAY_6S) {	//风机独立开判断是为了风机在一直旋转，不停的检测风机的状态，当第一个5秒来到后，每1秒检测一次状态。
		// printf("mAutoVal485 = %d getCtrlStaModbus() = %d mTestVal485 = %d \r\n",mAutoVal485, getCtrlStaModbus(),mTestVal485);

        mCount.fanRunSta = mFanSta.fanFg;  //直流风扇，通过FG信号判断是否在旋转。
        if(mKeySta.nowKeySta == OPEN_DOOR) {	//只有开风机的时候才会更新风机状态  ， 进风百叶是没有风机的，为了使生产方便，代码都保持一致，只是进风百叶不连接S2端子。
			if(mCount.fanRunSta >= FAN_EFFICACY_NUM_MAX) {
				mOutputSta.fanS2 = MACHINE_OK;
			}
			else {
				mOutputSta.fanS2 = MACHINE_ERR;
			}
			// printf("mOutputSta.fanS2 = %d; mCount.fanRunSta = %d \r\n",mOutputSta.fanS2,mCount.fanRunSta);
		}
#if (STOP_FAN_CUR_CHECK == 1)      
        else {
            if(mCount.fanRunSta >= 35) {    //如果全速旋转，这个值应该是50。
				mOutputSta.fanS2 = MACHINE_ERR;     
                mMachineModbusSta.FanSta2 = ABNORMAL_MODE; //说明不该旋转的时候旋转了
			}
			else {
				mOutputSta.fanS2 = MACHINE_OK;
                mMachineModbusSta.FanSta2 = NORMAL_MODE;
			}
        }
#endif
		mCount.fanRunSta = 0;
        mFanSta.fanFg = 0;
		mCount.fan = DELAY_5S; //当第一个6秒来到后，每1秒检测一次风机状态。


#if (MACHINE_MODE == MACHINE_HW_MODE || MACHINE_MODE == MACHINE_YGDY_MODE || MACHINE_MODE == MACHINE_HY_MODE)  //华为和阳光电源，采用软件强拉常闭触点，更为科学。但是阳光电源实际取消的故障和状态两个干接点信号输出。
		if(mOutputSta.motorS1 == MACHINE_OK && mOutputSta.fanS2 == MACHINE_OK) {		//这个if语句放在这里降低了代码的可移植性，在移植时特别要注意
			printf("setSysErr(1);  ");      // 华为要求，正常时输出闭合状态
			setSysErr(OUT_STATUS_CLOSE);
		}
		else {
#if (MACHINE_MODE == MACHINE_HW_MODE || MACHINE_MODE == MACHINE_YGDY_MODE)
			mMachineModbusSta.machineWorkMode = FAILURE_MODE;
#endif
			printf("setSysErr(0);  ");      // 华为要求，异常时输出断开状态
			setSysErr(OUT_STATUS_OPEN);
		}
		if(mKeySta.nowKeySta == OPEN_DOOR) {
			// printf("setSysSta(1); %d_%d  FAN_%d,%d,%d\r\n",mOutputSta.motorS1,mOutputSta.fanS2,mFanAdc.NowVal,mFanAdc.ThresholdMin,mFanAdc.ThresholdMax);
			printf("setSysSta(1);\r\n");
			setSysSta(1);
		}
		else {
			printf("setSysSta(0);\r\n");
			setSysSta(0);
		}
#endif

#if (MACHINE_MODE == MACHINE_NO_MODE)  //MACHINE_QL_MODE，采用常开触点
		if(mOutputSta.motorS1 == MACHINE_OK && mOutputSta.fanS2 == MACHINE_OK) {		//这个if语句放在这里降低了代码的可移植性，在移植时特别要注意
			// printf("setSysErr(0);  ");      // 正常时输出开路状态
			setSysErr(OUT_STATUS_OPEN);
		}
		else {
			mMachineModbusSta.machineWorkMode = FAILURE_MODE;
			// printf("setSysErr(1);  ");      // 异常时输出闭合状态
			setSysErr(OUT_STATUS_CLOSE);
		}
		if(mKeySta.nowKeySta == OPEN_DOOR) {
			// printf("setSysSta(1); %d_%d  FAN_%d,%d,%d\r\n",mOutputSta.motorS1,mOutputSta.fanS2,mFanAdc.NowVal,mFanAdc.ThresholdMin,mFanAdc.ThresholdMax);
			// printf("setSysSta(1);\r\n");
			setSysSta(1);
		}
		else {
			// printf("setSysSta(0);\r\n");
			setSysSta(0);
		}
#endif

//>>>>>>>>>485 反馈状态>>>>>>>>>>>>> 
        uint32_t fg_cur[MOTOR_BDC_NUMBER_MAX] = {0};        
        if(mDoorRunNumSta == 0) {       //第一次默认都是正常的。
            for(i = 0 ;i < MOTOR_BDC_NUMBER_MAX;i++) {
                mDoorSta.motorFG[i] = 5000;
                mDoorSta.motorCurNum[i] = 5000;
            }
        }
#if (FG_CUR_TYPE == 0)  //0:直流有刷电机用FG信号判定电机工作转态；  1:直流有刷电机用电流信号判定电机工作转态；
        for(i = 0 ;i < MOTOR_BDC_NUMBER_MAX;i++) {
            fg_cur[i] = mDoorSta.motorFG[i];
        }
#endif
#if (FG_CUR_TYPE == 1)  //0:直流有刷电机用FG信号判定电机工作转态；  1:直流有刷电机用电流信号判定电机工作转态；
        for(i = 0 ;i < MOTOR_BDC_NUMBER_MAX;i++) {
            fg_cur[i] = mDoorSta.motorCurNum[i];
        }
#endif

        // printf("fg_cur = %d,%d,%d\r\n",fg_cur[0],fg_cur[1],fg_cur[2]);
        //百叶状态
        if(fg_cur[MOTOR_OUT_CHN] < 100) {  //排风
            mMachineModbusSta.outWindowsSta = WINDOWS_FAILURE_MODE;
		}
        else {
            if(mKeySta.nowKeySta == OPEN_DOOR)
                mMachineModbusSta.outWindowsSta = WINDOWS_OPEN_MODE;
            else
                mMachineModbusSta.outWindowsSta = WINDOWS_CLOSE_MODE;
        }
        if(fg_cur[MOTOR_IN_1_CHN] < 100) {  //进风
            mMachineModbusSta.inWindowsSta = WINDOWS_FAILURE_MODE;
		}
        else {            
            if(mKeySta.nowKeySta == OPEN_DOOR)
                mMachineModbusSta.inWindowsSta = WINDOWS_OPEN_MODE;
            else
                mMachineModbusSta.inWindowsSta = WINDOWS_CLOSE_MODE;
        }
        //风机状态
        if(mOutputSta.fanS2 == MACHINE_ERR) {
            mMachineModbusSta.fanSta = FAN_FAILURE_MODE;
        }
        else {
             if(mKeySta.nowKeySta == OPEN_DOOR) {
                mMachineModbusSta.fanSta = FAN_OPEN_MODE;
                mMachineModbusSta.fanRPMTotal = 2750;
                mMachineModbusSta.fan1RPM = 2750;
                mMachineModbusSta.fan2RPM = 2750;
            }
            else {
                mMachineModbusSta.fanSta = FAN_CLOSE_MODE;
                mMachineModbusSta.fanRPMTotal = 0;
                mMachineModbusSta.fan1RPM = 0;
                mMachineModbusSta.fan2RPM = 0;
            }
        }
//<<<<<<<<485 反馈状态<<<<<<<<<<<<<<
		if(mDoorRunNumSta >= 0) {
			mDoorRunNumSta = 2;	//mDoorRunNumSta = 0; // 用于自我循环测试时使用的，赋值0 可以在及时大于5秒后也能一直使用虚拟按键。
            // printf("mDoorRunNumSta = 2;\r\n");
		}
        if(mKeySta.nowKeySta == CLOSE_DOOR)     //这样写是简化判断逻辑。总之，只有在手动气感信号关闭百叶的时候，485信号才有效。
            mManualFlag = FALSE;
	}
	return retn;
}


void OpenExShades(void)
{
	if(mMachineSta.hBridgeSta == H_BRIDGE_STATE_WAIT) {
		setBDCMotorStop(MOTOR1_LOGIC_CHN);
		setBDCMotorStop(MOTOR2_LOGIC_CHN);
		mMachineSta.hBridgeSta = H_BRIDGE_STATE_OPEN_L;
	}
	else if(mMachineSta.hBridgeSta == H_BRIDGE_STATE_OPEN_L) {
#if (MOTOR_MODEL == DLK_TG_60W)
		setBDCMotorForward(MOTOR1_LOGIC_CHN);
		setBDCMotorForward(MOTOR2_LOGIC_CHN);
#endif
#if (MOTOR_MODEL == CHENXIN_5840_3650)
        setBLDCMotor(MOTOR1_LOGIC_CHN, 1);
        setBLDCMotor(MOTOR2_LOGIC_CHN, 1);
		setBDCMotorForward(MOTOR1_LOGIC_CHN);
		setBDCMotorForward(MOTOR2_LOGIC_CHN);
        mCount.motorCMD510BRunStaA = 0;
        mCount.motorCMD510BRunStaB = 0;
        mOSTM16_SysTick10us_CMD510B_M_A = 0;
        mOSTM16_SysTick10us_CMD510B_M_B = 0;
        for(int i = 0; i < 5; i++) {
            mDebugFlagPowerDownCMD510BA[i] = 0;
            mDebugFlagPowerDownCMD510BB[i] = 0;
        }
#endif
		mMachineSta.hBridgeSta = H_BRIDGE_STATE_OPEN_H;
	}
}

void CloseExShades(void)
{
	if(mMachineSta.hBridgeSta == H_BRIDGE_STATE_WAIT) {
		setBDCMotorStop(MOTOR1_LOGIC_CHN);
		setBDCMotorStop(MOTOR2_LOGIC_CHN);
		mMachineSta.hBridgeSta = H_BRIDGE_STATE_OPEN_L;
	}
	else if(mMachineSta.hBridgeSta == H_BRIDGE_STATE_OPEN_L) {
#if (MOTOR_MODEL == DLK_TG_60W)
		setBDCMotorBack(MOTOR1_LOGIC_CHN);
		setBDCMotorBack(MOTOR2_LOGIC_CHN);
#endif
#if (MOTOR_MODEL == CHENXIN_5840_3650)
        setBLDCMotor(MOTOR1_LOGIC_CHN, 0);
        setBLDCMotor(MOTOR2_LOGIC_CHN, 0);
		setBDCMotorForward(MOTOR1_LOGIC_CHN);
		setBDCMotorForward(MOTOR2_LOGIC_CHN);
        mCount.motorCMD510BRunStaA = 0;
        mCount.motorCMD510BRunStaB = 0;
        mOSTM16_SysTick10us_CMD510B_M_A = 0;
        mOSTM16_SysTick10us_CMD510B_M_B = 0;
        for(int i = 0; i < 5; i++) {
            mDebugFlagPowerDownCMD510BA[i] = 0;
            mDebugFlagPowerDownCMD510BB[i] = 0;
        }
#endif
		mMachineSta.hBridgeSta = H_BRIDGE_STATE_OPEN_H;
	}
}

void StopExShades(void)
{
	setBDCMotorStop(MOTOR1_LOGIC_CHN);
	setBDCMotorStop(MOTOR2_LOGIC_CHN);
#if (MOTOR_MODEL == CHENXIN_5840_3650)
        setBLDCMotor(MOTOR1_LOGIC_CHN, 0);
        setBLDCMotor(MOTOR2_LOGIC_CHN, 0);
#endif
	if(mMachineSta.hBridgeSta != H_BRIDGE_STATE_OFF && mMachineSta.hBridgeSta != H_BRIDGE_STATE_WAIT) {
		mDoorSta.msStopDelay = 0;
	}
	if(mMachineSta.hBridgeSta != H_BRIDGE_STATE_WAIT)
		mMachineSta.hBridgeSta = H_BRIDGE_STATE_OFF;
}
//chn : motor channel  0: exFan  1:inFan1  2:inFan2/exFan2
//sta : 0：motor reverse Rotation    1: motor forward rotate
void setBLDCMotor(uint8_t chn, uint8_t sta)
{
#if (MOTOR_MODEL == CHENXIN_5840_3650)
    setDirPwm(chn, sta);
#endif
}
//风机输入端;低电平时间4us , 45us 一个周期 ，22KHZ  ，平时低电平
//直角边 53 斜边109
//FG 16MS 一个周期，50%占空比

//108  //56+14 = 70
void StartFan(void)
{
#if (FAN_MODEL == FAN_MODEL_AC_75W)
	setFanCtrl(1);
#endif
#if (FAN_MODEL == FAN_MODEL_DC_100W)
	setFanCtrl(1);
    setFanPWM(0);
#endif
}

void StopFan(void)
{
#if (FAN_MODEL == FAN_MODEL_AC_75W)
	setFanCtrl(0);
#endif
#if (FAN_MODEL == FAN_MODEL_DC_100W)
	setFanCtrl(0);
#endif
}

void getBDCMotorCur(void)
{
	int32_t currentBDC = 0;
	if(mMotorAdc[0].CurFlag == 1)
		mMotorAdc[0].CurFlag = 0;
	else
		return;
	for(uint32_t i = 0;i < 3; i++) {
		currentBDC = mMotorAdc[i].NowVal;
		if(currentBDC > DC_CURRENT_MIN) {
			mDoorSta.motorCurNum[i]++;

            if(currentBDC > motorCurTemp[i])
                motorCurTemp[i] = currentBDC;
            // if(currentBDC >= DC_CURRENT_MAX && mCount.motor > DELAY_1S) {      // 发现严重堵转，电机电流为2A ，则立刻停止电机供电。
            //     setBDCMotorStop(i);
            // }
		}
	}
}

// void resetSampleFlag(void)
// {	uint8_t i;
// 	for(i = 0; i < MOTOR_BDC_NUMBER_MAX; i++) {
// 		mMotorBDC.sampleFlag[i] = 0;
// 	}
// }

void OpenBDCShares(uint8_t chn)
{
	// setBDCMotorForward(chn);
}

void CloseBDCShares(uint8_t chn)
{
	// setBDCMotorBack(chn);
}

void StopBDCShares(uint8_t chn)
{
	// setBDCMotorStop(chn);
}

#define DEV_ADDR_AHT20          0x38
#define REG_ADDR_TRANS          0xAC

#define AHT20_TIMEOUT_MS           1       //1ms

#define I2C_MEMADD_SIZE_8BIT            0x00000001U

//检查设备是否准备好I2C通讯， 返回HAL_OK 表示OK
HAL_StatusTypeDef	AHT20IsDeviceReady(void)
{
	uint32_t	trials = 3;		//尝试次数
	HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, DEV_ADDR_AHT20, trials, AHT20_TIMEOUT_MS);
	return	result;
}

//向任意地址写入一个字节的数据, memAddr是存储器内部地址，byteData是需要写入的dataLen个字节的数据
HAL_StatusTypeDef	AHT20WriteData(uint16_t memAddress, uint8_t *byteData, uint8_t dataLen)
{
	HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR_AHT20, memAddress,
			I2C_MEMADD_SIZE_8BIT, byteData, dataLen, AHT20_TIMEOUT_MS);
	return	result;
}

//从任意地址读出一个字节的数据, memAddr是存储器内部地址，byteData是读出的1个字节的数据，若返回HAL_OK表示读取成功
HAL_StatusTypeDef	AHT20ReadData(uint16_t memAddress, uint8_t *byteData, uint8_t dataLen)
{
	HAL_StatusTypeDef result = HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR_AHT20, memAddress,
			I2C_MEMADD_SIZE_8BIT, byteData, dataLen,AHT20_TIMEOUT_MS);
	return	result;
}

HAL_StatusTypeDef  AHT20TransCmd(void)
{   uint8_t byteData[2] = {0x33, 0x00};
    HAL_StatusTypeDef result = AHT20WriteData(REG_ADDR_TRANS, byteData, 2);
    return	result;
}

HAL_StatusTypeDef  AHT20GetStatusTempRH(stATH20DATA *ath20data)
{   
    uint8_t byteData[7] = {0xff},crc8 = 0xff;
    uint8_t status = 0;
    int32_t RH = 0, temperature = 0;
    HAL_StatusTypeDef result = AHT20ReadData(REG_ADDR_TRANS, byteData, 7);
    crc8 = Calc_CRC8(byteData, 7);
    if(crc8 != 0) {
        printf("crc8 = %d :: %d_%d_%d_%d_%d_%d_%d\r\n",crc8,byteData[0],byteData[1],byteData[2],byteData[3],byteData[4],byteData[5],byteData[6]);
        return HAL_ERROR;
    }
    status = byteData[0];
    if(((status & 0x80) == 0x80) || ((status & 0x10) == 0x00)) {
        printf("status = %d \r\n",status);
        return HAL_ERROR;
    }

    RH = byteData[1];
    RH = (RH << 8) + byteData[2];
    RH = (RH << 4) + (byteData[3] >> 4);
    temperature = (byteData[3] & 0x0f);
    temperature = (temperature << 8) + byteData[4];
    temperature = (temperature << 8) + byteData[5];
    ath20data->status = status;
    ath20data->RH = RH;
    ath20data->temperature = temperature;
    printf("status:0x%x_%d_%d :: %d_%d_%d_%d_%d_%d_%d\r\n",status,RH >> 18,(((temperature *200) / 1048576) - 50)
            ,byteData[0],byteData[1],byteData[2],byteData[3],byteData[4],byteData[5],byteData[6]);
    return	result;
}

// 将测量读取到的Status、SRH[19:0]、ST[19:0]进行CRC8检验，CRC初始值为0xFF，CRC8校验多项
// 式为：CRC[7:0]=1+x4+x5+x8
// , CRC计算代码如下：
//**********************************************************//
//CRC校验类型：CRC8
//多项式：X8+X5+X4+1
//Poly:0011 0001 0x31
unsigned char Calc_CRC8(unsigned char *message,unsigned char Num)
{
    unsigned char i;
    unsigned char byte;
    unsigned char crc =0xFF;
    for( byte = 0; byte < Num; byte++) {
        crc ^= message[byte];
        for(i = 8; i > 0; --i) {
            if(crc & 0x80)
                crc=(crc << 1) ^ 0x31;
            else
                crc=(crc << 1);
        }
    }
    return crc;
}
//**********************************************************//

/*****************************************************************************************************************************
 * END OF FILE: StepMotor.c
 ****************************************************************************************************************************/
