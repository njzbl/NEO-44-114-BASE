/*****************************************************************************************************************************
 * \file   : adcSampling.c    
 * \module : adcSampling
 * \brief  : adcSampling module  
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
#include "adcSampling.h"
/*****************************************************************************************************************************
 * Macro Definition
 ****************************************************************************************************************************/

/*****************************************************************************************************************************
 * Enumeration Definition
 ****************************************************************************************************************************/

/*****************************************************************************************************************************
 * Typedef Definition
 
 ****************************************************************************************************************************/

#define MOTOR_PUSH_CURRENT              875     //875 = 1342mA ;  700 = 1074mA ;
#define MOTOR_PUSH_CURRENT_AVG          600     //600 = 1142mA ;
/*****************************************************************************************************************************
 * Static Local Variables Declaration
 ****************************************************************************************************************************/

/*****************************************************************************************************************************
 * Table Constant Definition
 ****************************************************************************************************************************/

// const uint32_t NTC_10K_B_3950[181] = {  //环控风机外采的10K_B_3950
//                                     1034600 ,959006 ,889452 ,825419 ,766434 ,712066 ,661926 ,615656 ,572934 ,533466 ,
//                                     496983 ,463240 ,432015 ,403104 ,376320 ,351495 ,328472 ,307110 ,287279 ,268859 ,
//                                     251741 ,235826 ,221021 ,207242 ,194412 ,182460 ,171320 ,160932 ,151241 ,142196 ,
//                                     133750 ,125859 ,118485 ,111589 ,105139 ,99102 ,93450 ,88156 ,83195 ,78544 ,
//                                     74183 ,70091 ,66250 ,62643 ,59255 ,56071 ,53078 ,50263 ,47614 ,45121 ,
//                                     42774 ,40563 ,38480 ,36517 ,34665 ,32919 ,31270 ,29715 ,28246 ,26858 ,
//                                     25547 ,24307 ,23135 ,22026 ,20977 ,19987 ,19044 ,18154 ,17310 ,16510 ,
//                                     15752 ,15034 ,14352 ,13705 ,13090 ,12507 ,11953 ,11427 ,10927 ,10452 ,
//                                     10000 ,9570 ,9161 ,8771 ,8401 ,8048 ,7712 ,7391 ,7086 ,6795 ,
//                                     6518 ,6254 ,6001 ,5761 ,5531 ,5311 ,5102 ,4902 ,4710 ,4528 ,
//                                     4353 ,4186 ,4026 ,3874 ,3728 ,3588 ,3454 ,3326 ,3203 ,3085 ,
//                                     2973 ,2865 ,2761 ,2662 ,2567 ,2476 ,2388 ,2304 ,2223 ,2146 ,
//                                     2072 ,2000 ,1932 ,1866 ,1803 ,1742 ,1684 ,1627 ,1573 ,1521 ,
//                                     1471 ,1423 ,1377 ,1332 ,1289 ,1248 ,1208 ,1170 ,1133 ,1097 ,
//                                     1063 ,1030 ,998 ,968 ,938 ,909 ,882 ,855 ,829 ,805 ,
//                                     781 ,758 ,735 ,714 ,693 ,673 ,653 ,635 ,616 ,599 ,
//                                     582 ,565 ,550 ,534 ,519 ,505 ,491 ,478 ,465 ,452 ,
//                                     440 ,428 ,416 ,405 ,395 ,384 ,374 ,364 ,355 ,345 ,
//                                     337 };

const uint32_t NTC_10K_B_3950[181] = {  //CMFB103F3950FANT   0805_10K_3950  风华
        583542 ,554647 ,526968 ,500480 ,475159 ,450974 ,427897 ,405892 ,384927 ,364967 ,
        345975 ,327915 ,310751 ,294448 ,278969 ,264279 ,250344 ,237130 ,224603 ,212733 ,
        201487 ,190836 ,180750 ,171201 ,162163 ,153610 ,145516 ,137858 ,130614 ,123761 ,
        117280 ,111149 ,105351 ,99867  ,94681  ,89776  ,85137  ,80750  ,76600  ,72676  ,
        68963  ,65451  ,62129  ,58986  ,56012  ,53198  ,50534  ,48013  ,45627  ,43368  ,
        41229  ,39204  ,37285  ,35468  ,33747  ,32116  ,30570  ,29105  ,27716  ,26399  ,
        25150  ,23965  ,22842  ,21776  ,20764  ,19783  ,18892  ,18026  ,17204  ,16423  ,
        15681  ,14976  ,14306  ,13669  ,13063  ,12487  ,11939  ,11418  ,10921  ,10449  ,
        10000  ,9571   ,9164   ,8775   ,8405   ,8052   ,7716   ,7396   ,7090   ,6798   ,
        6520   ,6255   ,6002   ,5760   ,5529   ,5309   ,5098   ,4897   ,4704   ,4521   ,
        4345   ,4177   ,4016   ,3863   ,3716   ,3588   ,3440   ,3311   ,3188   ,3069   ,
        2956   ,2848   ,2744   ,2644   ,2548   ,2457   ,2369   ,2284   ,2204   ,2126   ,
        2051   ,1980   ,1911   ,1845   ,1782   ,1721   ,1663   ,1606   ,1552   ,1500   ,
        1450   ,1402   ,1356   ,1312   ,1269   ,1228   ,1188   ,1150   ,1113   ,1078   ,
        1044   ,1011   ,979    ,948    ,919    ,891    ,863    ,837    ,811    ,787    ,
        763    ,740    ,718    ,697    ,676    ,657    ,637    ,619    ,601    ,584    ,
        567    ,551    ,535    ,520    ,505    ,491    ,477    ,464    ,451    ,439    ,
        427    ,415    ,404    ,393    ,383    ,373    ,363    ,353    ,344    ,335    ,
        326};

// const uint32_t NTC_10K_B_3950[181] = {  //   MF58-103F 3950       0805_10K_3950  南京时恒
//         1399250,1172570, 999260, 864178, 757035, 670669, 600002, 541371, 492090, 450165, 
//         414092, 382723, 355176, 330764, 308947, 289300, 271480, 255217, 240290, 226520, 
//         213761, 201893, 190818, 180452, 170728, 161587, 152979, 144862, 137201, 129963, 
//         123120, 116647, 110522, 104725,  99237,  94042,  89123,  84467,  80059,  75886,  
//         71937,  68199,  64662,  61316,  58150,  55155,  52321,  49641,  47107,  44709,  
//         42442,  40298,  38270,  36352,  34537,  32371,  31198,  29662,  28208,  26833,  
//         25531,  24299,  23133,  22028,  20982,  19893,  19052,  18162,  17318,  16518,  
//         15760,  15040,  14357,  13710,  13095,  12511,  11956,  11429,  10928,  10452,  
//         10000,  9569,   9160,   8770,   8399,   8046,   7710,   7389,   7084,   6793,   
//         6516,   6251,   5999,   5758,   5529,   5309,   5100,   4900,   4709,   4526,   
//         4352,   4185,   4026,   3873,   3727,   3588,   3454,   3326,   3203,   3086,   
//         2973,   2866,   2762,   2663,   2568,   2477,   2390,   2306,   2226,   2148,   
//         2074,   2003,   1935,   1869,   1806,   1745,   1687,   1631,   1577,   1525,   
//         1475,   1427,   1381,   1337,   1294,   1253,   1213,   1175,   1138,   1103,   
//         1065,   1035,   1004,   973,    944,    915,    888,    861,    835,    811,    
//         787,    764,    742,    720,    699,    677,    660,    641,    623,    583,    
//         567,    551,    535,    520,    506,    492,    479,    466,    453,    441,    
//         429,    417,    406,    396,    385,    375,    365,    356,    347,    338,    
//         329};
/*****************************************************************************************************************************
 * Static Local Functions Declaration
 ****************************************************************************************************************************/

extern __IO   uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];
/*****************************************************************************************************************************
 *                                                                  Function Source code
 ****************************************************************************************************************************/
extern __IO uint32_t mOSTM16_SysTick20us_A;
extern uint8_t mMotorPowerEnabled;
__IO stADC_SAMPLING mFanAdc = {0};
__IO stADC_SAMPLING mMotorAdc[3] = {0};
__IO stADC_SAMPLING mPowerAdcB = {0};
__IO stADC_SAMPLING mNtc10KAdc = {0};
__IO stADC_SAMPLING mTemplateAdc = {0};
__IO stADC_SAMPLING mVrefValAdc = {0};
__IO uint32_t mPtMotorCurrentMax[30] = {0};
__IO uint8_t mPtMotorCurrentCount[3] = {0};
__IO uint8_t mPtMotorPushCurrentCount[3] = {0};
#if(MACHINE_DEBUG == DEBUG_ENABLED)
__IO uint16_t mMotorCurMaxTotal[2][40] = {0};               //测试代码需要的变量，正式程序中要屏蔽，减少堆的占用
__IO uint16_t mMotorCurMaxNow[2][40] = {0};                 //测试代码需要的变量，正式程序中要屏蔽，减少堆的占用
uint16_t mMotorCurNow[1000] = {0};
#endif
uint32_t mMotorMaxCurrentTime[3] = {0};             		//大电流时FG信号最大持续时间

int32_t mFanAdcAvg[10] = {0};
uint8_t mFanAdcAvgNum = 0;
void ClsArray(uint8_t* buf, uint32_t size)
{
    for(uint32_t i = 0;i < size;i++) {
        buf[i] = 0;
    }
}

void InitSampling(void)
{
    for(uint32_t i = 0; i < ADC_DIVISION_VAL; i++) {
        ClsArray((uint8_t*)& mFanAdc, sizeof(mFanAdc));
        ClsArray((uint8_t*)& mMotorAdc, sizeof(mMotorAdc));
        ClsArray((uint8_t*)& mPowerAdcB, sizeof(mPowerAdcB));
        ClsArray((uint8_t*)& mTemplateAdc, sizeof(mTemplateAdc));
        ClsArray((uint8_t*)& mVrefValAdc, sizeof(mVrefValAdc));
        ClsArray((uint8_t*)& mNtc10KAdc, sizeof(mNtc10KAdc));
    }
}
/*****************************************************************************************************************************
 * adcSampling
 ****************************************************************************************************************************/
//  mAdcV[i] = aADCxConvertedData[i] * 3300 / 4096;
//             aADCxConvertedData[i] = VAR_CONVERTED_DATA_INIT_VALUE;//����������0
//FAN（9.5W风机）: 风机没启动时 adcAvg = 2840 ，adcSum基本都是小于adcAvg，所以 adcSum < adcAvg * 2 恒成立。风机启动时： adcSum > adcAvg * 4 ,基本是恒成立。 所以定在adcSum < adcAvg * 2为分割点
//                改版后 V1.0版本硬件，风机没启动时 adcAvg = 3062（A样机)、3107(B样机) adcSum = 8534(A样机，adcSum绝大多数是小于8500，也有极个别达到17281，90%小于10000。adcAvg很稳定，基本相差在个位数) B样机也差不多(B样机再次上电发现，为启动风机时，adcSum小了很多，有一半的时候是小于adcAvg，但启动风机后95%+是大于20000的，此时adcAvg = 3113)，启动风机后 adcSum = 18853 ~ 33491，绝大多数是大于20000的。
//                改版后 V1.0版本硬件，风机启动:adcAvg = 3062（A样机),adcSum = 8534(A样机，adcSum 90%+ 大于 22130)
//FAN（5.2W风机）: 风机没启动时 adcAvg = 2840 ，adcSum基本都是小于adcAvg，所以 adcSum < adcAvg * 3 / 2 恒成立。风机启动时： adcSum > adcAvg * 2.14 ,基本是恒成立。 所以定在adcSum < adcAvg * 3 / 2 为分割点

//MOTOR:(国产科辰电机) ：百叶没启动时0%电流最大500，（30%电流时最大1850）； 100% 电流时正常都在2000以上，最大2582 ， 所以设置 2000 作为分割。
//MOTOR:(国产科辰电机,新款，表面处理)  百叶没启动时30% 电流，mMotorACurrentVal 90~150， 100% 时，mMotorACurrentVal 0~5 ，所以阈值定在 30。,拔掉电机线，mMotorACurrentVal 0~5 。所以以30%电流时为判定依据
//FAN（9.5W风机）(180160-NH 主板): 风机没启动时 ：adcSum < 80000(基本在40000 ~70000) adcAvg = 2840 ，所以 adcSum < adcAvg * 100 恒成立。风机启动时： adcSum > 800 0000(基本在900 0000 ~1000 0000) adcSum > adcAvg * 100 ,基本是恒成立。 所以定在adcSum < adcAvg * 100为分割点

void adcSampling(void)
{
    uint32_t adcSum = 0,adcAvg = 0,tempflag[3] = {0};
    uint32_t adcPower[3] = {0};
    uint32_t curVal[ADC_DIVISION_VAL] = {0};
    uint32_t maxCurThreshold[3] = {8000},motorRunAngle[3] = {0};
    uint32_t motorMaxCurHoldTimeMsThreshold = 50; //ms
    
            
    if(mFanAdc.SamplingFalg == 1) {
        adcSum = 0;
        for(int i = 0;i <ADC_DIVISION_VAL;i++) {
            curVal[i] = mFanAdc.DivVal[i];
            adcSum += mFanAdc.DivVal[i];
        }
        mFanAdc.DivNum = 0; //新的一轮采集开始
        mFanAdc.SamplingFalg = 0;   //清除采集完成标识
        adcAvg = adcSum / ADC_DIVISION_VAL;
        adcSum = 0;
        for(int i = 0;i <ADC_DIVISION_VAL;i++) {
            if(curVal[i] < adcAvg) {
                adcSum += (adcAvg - curVal[i]);//adcSum += ((adcAvg - curVal[i]) * (adcAvg - curVal[i]));
            }
            else {
                adcSum += (curVal[i] - adcAvg);//adcSum += ((curVal[i] - adcAvg) * (curVal[i] - adcAvg));
            }
        }
    mFanAdc.CurrentVal = adcSum;
#if (FAN_MODEL == FAN_MODEL_AC_75W)
    //这个值是NEO自己的风机，福佑的风机值还要再测试。用于华为项目，华为和阳光电源mFanAdc.ThresholdMin = 10000;
    mFanAdc.ThresholdMin = 3500;   //风扇不转时350,  风扇转时10000+   //mFanAdc.ThresholdMin = 1000000;          //实际电流约30000~40000
    mFanAdc.ThresholdMax = 575000000;  //mFanAdc.ThresholdMax = 575000000;        //实际电流约30000000~75000000
#endif
        
        // if(adcSum > mFanAdc.ThresholdMin) {
            // printf("fancur1: %d,%d,%d,%d,%d, %d,%d,%d,%d,%d\r\n",curVal[0],curVal[1],curVal[2],curVal[3],curVal[4],curVal[5],curVal[6],curVal[7],curVal[8],curVal[9]);
            // printf("fancur2: %d,%d,%d,%d,%d, %d,%d,%d,%d,%d\r\n",curVal[10],curVal[11],curVal[12],curVal[13],curVal[14],curVal[15],curVal[16],curVal[17],curVal[18],curVal[19]);
            // printf("fancur3: %d,%d,%d,%d,%d, %d,%d,%d,%d,%d\r\n",curVal[20],curVal[21],curVal[22],curVal[23],curVal[24],curVal[25],curVal[26],curVal[27],curVal[28],curVal[29]);
            // printf("fancur4: %d,%d,%d,%d,%d, %d,%d,%d,%d,%d\r\n",curVal[30],curVal[31],curVal[32],curVal[33],curVal[34],curVal[35],curVal[36],curVal[37],curVal[38],curVal[39]);
            // printf("fancur5: %d,%d,%d,%d,%d, %d,%d,%d,%d,%d\r\n",curVal[40],curVal[41],curVal[42],curVal[43],curVal[44],curVal[45],curVal[46],curVal[47],curVal[48],curVal[49]);
            // printf("adcAvg,adcSum:%d,%d\r\n",adcAvg,adcSum);
            // if(adcSum > mFanAdc.MaxTempRun) {
            //     mFanAdc.MaxTempRun = adcSum;
            // }
            // if(adcSum < mFanAdc.MinTempRun) {
            //     mFanAdc.MinTempRun = adcSum;
            // }
        // }
        // else {            if(adcSum > mFanAdc.MaxTempStop) {
        //         mFanAdc.MaxTempStop = adcSum;
        //     }
        //     if(adcSum < mFanAdc.MinTempStop)
        //         mFanAdc.MinTempStop = adcSum;
        // }
        // printf("adcAvg,adcSum,MinB,MaxB,S,S:%d,%d  %d,%d,%d,%d\r\n",adcAvg,adcSum,mFanAdc.MinTempRun,mFanAdc.MaxTempRun,mFanAdc.MinTempStop,mFanAdc.MaxTempStop);
        // printf("adcAvg,adcSum:%d,%d\r\n",adcAvg,adcSum);
        int32_t nowVal = 0,nowValTemp = 0;
        nowValTemp = ((mFanAdc.CurrentVal * 9) / 1000);        //CurrentVal:采集数据有效值 单位: mA
        if(mFanAdcAvgNum > 9) {
            mFanAdcAvgNum = 0;
        }
        mFanAdcAvg[mFanAdcAvgNum] = nowValTemp;
        mFanAdcAvgNum++;
        adcSum = 0;
        for(int i = 0; i < 10;i++) {
            adcSum += mFanAdcAvg[i];
        }
        nowVal = adcSum / 10;
        if(nowVal > 40)
            mMachineModbusSta.fanCurrent = nowVal - 20;  //有20mA 的基准偏差
        else
            mMachineModbusSta.fanCurrent = 0;
        //  printf("fan current:%d mA %d\r\n",nowVal,mFanAdc.CurrentVal);
        mFanAdc.CurFlag = 1;                //CurFlag：采集数据有效值标识
    }
//     if(mTemplateAdc.SamplingFalg == 1) {
//         int32_t VrefData = 0;
//         int32_t Tempruate = 0;
//         int32_t tempSum = 0;
//         for(int i = 0;i <ADC_DIVISION_VAL;i++) {            
//             VrefData = 3300;//
// 		    Tempruate=__HAL_ADC_CALC_TEMPERATURE(VrefData,mTemplateAdc.DivVal[i],ADC_RESOLUTION_12B);//
//             tempSum += Tempruate;
//         }
//         mTemplateAdc.DivNum = 0; //新的一轮采集开始
//         mTemplateAdc.SamplingFalg = 0;   //清除采集完成标识
//         mTemplateAdc.CurrentVal = tempSum / ADC_DIVISION_VAL;
//         mMachineModbusSta.temperature = mTemplateAdc.CurrentVal + 273;
//         mTemplateAdc.ThresholdMin = 2000;
// #if (MOTOR_MODEL == CHENXIN_5840_3650)
//         mTemplateAdc.ThresholdMin = 30;   //mMotorAdcAvgThreshold = 2000;
// #endif
//         mTemplateAdc.CurFlag = 1;                //mMotorCurFalg：采集数据有效值标识
//     }

    if(mMotorAdc[0].SamplingFalg == 1) {
        for(int j = 0; j < 3;j++) {
            tempflag[j] = 0;
            adcPower[j] = 0;
            for(int i = 0;i <ADC_DIVISION_VAL;i++) {
                curVal[i] = mMotorAdc[j].DivVal[i] * 3300 >> 12;
                adcPower[j] += curVal[i];
#if(MOTOR_MODEL == DLK_TG_60W)
                // if(mMotorAdc[j].DivVal[i] > mPtMotorCurrentMax[j]) {
                //     mPtMotorCurrentMax[j] = mMotorAdc[j].DivVal[i];
                //     tempflag[j] = 1;
                // }
#endif
#if(MOTOR_MODEL == CHENXIN_5840_3650)
                if(mMotorAdc[j].DivVal[i] > mPtMotorCurrentMax[j] && (mCount.motorCMD510BRunStaA > 50 || mCount.motorCMD510BRunStaB > 50 || mCount.motorCMD510BRunStaC > 50)) {
                    mPtMotorCurrentMax[j] = mMotorAdc[j].DivVal[i];
                    tempflag[j] = 1;
                }
#endif
            }
            // if((mKeySta.nowKeySta == CLOSE_DOOR && adcPower[j] > 5000) || (mKeySta.nowKeySta == OPEN_DOOR && adcPower[j] > 10000)) {
            //     if(j == 0) {// if(mKeySta.nowKeySta == CLOSE_DOOR && j == 0) {
            //         mMotorCurTemp[mMotorCurCount++] = (adcPower[j] / ADC_DIVISION_VAL);
            //         //L1-0 0 0 0 0_0 0 0 0 0__0 0 0 0 861_826 779 752 0 0   电机空载电流是50 ~ 70，疑似861_826 779 752  电流导致齿轮崩了，计算得约1.5A
            //         //L9-121 126 160 201 190_250 342 9 8 9__8 8 9 8 9_9 9 9 9 9                 阈值：400时的电流，342 = 651mA
            //         //L21-358 429 415 372 398_412 443 312 428 473__504 552 570 8 8_9 9 9 8 9    阈值：500时的电流，570 = 1085mA
            //         //L3-408 423 501 466 509_477 493 297 454 452__450 521 672 705 0_0 0 0 0 0   阈值：600时的电流，705 = 1342mA
            //         if(mMotorCurCount >= 998) {
            //             mMotorCurCount = 998;
            //         }
            //     }
            // }


            // if((adcPower[j] / ADC_DIVISION_VAL) >= MOTOR_PUSH_CURRENT_AVG) {
            //     if(j == 0) {
            //         if(mCount.motorCMD510BRunStaA > 10) {
            //             HAL_GPIO_WritePin(MOTOR_PWR_CTRL1_GPIO_Port, MOTOR_PWR_CTRL1_Pin, GPIO_PIN_RESET);
            //             // printf("mCount.motorCMD510BRunStaA = %d adcPower[%d] = %d\r\n",mCount.motorCMD510BRunStaA,j, (adcPower[j] / ADC_DIVISION_VAL));
            //         }
            //     }
            //     else if(j == 1) {
            //         if(mCount.motorCMD510BRunStaB > 10) {
            //             HAL_GPIO_WritePin(MOTOR_PWR_CTRL2_GPIO_Port, MOTOR_PWR_CTRL2_Pin, GPIO_PIN_RESET);
            //             // printf("mCount.motorCMD510BRunStaB = %d adcPower[%d] = %d\r\n",mCount.motorCMD510BRunStaB,j, (adcPower[j] / ADC_DIVISION_VAL));
            //         }
            //     }
            // }


            mMotorAdc[j].ThresholdMin = 120; //150mA(30W 电机空载的时候基本都大于150mA ， 启动电流会达到1680mA ) ,电机停止时电流 为12mA左右。所以阈值定位150mA
                                             //     (60W 电机空载的时候基本都大于300mA ， 启动电流会达到1960mA )

                                             
            // mMotorAdc[j].ThresholdMin = 50; //50mA
            // ((adcPower[j] << 2) / 105)  这个公式正确 ，实测800mA电流误差小于 12% ,  信号毛刺通常在20mA,去毛刺阈值应设定为40   2025-07-16
            mMotorAdc[j].CurrentVal = ((adcPower[j] << 2) / 105); //单位:mA  (adcPower[j] / ADC_DIVISION_VAL) / (21 * 0.025)  = ((adcPower[j] << 2) / 105)   //mMotorAdc[j].CurrentVal = ((adcPower[j] / ADC_DIVISION_VAL) * 800 / 523); //单位:mA
            // mMotorAdc[j].CurrentVal = (adcPower[j] / ADC_DIVISION_VAL) * 1000 / (21 * 25);
            // if(mMotorAdc[j].CurrentVal > mPtMotorCurrentMax[j])
            //     mPtMotorCurrentMax[j] = mMotorAdc[j].CurrentVal;
            // if(mMotorAdc[j].CurrentVal < 40) {  //去毛刺，实际电机启动时电流值一定大于40mA
            //     mMotorAdc[j].CurrentVal = 0;
            // }

            // mMotorAdc[j].CurrentVal = ((adcPower[j] * 20) / 61) / 5;   // mMotorAdc[j].CurrentVal 单位:mA
            // if(j == 0 && mPtMotorCurrentMax[j] > 2500 && tempflag[j] == 1) {
            //     printf("mMotorAdc[%d] = %d,%d,%d,%d,%d,  %d,%d,%d,%d,%d  \r\n",j, curVal[0], curVal[1], curVal[2], curVal[3], curVal[4], curVal[5], curVal[6], curVal[7], curVal[8], curVal[9]);
            //     printf("mMotorAdc[%d] = %d,%d,%d,%d,%d,  %d,%d,%d,%d,%d  \r\n",j, curVal[10], curVal[11], curVal[12], curVal[13], curVal[14], curVal[15], curVal[16], curVal[17], curVal[18], curVal[19]);
            //     printf("mMotorAdc[%d] = %d,%d,%d,%d,%d,  %d,%d,%d,%d,%d  \r\n",j, curVal[20], curVal[21], curVal[22], curVal[23], curVal[24], curVal[25], curVal[26], curVal[27], curVal[28], curVal[29]);
            //     printf("mMotorAdc[%d] = %d,%d,%d,%d,%d,  %d,%d,%d,%d,%d  \r\n",j, curVal[30], curVal[31], curVal[32], curVal[33], curVal[34], curVal[35], curVal[36], curVal[37], curVal[38], curVal[39]);
            //     printf("mMotorAdc[%d] = %d,%d,%d,%d,%d,  %d,%d,%d,%d,%d  \r\n",j, curVal[40], curVal[41], curVal[42], curVal[43], curVal[44], curVal[45], curVal[46], curVal[47], curVal[48], curVal[49]);
            // }

            
#if(MOTOR_MODEL == DLK_TG_60W || MOTOR_MODEL == G_ROCH_D3Ex)
                // if(mMotorAdc[j].CurrentVal > mPtMotorCurrentMax[j]) {
                //     mPtMotorCurrentMax[j] = mMotorAdc[j].CurrentVal;
                //     tempflag[j] = 1;
                // }
#endif
#if(MOTOR_MODEL == G_ROCH_D3Ex)
            maxCurThreshold[j] = 3500;                      //空载电流 200 ~ 300mA   堵转电流 2.0 ~ 3.0 A
            motorRunAngle[j] = mDoorSta.motorCurNum[j];
            motorMaxCurHoldTimeMsThreshold = 500;
#endif

#if(MOTOR_MODEL == DLK_TG_60W)          //25°C 电机推出时完全堵转，mMotorAdc[j].CurrentVal 瞬间最大值3275，然后逐渐降低最后一直稳定在2800左右，此时稳压电源读数为：2.6 ~ 2.8A ，在低温-30°C时小裴测试稳压电源读数2.9A。 -40°C时 串口监测到mMotorAdc[j].CurrentVal 长时间达到：4000+
            if(mNtc10KAdc.CurrentVal > -10) {
                maxCurThreshold[j] = 2000; //maxCurThreshold[j] = 2800;  //迪洛克常规款电机堵转时电流为2400 ~ 2850 左右，考虑个体差异性，取偏小值 2.4A ，持续500ms
            }
            else {
                maxCurThreshold[j] = 7500;
            }
            motorRunAngle[j] = mDoorSta.motorCurNum[j];
            motorMaxCurHoldTimeMsThreshold = 500;
#endif
#if(MOTOR_MODEL == DLK_YLSZ23 || MOTOR_MODEL == DLK_YLSZ23_FB)
            maxCurThreshold[j] = 2000;                      //待机电流0.04mA  空载 500 ~ 600mA   
            motorRunAngle[j] = mDoorSta.motorCurNum[j];
            motorMaxCurHoldTimeMsThreshold = 50;
#endif
#if(MOTOR_MODEL == WG_TG)
            maxCurThreshold[j] = 2000;
            motorRunAngle[j] = mDoorSta.motorCurNum[j];
            motorMaxCurHoldTimeMsThreshold = 50;
#endif
#if(MOTOR_MODEL == CHENXIN_5840_3650)           //辰鑫电机运行时电流通常50+ ， 不通电20+ ，
            uint8_t countTemp = 0;
            if(mNtc10KAdc.CurrentVal >= -20) {
                motorMaxCurHoldTimeMsThreshold = 20;
                maxCurThreshold[j] = 1600;                  //常温时，1600阈值跑50000W次电机齿轮没有坏。
            }
            else {
                motorMaxCurHoldTimeMsThreshold = 20;
                maxCurThreshold[j] = 1900; //maxCurThreshold = 1600;       //1600 常温可能可以，但是低温可能不够(常温和低温阈值参数可以分开)，会导致-30°C提前停止。实测-30°C整机实验，第一次开启百叶正常，再关闭百叶在45°时提前停止。可能是这里的问题。而且多台主板测试发现电流采样值差异挺大的，基本无法精确保护，只能放大这里的值，用于宽范围保护。
            }
            if(j == 0) {
                motorRunAngle[0] = mCount.motorCMD510BRunStaA;
// #if(MACHINE_DEBUG == DEBUG_ENABLED)
//                 if(motorRunAngle[0] < 990) {
//                     if(mMotorCurNow[motorRunAngle[0]] < mMotorAdc[j].CurrentVal)
//                         mMotorCurNow[motorRunAngle[0]] = mMotorAdc[j].CurrentVal;
//                 }
//                 if((motorRunAngle[0] >= 800) && (motorRunAngle[0] < 840)) {
//                     if(mMotorCurMaxNow[0][motorRunAngle[0] - 800] < mMotorAdc[j].CurrentVal)
//                         mMotorCurMaxNow[0][motorRunAngle[0] - 800] = mMotorAdc[j].CurrentVal;
//                     if(mMotorCurMaxTotal[0][motorRunAngle[0] - 800] < mMotorAdc[j].CurrentVal)
//                         mMotorCurMaxTotal[0][motorRunAngle[0] - 800] = mMotorAdc[j].CurrentVal;
//                 }
//                 countTemp = motorRunAngle[0] / 30;
//                 if(countTemp < 40) {
//                     if(mMotorCurMaxNow[j][countTemp] < mMotorAdc[j].CurrentVal)
//                         mMotorCurMaxNow[j][countTemp] = mMotorAdc[j].CurrentVal;
//                     if(mMotorCurMaxTotal[j][countTemp] < mMotorAdc[j].CurrentVal)
//                         mMotorCurMaxTotal[j][countTemp] = mMotorAdc[j].CurrentVal;
//                 }
// #endif
            }
            else if(j == 1) {
                motorRunAngle[1] = mCount.motorCMD510BRunStaB;
#if(MACHINE_DEBUG == DEBUG_ENABLED)
                // if(motorRunAngle[1] < 30) {
                //     if(mMotorCurMaxNow[0][motorRunAngle[1]] < mMotorAdc[j].CurrentVal)
                //         mMotorCurMaxNow[0][motorRunAngle[1]] = mMotorAdc[j].CurrentVal;
                //     if(mMotorCurMaxTotal[0][motorRunAngle[1]] < mMotorAdc[j].CurrentVal)
                //         mMotorCurMaxTotal[0][motorRunAngle[1]] = mMotorAdc[j].CurrentVal;
                // }
                // else {
                //     countTemp = mCount.motorCMD510BRunStaB / 30;
                //     if(mMotorCurMaxNow[j][countTemp] < mMotorAdc[j].CurrentVal)
                //         mMotorCurMaxNow[j][countTemp] = mMotorAdc[j].CurrentVal;
                //     if(mMotorCurMaxTotal[j][countTemp] < mMotorAdc[j].CurrentVal)
                //         mMotorCurMaxTotal[j][countTemp] = mMotorAdc[j].CurrentVal;
                // }
                if(motorRunAngle[1] < 990) {
                    if(mMotorCurNow[motorRunAngle[1]] < mMotorAdc[j].CurrentVal)
                        mMotorCurNow[motorRunAngle[1]] = mMotorAdc[j].CurrentVal;
                }
                if((motorRunAngle[1] >= 800) && (motorRunAngle[1] < 840)) {
                    if(mMotorCurMaxNow[0][motorRunAngle[1] - 800] < mMotorAdc[j].CurrentVal)
                        mMotorCurMaxNow[0][motorRunAngle[1] - 800] = mMotorAdc[j].CurrentVal;
                    if(mMotorCurMaxTotal[0][motorRunAngle[1] - 800] < mMotorAdc[j].CurrentVal)
                        mMotorCurMaxTotal[0][motorRunAngle[1] - 800] = mMotorAdc[j].CurrentVal;
                }
                countTemp = mCount.motorCMD510BRunStaB / 30;
                if(countTemp < 40) {
                    if(mMotorCurMaxNow[j][countTemp] < mMotorAdc[j].CurrentVal)
                        mMotorCurMaxNow[j][countTemp] = mMotorAdc[j].CurrentVal;
                    if(mMotorCurMaxTotal[j][countTemp] < mMotorAdc[j].CurrentVal)
                        mMotorCurMaxTotal[j][countTemp] = mMotorAdc[j].CurrentVal;
                }
#endif

            }
//             if(j == 0) {
//                 motorRunAngle[0] = mCount.motorCMD510BRunStaA;
// #if(MACHINE_DEBUG == DEBUG_ENABLED)
//                 countTemp = mCount.motorCMD510BRunStaA / 30;
//                 if(mMotorCurMaxNow[j][countTemp] < mMotorAdc[j].CurrentVal)
//                     mMotorCurMaxNow[j][countTemp] = mMotorAdc[j].CurrentVal;
//                 if(mMotorCurMaxTotal[j][countTemp] < mMotorAdc[j].CurrentVal)
//                     mMotorCurMaxTotal[j][countTemp] = mMotorAdc[j].CurrentVal;
// #endif
//             }
//             else if(j == 1) {
//                 motorRunAngle[1] = mCount.motorCMD510BRunStaB;
// #if(MACHINE_DEBUG == DEBUG_ENABLED)
//                 countTemp = mCount.motorCMD510BRunStaB / 30;
//                 if(mMotorCurMaxNow[j][countTemp] < mMotorAdc[j].CurrentVal)
//                     mMotorCurMaxNow[j][countTemp] = mMotorAdc[j].CurrentVal;
//                 if(mMotorCurMaxTotal[j][countTemp] < mMotorAdc[j].CurrentVal)
//                     mMotorCurMaxTotal[j][countTemp] = mMotorAdc[j].CurrentVal;
// #endif

//             }
#endif
            if(mMotorAdc[j].CurrentVal >= maxCurThreshold[j]) {
                mMotorMaxCurrentTime[j]++;		//大电流时FG信号最大持续时间
                if(j == 0) {
                    if(motorRunAngle[j] > 10 && mMotorMaxCurrentTime[0] > motorMaxCurHoldTimeMsThreshold) {  //压紧胶条时间长度大于500ms
		                mMachineSta.motorPowerSta[MOTOR1_LOGIC_CHN] = MOTOR_POWER_DOWN_STA;
                        HAL_GPIO_WritePin(MOTOR_PWR_CTRL1_GPIO_Port, MOTOR_PWR_CTRL1_Pin, GPIO_PIN_RESET);
                        // printf("=========mCount.motorCMD510BRunStaA = %d adcPower[%d] = %d motorRunAngle[%d] = %d\r\n",mCount.motorCMD510BRunStaA,j, mMotorAdc[j].CurrentVal,j,motorRunAngle[j]);
                        printf("++++++++++++++++++++++++mCount.motorCMD510BRunStaA = %d adcPower[%d] = %d motorRunAngle[%d] = %d\r\n",mCount.motorCMD510BRunStaA,j, mMotorAdc[j].CurrentVal,j,motorRunAngle[j]);
                    }
                }
                else if(j == 1) {
                    if(motorRunAngle[j] > 10 && mMotorMaxCurrentTime[1] > motorMaxCurHoldTimeMsThreshold) {  //压紧胶条时间长度大于500ms
		                mMachineSta.motorPowerSta[MOTOR2_LOGIC_CHN] = MOTOR_POWER_DOWN_STA;
                        HAL_GPIO_WritePin(MOTOR_PWR_CTRL2_GPIO_Port, MOTOR_PWR_CTRL2_Pin, GPIO_PIN_RESET);
                        printf("+++++++++++++++++++++++mCount.motorCMD510BRunStaB = %d adcPower[%d] = %d motorRunAngle[%d] = %d\r\n",mCount.motorCMD510BRunStaB,j, mMotorAdc[j].CurrentVal,j,motorRunAngle[j]);
                    }
                }
            }
        }
        // printf("---------mMotorAdc.CurrentVal = %d,%d,%d %d--------------\r\n", mMotorAdc[0].CurrentVal, mMotorAdc[1].CurrentVal, mMotorAdc[2].CurrentVal,mNtc10KAdc.CurrentVal);
        // printf("---------mMotorAdc.CurrentVal = %d,%d,%d --------------\r\n", adcPower[0] / ADC_DIVISION_VAL, adcPower[1] / ADC_DIVISION_VAL, adcPower[2] / ADC_DIVISION_VAL);
        mMotorAdc[0].DivNum = 0; //新的一轮采集开始
        mMotorAdc[0].SamplingFalg = 0;   //清除采集完成标识
        mMotorAdc[0].CurFlag = 1;
    }
#if (MACHINE_POWER_LOSS_DETECT == PROTECTION_ENABLED)
    if(mPowerAdcB.SamplingFalg == 1) {
        int32_t tempSum = 0;
        for(int i = 0;i <ADC_DIVISION_VAL;i++) {
            curVal[i] = mPowerAdcB.DivVal[i] * 3300 >> 12;
            tempSum += curVal[i];
        }
        mPowerAdcB.ThresholdMin = 1100;     //1800mV 对应 R1 = 20K ; 1100mV 对应 R1 = 47K (DC24V 正常供电时 1220mV ； DC20V 供电时 1060mV ； DC18V 供电时 980mV )
        mPowerAdcB.CurrentVal = (tempSum / ADC_DIVISION_VAL);
        // mMachineModbusSta.Ntctemperature = mNtc10KAdc.CurrentVal + 273;
        // printf("mPowerAdcB.CurrentVal = %d\r\n", mPowerAdcB.CurrentVal);
        mPowerAdcB.DivNum = 0; //新的一轮采集开始
        mPowerAdcB.SamplingFalg = 0;   //清除采集完成标识
        mPowerAdcB.CurFlag = 1;
    }
#endif
    if(mNtc10KAdc.SamplingFalg == 1) {
        int32_t tempSum = 0;
        int32_t tempval = 0;
        for(int i = 0;i <ADC_DIVISION_VAL;i++) {
            curVal[i] = mNtc10KAdc.DivVal[i] * 3300 >> 12;
            tempSum += curVal[i];
        }
        mNtc10KAdc.ThresholdMin = 150;
        adcAvg = (tempSum / ADC_DIVISION_VAL);
        tempval = (10000 * adcAvg) / (3300 - adcAvg);   //tempval = (4700 * (3300 - adcAvg)) / adcAvg; 
        int i;
        for(i = 0; i < sizeof(NTC_10K_B_3950) / sizeof(uint32_t); i++) {
            if(tempval >= NTC_10K_B_3950[i]) {
                break;
            }
        }
        mNtc10KAdc.CurrentVal = i - 55;
        mMachineModbusSta.Ntctemperature = mNtc10KAdc.CurrentVal + 273;
        mNtc10KAdc.DivNum = 0; //新的一轮采集开始
        // mNtc10KAdc.SamplingFalg = 0;   //清除采集完成标识
        mNtc10KAdc.CurFlag = 1;
    }
}



void adcCallback(void)
{
    if(mOSTM16_SysTick20us_A >= ADC_SAMPLING_PERIOD) {
        mOSTM16_SysTick20us_A = 0;
        mFanAdc.DivVal[mFanAdc.DivNum++] = aADCxConvertedData[ADC_FAN_CURRENT_CH];
        if(mFanAdc.DivNum == ADC_DIVISION_VAL) {
            mFanAdc.DivNum = 0;
            mFanAdc.SamplingFalg = 1;
        }
    }

    // mTemplateAdc.DivVal[mTemplateAdc.DivNum] = aADCxConvertedData[ADC_MCU_TEMP_CH];
    // mTemplateAdc.DivNum++;
    // if(mTemplateAdc.DivNum == ADC_DIVISION_VAL) {
    //     mTemplateAdc.DivNum = 0;
    //     mTemplateAdc.SamplingFalg = 1;
    // }

    mMotorAdc[0].DivVal[mMotorAdc[0].DivNum] = aADCxConvertedData[ADC_M1_CURRENT_CH];
    mMotorAdc[1].DivVal[mMotorAdc[0].DivNum] = aADCxConvertedData[ADC_M2_CURRENT_CH];
    mMotorAdc[0].DivNum++;
    if(mMotorAdc[0].DivNum == ADC_DIVISION_VAL) {       //这里只判断[0]是为了减少中断处理时间，20us一个中断。 
        mMotorAdc[0].DivNum = 0;
        mMotorAdc[0].SamplingFalg = 1;
    }
    
    if(mMotorAdc[0].DivVal[mMotorAdc[0].DivNum] >= MAX_CURRENT_MOTOR) {  //5A  ，实测效果很好，MOS管不会坏 ， 2025-07-16
        mPtMotorCurrentCount[0]++;
        if(mPtMotorCurrentCount[0] > 30) {              //参数为5时，短路时峰值电流为16A，持续时间约100us, 取值10的原因：辰鑫400350DW电机，启动瞬间会有4个周期电流值超过2625，
            mMachineSta.motorPowerSta[MOTOR1_LOGIC_CHN] = MOTOR_POWER_DOWN_STA;
            HAL_GPIO_WritePin(MOTOR_PWR_CTRL1_GPIO_Port, MOTOR_PWR_CTRL1_Pin, GPIO_PIN_RESET);
            mPtMotorCurrentMax[4]++;
        }

        if(mMotorAdc[0].DivVal[mMotorAdc[0].DivNum] > mPtMotorCurrentMax[1]) {
            mPtMotorCurrentMax[0] = mMotorAdc[0].DivVal[mMotorAdc[0].DivNum];

        }
    }
    else {
        mPtMotorCurrentCount[0] = 0;
        // if(mCount.motorCMD510BRunStaA > 5) {
        //     if (mMotorAdc[0].DivVal[mMotorAdc[0].DivNum] > MOTOR_PUSH_CURRENT) { //875 = (705 << 12) / 3300 ; 阈值：600时的电流，705 = 1342mA ; 700 = 1074mA ;
        //         mPtMotorPushCurrentCount[0]++;
        //         if(mPtMotorPushCurrentCount[0] > 10) {
        //             HAL_GPIO_WritePin(MOTOR_PWR_CTRL1_GPIO_Port, MOTOR_PWR_CTRL1_Pin, GPIO_PIN_RESET);
        //         }
        //     }
        //     else
        //         mPtMotorPushCurrentCount[0] = 0;
        //     }
        // }        
    }
    if(mMotorAdc[1].DivVal[mMotorAdc[0].DivNum] >= MAX_CURRENT_MOTOR) {
        mPtMotorCurrentCount[1]++;
        if(mPtMotorCurrentCount[1] > 30) {
		    mMachineSta.motorPowerSta[MOTOR2_LOGIC_CHN] = MOTOR_POWER_DOWN_STA;
            HAL_GPIO_WritePin(MOTOR_PWR_CTRL2_GPIO_Port, MOTOR_PWR_CTRL2_Pin, GPIO_PIN_RESET);
            mPtMotorCurrentMax[5]++;
        }
        
        if(mMotorAdc[1].DivVal[mMotorAdc[0].DivNum] > mPtMotorCurrentMax[1]) {
            mPtMotorCurrentMax[1] = mMotorAdc[1].DivVal[mMotorAdc[0].DivNum];

        }
    }
    else {
        mPtMotorCurrentCount[1] = 0;
        // if(mCount.motorCMD510BRunStaB > 10) {
        //     if (mMotorAdc[1].DivVal[mMotorAdc[0].DivNum] > MOTOR_PUSH_CURRENT) { //875 = (705 << 12) / 3300 ; 阈值：600时的电流，705 = 1342mA ; 700 = 1074mA ;
        //         mPtMotorPushCurrentCount[1]++;
        //         if(mPtMotorPushCurrentCount[1] > 10) {
        //             HAL_GPIO_WritePin(MOTOR_PWR_CTRL2_GPIO_Port, MOTOR_PWR_CTRL2_Pin, GPIO_PIN_RESET);
        //         }
        //     }
        //     else {
        //         mPtMotorPushCurrentCount[1] = 0;
        //     }
        // }
    }
    mPowerAdcB.DivVal[mNtc10KAdc.DivNum] = aADCxConvertedData[ADC_POWER_VALID_CH];
    mNtc10KAdc.DivVal[mNtc10KAdc.DivNum] = aADCxConvertedData[ADC_BOARD_TEMP_CH];
    mNtc10KAdc.DivNum++;
    if(mNtc10KAdc.DivNum == ADC_DIVISION_VAL) {
        mNtc10KAdc.DivNum = 0;
        mNtc10KAdc.SamplingFalg = 1;
        mPowerAdcB.SamplingFalg = 1;
    }
}
/*****************************************************************************************************************************
 * END OF FILE: StepMotor.c
 ****************************************************************************************************************************/
