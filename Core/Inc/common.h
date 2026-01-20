#ifndef __COMMON_H
#define __COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define PROTECTION_DISABLED                 0
#define PROTECTION_ENABLED                  1
#define MACHINE_POWER_LOSS_DETECT           PROTECTION_DISABLED                     //系统掉电百叶自复位功能

#define DETECT_FOREIGN_DISABLED             0
#define DETECT_FOREIGN_ENABLED              1
#define MACHINE_DETECT_FOREIGN              DETECT_FOREIGN_DISABLED                 //卡异物检测功能

#define THIRD_MOTOR_DISABLED                0
#define THIRD_MOTOR_ENABLED                 1
#define MACHINE_THIRD_MOTOR                 THIRD_MOTOR_DISABLED                    //使用 DC_FAN 通道驱动第三个 DW 电机

#define DEBUG_ENABLED                       1
#define DEBUG_DISABLED                      0
#define MACHINE_DEBUG                       DEBUG_DISABLED                           //控制调试语句

//风机功率设定，只能选择一种风机
#define FAN_MODEL_AC_75W                  1
#define FAN_MODEL_DC_100W                 2

#define DLK_TG_60W                      0       //迪洛克推杆电机，内部没有堵转电流保护
#define CHENXIN_5840_3650               1       //辰鑫蜗涡轮杆电机 400350DW
#define TZC36_5840_3650                 2       //万融蜗涡轮杆电机
#define DLK_YLSZ23                      3       //迪洛克防爆款fg推杆电机，内部没有堵转电流保护
#define DLK_YLSZ23_FB                   4       //迪洛克防爆款FB推杆电机,只反馈关紧百叶（电机完全伸出）的电平信号，内部没有堵转电流保护
#define WG_TG                           5       //微光推杆电机，内部没有堵转电流保护
#define G_ROCH_D3Ex                     6       //基洛克防爆推杆电机 ，内部有堵转电流保护
#define MOTOR_MODEL                     CHENXIN_5840_3650

#if (MOTOR_MODEL == DLK_YLSZ23 || MOTOR_MODEL == DLK_YLSZ23_FB)
#define MAX_CURRENT_MOTOR               2625        //5A     这里的阈值8000为暂定值，需要测试

#endif
#if (MOTOR_MODEL == DLK_TG_60W || MOTOR_MODEL == G_ROCH_D3Ex || MOTOR_MODEL == TZC36_5840_3650 || MOTOR_MODEL == WG_TG)
#define MAX_CURRENT_MOTOR               2625        //5A     这里的阈值5000为暂定值，需要测试 //#define MAX_CURRENT_MOTOR               5000        //9A     这里的阈值5000为暂定值，需要测试

#endif
#if (MOTOR_MODEL == CHENXIN_5840_3650)
#define MAX_CURRENT_MOTOR               2625        //5A

#endif
// 项目宏定义配置如下
#define MACHINE_NO_MODE                 0   //常开触点机型
#define MACHINE_YGDY_MODE               1   //阳光电源机型
#define MACHINE_HW_MODE                 2   //华为机型
#define MACHINE_HY_MODE                 3   //广州电力公司机型


#define MACHINE_MODE                    MACHINE_NO_MODE

#define DEBUG_MODEL                     0   //0: no debug
#define LOOP_TEST                       0   //0: no loop   1: loop test
#define MOTOR_TYPE                      1   //0：step motor  1: DC brushless motor

#if (MACHINE_MODE == MACHINE_NO_MODE)
#define MODBUS_CTRL                     PARAM_ENABLED   //0: no modbus   1: enabled modbus
#define STOP_FAN_CUR_CHECK              0   //0: 关闭风机时不检测电流值  1：关闭风机时依然检测电流
#define FG_CUR_TYPE                     1   //0:直流有刷电机用FG信号判定电机工作转态；  1:直流有刷电机用电流信号判定电机工作转态；
#define FAN_MODEL                       FAN_MODEL_AC_75W

#define DC_CURRENT_MIN		150		//30W 电机：没有负载,不接电机时电流 12mA。有电机接百叶时电流基本都大于160mA ，堵转时电流1.6A~2.0A。
#define DC_CURRENT_MAX		2000    //2A 电机处于开始堵转状态。

#define SOFTWARE_VERSION                105

#endif

#if (MACHINE_MODE == MACHINE_YGDY_MODE || MACHINE_MODE == MACHINE_HY_MODE)
#define MODBUS_CTRL                     PARAM_ENABLED   //0: no modbus   1: enabled modbus
#define STOP_FAN_CUR_CHECK              1   //0: 关闭风机时不检测电流值  1：关闭风机时依然检测电流
#define FG_CUR_TYPE                     1   //0:直流有刷电机用FG信号判定电机工作转态；  1:直流有刷电机用电流信号判定电机工作转态；

#define DC_CURRENT_MIN		150		//30W 电机：没有负载,不接电机时电流 12mA。有电机接百叶时电流基本都大于160mA ，堵转时电流1.6A~2.0A。
#define DC_CURRENT_MAX		2000    //2A 电机处于开始堵转状态。

#define SOFTWARE_VERSION                104

#endif

#if (MACHINE_MODE == MACHINE_HW_MODE)
#define MODBUS_CTRL                     PARAM_DISENABLED   //0: no modbus   1: enabled modbus
#define STOP_FAN_CUR_CHECK              0   //0: 关闭风机时不检测电流值  1：关闭风机时依然检测电流
#define FG_CUR_TYPE                     0   //0:直流有刷电机用FG信号判定电机工作转态；  1:直流有刷电机用电流信号判定电机工作转态；

#define DC_CURRENT_MIN		150		//30W 电机：没有负载,不接电机时电流 12mA。有电机接百叶时电流基本都大于160mA ，堵转时电流1.6A~2.0A。
#define DC_CURRENT_MAX		5000    //5A 电机处于开始堵转状态。

#define SOFTWARE_VERSION                104

#endif



#define MACHINE_STRUCTURE               0   //0: no protect   1: enabled protect

#define HARDWARE_VERSION                102
#define MODBUS_ADDRESS_DEFAULT          1



#define OPEN_DOOR                       0
#define CLOSE_DOOR                      1
#define UNCERTAIN                       2

#define UNCOMPLETE                      0
#define COMPLETE                        1
#define COMPLETE_HALF                   2

#define HOME_POSITION                   0
#define FARTHEST_POSITION               6374   //科辰电机，2相8拍   85  // 6750    //科辰电机，2相8拍   90°   //5926 //科辰电机，2相8拍   80°    //(1733*2)  //NPM电机 65°//科辰电机 ：3240  //1470//55°     1253  //47°      1120  //42°  1066  // 40°     770   //29°    0xf460    //9600/360 = 26.667
#define REMOVE_BACKLASH                 600     //科辰电机 多走8°，消除反向间隙
#define MORE_STEPS                      0      //接近开关动作后，继续行走的步数 = 800，并且以DEC_STEP_7的速度行走，保证慢速大力矩压紧，对于风阀项目 MORE_STEPS = 0


// #define FARTHEST_POSITION_DC_B_MOTOR    4000//2900 (实际测试发现同一个百叶，一个完整行程远远大于2600，也比2900大)    //2600(max) * 1.115  	//45 ° , pluse is 2600. 2600 * 1.115 = 2900
// #define FARTHEST_POSITION_DC_B_MOTOR    3500//2900 (实际测试发现同一个百叶，一个完整行程远远大于2600，也比2900大)    //2600(max) * 1.115  	//45 ° , pluse is 2600. 2600 * 1.115 = 2900
#define FARTHEST_POSITION_DC_B_MOTOR    800 //800 辰鑫400350DW电机正式程序的参数800        //辰鑫400350DW电机 ,700 是低温实验的程序
                                                        //                                                                ;  8（进）,8（排）    ;  9, 10     ； 7, 16    ；4, 11 ;    30, 34（摇晃）;   15, 3 ;   19, 15  ;  12, -149 （摇晃）; 16, 0 ; 
                                                        //第2、3、4、8台，百叶开启到最大+70step，
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>新版电机（增加运放，没有取消120Ω电阻）+新版结构（2个螺丝固定+减小压铸件间隙）  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// 10套隔爆电机，20mm行程 ,各运行10次来回，最小值：         最大值：
// 风机序号       1     2      3      4      5      6      7      8      9      10
// 进风电机MIN  5841   5848   5783   5799   5848   5763   5811   5759   5806
// 进风电机MAX  5862   5868   5812   5820   5862   5782   5831   5779   5823
// 关闭间隙MIN   -20    -4     -6     7      -1     14     15     6      12
// 关闭间隙MAX   16     11     15     16     12     18     19     20     32

// 排风电机MIN  5552   5713   5831   6191   5901   5786   5402   5686   5832
// 排风电机MAX  5641   5732   5862   6211   5908   5816   5424   5717   5868
// 关闭间隙MIN   -21    -36    -18    11     -2     16     10     -2     -1
// 关闭间隙MAX   17     13     -8     15     10     27     18     16     27
// 7# 排风没有垫片 
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<新版电机（增加运放，没有取消120Ω电阻）+新版结构（2个螺丝固定+减小压铸件间隙）<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                                                        //
#define NEAREST_POSITION_DC_MOTOR_GB    250           //隔爆电机，关百叶允许的最大间隙。                                                       
#define FAREST_POSITION_DC_MOTOR_GB     1600          //隔爆电机，开百叶允许的最大间隙。


#define DOOR_POSITION_OPENED_SENSON     0           //接近开关传感器反馈的开门到位
#define DOOR_POSITION_CLOSEED_SENSON    1           //接近开关传感器反馈的关门到位
#define DOOR_POSITION_UNCERTAIN_SENSON  2           //接近开关传感器反馈的未知位置

#define MOTOR_A                         0
#define MOTOR_B                         1
#define DOOR_POWER_DOWN_UNHAPPEN        0           //本次控制循环还未发生过主动给百叶电机断电
#define DOOR_POWER_DOWN_HAPPEN          1           //本次控制循环已经发生过主动给百叶电机断电

#define STOP_MOTOR                      0
#define RUN_MOTOR                       1

#define RUN_STA                         0
#define STOP_STA                        1

#define ACCELERATION                    0
#define DECELERATION                    1
#define CONSTANT                        2
#define HOLD                            3
#define LASH                            4

// // 2相4拍
// #define LASH_STEP                       50000
// #define ACC_STEP_0                      125
// #define ACC_STEP_1                      125
// #define ACC_STEP_2                      125
// #define ACC_STEP_3                      120
// #define ACC_STEP_4                      115
// #define ACC_STEP_5                      110
// #define ACC_STEP_6                      105
// #define ACC_STEP_7                      100
// #define CONST_STEP                      100
// #define DEC_STEP_0                      100
// #define DEC_STEP_1                      105
// #define DEC_STEP_2                      110
// #define DEC_STEP_3                      115
// #define DEC_STEP_4                      120
// #define DEC_STEP_5                      125
// #define DEC_STEP_6                      125
// #define DEC_STEP_7                      125

//2相8拍
#define LASH_STEP                       50000
#define ACC_STEP_0                      250
#define ACC_STEP_1                      250
#define ACC_STEP_2                      250
#define ACC_STEP_3                      250
#define ACC_STEP_4                      250
#define ACC_STEP_5                      250
#define ACC_STEP_6                      250
#define ACC_STEP_7                      150
#define CONST_STEP                      80
#define DEC_STEP_0                      80
#define DEC_STEP_1                      80
#define DEC_STEP_2                      80
#define DEC_STEP_3                      80
#define DEC_STEP_4                      150
#define DEC_STEP_5                      150
#define DEC_STEP_6                      150
#define DEC_STEP_7                      150

#define ACC_STEP_MAX                    8
#define DEC_STEP_MAX                    8

#define CONST_STEP_LOW_TEMPLATE         250       //200pps , 在低温-10°C 及以下时，采用恒定的 200pps 低速运行。

#define PHASE_0                         0
#define PHASE_1                         1
#define PHASE_2                         2
#define PHASE_3                         3
#define PHASE_4                         4
#define PHASE_5                         5
#define PHASE_6                         6
#define PHASE_7                         7

// // 2相4拍
// #define PHASE_MAX                       PHASE_3
// // 2相8拍
#define PHASE_MAX                       PHASE_7


#define DIGITAL_SCALE_12BITS             ((uint32_t) 0xFFF)
/* Init variable out of ADC expected conversion data range */
#define VAR_CONVERTED_DATA_INIT_VALUE    (DIGITAL_SCALE_12BITS + 1)
/* Definition of ADCx conversions data table size */
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  8)

#define ADC_SAMPLING_PERIOD             20
#define ADC_DIVISION_VAL                50

#define ADC_LOGIC_CHANNEL_0             0
#define ADC_LOGIC_CHANNEL_1             1
#define ADC_LOGIC_CHANNEL_2             2
#define ADC_LOGIC_CHANNEL_3             3
#define ADC_LOGIC_CHANNEL_4             4
#define ADC_LOGIC_CHANNEL_5             5
#define ADC_LOGIC_CHANNEL_6             6
#define ADC_LOGIC_CHANNEL_7             7
#define ADC_HEATER_TEMP_CH              ADC_LOGIC_CHANNEL_0
#define ADC_POWER_VALID_CH              ADC_LOGIC_CHANNEL_1
#define ADC_HEATER_CURRENT              ADC_LOGIC_CHANNEL_2
#define ADC_FAN_CURRENT_CH              ADC_LOGIC_CHANNEL_3
#define ADC_M1_CURRENT_CH               ADC_LOGIC_CHANNEL_4
#define ADC_M2_CURRENT_CH               ADC_LOGIC_CHANNEL_5
#define ADC_BOARD_TEMP_CH               ADC_LOGIC_CHANNEL_6
#define ADC_MCU_TEMP_CH                 ADC_LOGIC_CHANNEL_7

#define ADC_FAN_MAX                     10000
#define ADC_FAN_MIN                     500
#define ADC_MOTOR_MAX                   27000
#define ADC_MOTOR_MIN                   1000

#define MOTOR1_LOGIC_CHN                0
#define MOTOR2_LOGIC_CHN                1
#define MOTOR3_LOGIC_CHN                2
#define MOTOR4_LOGIC_CHN                3

#define MOTOR_IN_1_CHN                  MOTOR2_LOGIC_CHN        //进风百叶1
#define MOTOR_OUT_CHN                   MOTOR3_LOGIC_CHN        //排风百叶


#define H_BRIDGE_STATE_INIT             0
#define H_BRIDGE_STATE_OFF              1
#define H_BRIDGE_STATE_WAIT             2
#define H_BRIDGE_STATE_OPEN_L           3
#define H_BRIDGE_STATE_OPEN_H           4


#define VALID                           1
#define INVALID                         0
#define MACHINE_ERR                     1
#define MACHINE_OK                      0

#if(FAN_MODEL == FAN_MODEL_AC_75W)
#define FAN_EFFICACY_NUM_MAX            25

#endif
#if(FAN_MODEL == FAN_MODEL_DC_100W)
#define FAN_EFFICACY_NUM_MAX            80    //实际1秒会有194个FG信号，台达22053

#endif

#define OUT_STATUS_CLOSE                1
#define OUT_STATUS_OPEN                 0
#if (MOTOR_TYPE == 1)

#define MOTOR_EFFICACY_NUM_MAX          100    //开45° ，一共2600个值  这里只要有 1.7 °，就可以了。
#define MOTOR_INIT_EFFICACY_NUM_MAX     100
#else

#define MOTOR_EFFICACY_NUM_MAX          10    //开85° ，一共490/300个值  这里只要有 2 °，就可以了。
#define MOTOR_INIT_EFFICACY_NUM_MAX     2

#endif

//>>>>>>>>>>>>>>>>以下是直流有刷推杆电机和福佑风机的电流参数表>>>>>>>>>>>>>>>>>


#define DC_MOTOR0_MA		VALID                   //PCB 位号 P5       //排风
#define DC_MOTOR1_MA		VALID                 //PCB 位号 P6       //进风
#define DC_MOTOR2_MA		INVALID                 //实际没有这个通道
#define DC_MOTOR3_MA		INVALID                 //实际没有这个通道





//<<<<<<<<<<<<<<<<以上是直流有刷推杆电机和福佑风机的电流参数表>>>>>>>>>>>>>>>>






#ifdef __cplusplus
}
#endif

#endif /* __COMMON_H */

