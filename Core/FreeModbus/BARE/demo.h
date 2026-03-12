#ifndef _DEMO_H
#define _DEMO_H
#include "stm32g0xx_hal.h"
//MODBUS 宏定义
//3x 地址类型
#define MACHINE_WORK_MODE_R_REG               0x01
#define OUT_WINDOWS_STATUS_R_REG              0x02
#define IN_WINDOWS1_STATUS_R_REG              0x03
#define IN_WINDOWS2_STATUS_R_REG              0x04
#define OUT_WINDOWS_CUR_VAL_R_REG             0x05
#define IN_WINDOWS1_CUR_VAL_R_REG             0x06
#define IN_WINDOWS2_CUR_VAL_R_REG             0x07
#define FAN1_STATUS_R_REG                     0x08
#define FAN1_CURRENT_R_REG                    0x09
#define FAN2_STATUS_R_REG                     0x0A
#define FAN2_CURRENT_R_REG                    0x0B
#define COMMUNICATION_STATUS_R_REG            0x0C
#define TEMPERATURE_VAL_R_REG                 0x0D

#define FAN_RPM_TOTAL_R_REG                   0x20
#define FAN1_RPM_R_REG                        0x21
#define FAN2_RPM_R_REG                        0x22
#define NTC_TEMPERATURE_VAL_R_REG             0x23
#define FAN_STATUS2_R_REG                     0x24
#define NUMBER_CHANGE_ADDRESS_R_REG           0x25  //上电后MUDBUS地址更改次数，不允许超过100次，为了防止flash寿命问题。
#define NUMBER_FLASH_WRITE_TOTAL_R_REG        0x26  //flash 累计写的次数 / 64 = 擦写总次数

#define SORFWARE_VERSION_R_REG                0x30

//4x 地址类型
#define MACHINE_CTRL_WR_REG                   0x01
#define MACHINE_AUTO_MODE_WR_REG              0x02
#define MACHINE_TEST_MODE_WR_REG              0x06
#define WORK_MODE_WR_REG                      0x10
//4x 地址类型 只保留地址
#define ADDR_CHANGE_NUM_WR_REG                0x11   //临时代码。真实代码要删除、上电后MUDBUS地址更改次数，不允许超过100次，为了防止flash寿命问题。
#define FLASH_WRITE_TOTAL_NUM_WR_REG          0x12   //临时代码。真实代码要删除、flash 累计写的次数 / 64 = 擦写总次数
#define MODBUS_ADDRESS_WR_REG                 0x20
//寄存器值定义
//设备工作模式
#define CHECK_SELF_MODE                       0x00   //自检模式
#define WAITTING_MODE                         0x01   //待机模式
#define RUNNINT_MODE                          0x02   //运行模式
#define FAILURE_MODE                          0x03   //故障模式
#define UPDATA_MODE                           0x05   //升级模式
#define TEST_MODE                             0x06   //测试模式
//百叶状态
#define WINDOWS_OPEN_MODE                     0x00   //开启模式
#define WINDOWS_CLOSE_MODE                    0x01   //关闭模式
#define WINDOWS_FAILURE_MODE                  0x02   //故障模式
//风扇状态
#define FAN_OPEN_MODE                         0x00   //开启模式
#define FAN_CLOSE_MODE                        0x01   //关闭模式
#define FAN_FAILURE_MODE                      0x02   //故障模式
//通信状态
#define CONNECT_MODE                          0x00   //通信连接模式
#define DISCONNECT_MODE                       0x01   //通信断开模式
//系统待机风扇未停转故障
#define NORMAL_MODE                           0x00   //正常模式
#define ABNORMAL_MODE                         0x01   //异常模式
//风机控制
#define ON_MODE_PARAMS                        0x00AA //风机开启
#define OFF_MODE_PARAMS                       0x0055 //风扇关闭
#define TEST_MODE_PARAMS                      0x00BB //测试模式
#define OUT_TEST_MODE_PARAMS                  0x0066 //退出测试模式   

typedef struct MACHINE_MDBUS_STA
{
  uint16_t machineWorkMode;         //
  uint16_t outWindowsSta;           //排风百叶状态
  uint16_t inWindows1Sta;           //进风百叶1状态
  uint16_t inWindows2Sta;           //进风百叶2状态
  uint16_t outWindowsCurVal;        //排风百叶当前电流或角度值
  uint16_t inWindows1CurVal;        //进风百叶1当前电流或角度值
  uint16_t inWindows2CurVal;        //进风百叶2当前电流或角度值
  uint16_t fan1Sta;                 //风扇 1 状态
  uint16_t fan1Current;             //风扇 1 电流
  uint16_t fan2Sta;                 //风扇 2 状态
  uint16_t fan2Current;             //风扇 2 电流
  uint16_t communicationSta;        //
  uint16_t softwareVer;             //
  uint16_t fanRPMTotal;             //
  uint16_t fan1RPM;                 //
  uint16_t fan2RPM;                 //
  uint16_t temperature;             //
  uint16_t FanSta2;                 //
  uint16_t Ntctemperature; 
}stMACHINE_MDBUS_STA;



extern void InitModbus(void);
extern void UpdataModbusAddr(void);
extern uint16_t getCtrlStaModbus(void);
extern uint16_t getAutoStaModbus(void);
extern uint16_t getTestStaModbus(void);
extern void UpdataModbusAll(void);

#endif

