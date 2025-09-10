#ifndef _stm32_flash_H
#define _stm32_flash_H
#include "main.h"
#include "stm32g0xx.h"
#include "stm32g0xx_hal.h"

// #define FLASH_WAITETIME  	50000
#define INFO_NUMBER_PAGES   64     //2page ,total 64, 2 * 32 = 64 
#define FLASH_PAGE_0                        0
#define FLASH_PAGE_4                        4
#define FLASH_PAGE_30                       30
#define FLASH_PAGE_31                       31

#define FLASH_USER_START_ADDR               (FLASH_BASE)
#define FLASH_USER_END_ADDR                 (FLASH_BASE + 32 * FLASH_PAGE_SIZE)  //total 64K Byte = 32 * 2K
#define FLASH_SYSTEM_PARAM_1_ADDR           (FLASH_BASE + FLASH_PAGE_30 * FLASH_PAGE_SIZE)  //SYSTEM_PARAM 1 * 2K ： 1 page
#define FLASH_SYSTEM_PARAM_2_ADDR           (FLASH_BASE + FLASH_PAGE_31 * FLASH_PAGE_SIZE)  //SYSTEM_PARAM 1 * 2K ： 1 page

typedef enum {
	APPLICATION_NORMAL = 0,		// APP能正常稳定运行
	APPLICATION_UPDATED,		// APP刚更新完成，等待测试启动,实际不存在此状态
	APPLICATION_ERROR,			// APP错误，不能正常工作
}Application_Status_t;

typedef	struct SystemParamTypeDef{      // TOTAL : 4 * 16 = 64 Byte  ,  2KB / 64B = 32 ,结构体长度必须是 2048 的整数因子
	uint32_t Device_id;					// 设备号
	uint32_t Hardware_Version;			// 硬件版本信息
	uint32_t Application0_Version;		// APP0软件版本
	uint32_t Application1_Version;		// APP1软件版本
	uint32_t Application0_Status;		// APP0的状态. @Application_Status_t
	uint32_t Application1_Status;		// APP1的状态. @Application_Status_t
	uint32_t Modbus_Address;			// Modbus 本机地址
	uint32_t Modbus_Key;				// Modbus的登录密钥，随时更新
	uint32_t TimeStamp;					// 时间戳
    uint32_t FlashWriteCount;           // FLASH写的次数，本字段/64 = Flash Erase 寿命次数
    uint32_t InMotorSta;                // 进风电机短路状态信息  0:未短路  1：短路
    uint32_t Reserve[4];                // 预留字段
	uint32_t SystemParamCRC32;          // 参数CRC32值
}stSystemParamTypeDef;


void FlashRead_8(uint32_t address, uint8_t* data, uint32_t size);
void FlashRead_32(uint32_t address, uint32_t* data, uint32_t size);
HAL_StatusTypeDef FlashProgram(uint32_t address, uint64_t* data, uint32_t size);
HAL_StatusTypeDef Flash_Erase(uint32_t firstPage, uint32_t nbPages);

int GetSystemParamInfo(stSystemParamTypeDef *info);
HAL_StatusTypeDef UpdataSystemParamInfo(stSystemParamTypeDef info);
void InvertUint8(uint8_t* dBuf, uint8_t* srcBuf);
void InvertUint16(uint16_t* dBuf, uint16_t* srcBuf);
void InvertUint32(uint32_t* dBuf, uint32_t* srcBuf);
uint16_t CRC16(uint8_t* puchMsg, uint32_t usDataLen);
uint32_t CRC32(uint8_t* puchMsg, uint32_t usDataLen);
void CopyByte(uint8_t* dec,uint8_t* src,uint32_t size);

#endif
