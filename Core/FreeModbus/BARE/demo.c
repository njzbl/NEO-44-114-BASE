/*
 * FreeModbus Libary: BARE Demo Application
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "demo.h"
#include "common.h"



/*----------------------------------------------------------------
���ƣ�         ��Ȧ
�洢����ʶ��   0XXXX
���ͣ�         λ
��д��         ��/д
�洢��Ԫ��ַ��(00001~0XXXX).XXXX:���豸���
�����룺       ��ȡ��Ȧ״̬ (0x01)��д������Ȧ (0x05)��д�����Ȧ (0x0F)
��;��         DO�̵���
//-----------------------------------------------------------------*/
#define REG_COILS_START     0x0001
#define REG_COILS_SIZE      16
unsigned char       ucRegCoilsBuf[REG_COILS_SIZE / 8]={0X00,0X00};

/*----------------------------------------------------------------
���ƣ�         ������Ȧ
�洢����ʶ��   1XXXX
���ͣ�         λ
��д��         ֻ��
�洢��Ԫ��ַ�� (10001~1XXXX).XXXX:���豸���
�����룺       ������״̬ (0x02)
��;��         DI�̵���
//-----------------------------------------------------------------*/
/* ----------------------- Defines ------------------------------------------*/
#define REG_DISC_START     0x0001
#define REG_DISC_SIZE      16
unsigned char ucRegDiscBuf[REG_DISC_SIZE / 8] = { 0x0F, 0XF0 };

/*----------------------------------------------------------------
���ƣ�         ����Ĵ���
�洢����ʶ��   3XXXX
���ͣ�         ��
��д��         ֻ��
�洢��Ԫ��ַ��(30001~3XXXX).XXXX:���豸���
�����룺       ������Ĵ��� (0x04)
��;��         ģ��������
//-----------------------------------------------------------------*/
#define REG_INPUT_START 0x0001
#define REG_INPUT_NREGS 256
static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegInputBuf[REG_INPUT_NREGS]=
{0x0001,
 0x0001,0x0001,0x0001,0x0001,0x0000,0x0000,0x0000,0x0000,
 0x0000,0x0001,0x0001,0x0001,0x0000,0x0000,0x0000,0x0000};


/*----------------------------------------------------------------
���ƣ�         ����/����Ĵ���
�洢����ʶ��   4XXXX
���ͣ�         ��
��д��         ��/д
�洢��Ԫ��ַ��(40001~4XXXX).XXXX:���豸���
�����룺       �����ּĴ��� (0x03)��д�����Ĵ��� (0x06)��д����Ĵ��� (0x10)
��;��         ģ�������
//-----------------------------------------------------------------*/
#if (MACHINE_MODE == MACHINE_HW_MODE || MACHINE_MODE == MACHINE_YGDY_MODE || MACHINE_MODE == MACHINE_NO_MODE)  //华为和阳光电源，采用软件强拉常闭触点，更为科学。但是阳光电源实际取消的故障和状态两个干接点信号输出。
#define REG_HOLDING_START 0x0021 //���ּĴ�����ʼ��ַ 

#endif

#if(MACHINE_MODE == MACHINE_HY_MODE)
#define REG_HOLDING_START 0x0001 //���ּĴ�����ʼ��ַ 

#endif

#define REG_HOLDING_NREGS 20    //���ּĴ�������
USHORT   usRegHoldingStart = REG_HOLDING_START;
USHORT   usRegHoldingBuf[REG_HOLDING_NREGS]=
{0x0000,
 0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,
 0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000};  


/*===================================================================
function name: eMBRegInputCB()
Description:   0x04 ����Ĵ������������������ݷ���BUF 
Attention��
Input Parameters: 
Returncode��
====================================================================*/
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

/*===================================================================
function name: eMBRegHoldingCB()
Description:   0x03��0x06.0x10 ���ּĴ������������������ݷ���BUF 
Attention��
Input Parameters: 
Returncode��
====================================================================*/
eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if((usAddress>=REG_HOLDING_START) && (usAddress+usNRegs<=REG_HOLDING_START+REG_HOLDING_NREGS) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
				*pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
				*pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
				iRegIndex++;

				usNRegs--;
            }
            break;

        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
				usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
			break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

/*===================================================================
function name: eMBRegCoilsCB()
Description:  0x01 ,0x05,0x0F ��Ȧ���������������ݷ���BUF
Attention��0x01�������DO����0x05,0x0fǿ�Ƶ����������Ȧ��DO��
Input Parameters: 
Returncode��
====================================================================*/
eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    
    int             iNCoils = ( int )usNCoils;
    unsigned short  usBitOffset;

    /* Check if we have registers mapped at this block. */
    if( ( usAddress >= REG_COILS_START ) &&( usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE ) )
    {
        usBitOffset = ( unsigned short )( usAddress - REG_COILS_START );
        switch ( eMode )
        {
                /* Read current values and pass to protocol stack. */
            case MB_REG_READ:
                while( iNCoils > 0 )
                {
                    *pucRegBuffer++ = xMBUtilGetBits( ucRegCoilsBuf, usBitOffset, ( unsigned char )( iNCoils > 8 ? 8 :iNCoils ) );
                    iNCoils -= 8;
                    usBitOffset += 8;
                }
                break;

                /* Update current register values. */
            case MB_REG_WRITE:
				while( iNCoils > 0 )
                {
                    xMBUtilSetBits( ucRegCoilsBuf, usBitOffset,( unsigned char )( iNCoils > 8 ? 8 : iNCoils ), *pucRegBuffer++ );
                    iNCoils -= 8;
                    usBitOffset += 8;
                }
                break;
        }

    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

/*===================================================================
function name: eMBRegDiscreteCB()
Description:  0x02 ������״̬/��ȡ����״̬���������������ݷ���BUF
Attention��
Input Parameters: 
Returncode��
====================================================================*/
eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    short           iNDiscrete = ( short )usNDiscrete;
    unsigned short  usBitOffset;

    /* Check if we have registers mapped at this block. */
    if( (usAddress >= REG_DISC_START) &&( usAddress + usNDiscrete<= REG_DISC_START + REG_DISC_SIZE ))
    {
        usBitOffset = (unsigned short)( usAddress - REG_DISC_START );
        while( iNDiscrete > 0 )
        {
            *pucRegBuffer++ =xMBUtilGetBits(ucRegDiscBuf, usBitOffset, (unsigned char)( iNDiscrete >8?8: iNDiscrete ) );
            iNDiscrete -= 8;
            usBitOffset+=8;
        }
    }
    else
    {
        eStatus = MB_ENOREG;//illegal register address
    }
    return eStatus;
}


/*****************************************************************************************************************************
 *                                                                  Function Source code
 ****************************************************************************************************************************/
#include "stm32g0xx_hal.h"
#include "tim.h"
#include "MainControl.h"
#include "adcSampling.h"
#include "common.h"
#include "stm32_flash.h"
#include "demo.h"

extern int GetSystemParamInfo(stSystemParamTypeDef *info);
extern HAL_StatusTypeDef UpdataSystemParamInfo(stSystemParamTypeDef info);

uint32_t mCountSetModbusAddr = 0;
extern stSystemParamTypeDef mSystemParamInfo;



/*****************************************************************************************************************************
 * InitModbus
 ****************************************************************************************************************************/

void InitModbus(void)
{   int record = 0;
    record = GetSystemParamInfo(&mSystemParamInfo);
    mSystemParamInfo.Hardware_Version = HARDWARE_VERSION;
    mSystemParamInfo.Application0_Version = SOFTWARE_VERSION;
    // if(record == 0) {
        mSystemParamInfo.Modbus_Address = MODBUS_ADDRESS_DEFAULT;       //默认是1
    // }

    usRegInputBuf[MACHINE_WORK_MODE_R_REG] = mMachineModbusSta.machineWorkMode;         //上电时默认处于自检状态，后面初始化结束显示待机状态；
    usRegInputBuf[OUT_WINDOWS_STATUS_R_REG] = mMachineModbusSta.outWindowsSta;     //上电时默认百叶处于关闭状态；
    usRegInputBuf[IN_WINDOWS_STATUS_R_REG] =  mMachineModbusSta.inWindowsSta;      //上电时默认百叶处于关闭状态；
    usRegInputBuf[FAN_STATUS_R_REG] = mMachineModbusSta.fanSta;                 //上电时默认风扇处于关闭状态；
    usRegInputBuf[FAN_CURRENT_R_REG] = mMachineModbusSta.fanCurrent;
    usRegInputBuf[COMMUNICATION_STATUS_R_REG] = mMachineModbusSta.communicationSta;         //上电时默认通信处于连接状态；
    usRegInputBuf[SORFWARE_VERSION_R_REG] = mMachineModbusSta.softwareVer;
    usRegInputBuf[FAN_RPM_TOTAL_R_REG] = mMachineModbusSta.fanRPMTotal;                           //上电时默认风扇转速处于0状态；
    usRegInputBuf[FAN1_RPM_R_REG] = mMachineModbusSta.fan1RPM;                                //上电时默认风扇转速处于0状态；
    usRegInputBuf[FAN2_RPM_R_REG] = mMachineModbusSta.fan2RPM;                                //上电时默认风扇转速处于0状态；
    usRegInputBuf[TEMPERATURE_VAL_R_REG] = mMachineModbusSta.temperature;       //MCU温度
    usRegInputBuf[FAN_STATUS2_R_REG] = mMachineModbusSta.FanSta2;                   //上电时默认百叶处于关闭状态；
    usRegInputBuf[NUMBER_CHANGE_ADDRESS_R_REG] = mCountSetModbusAddr; //MODBUS设置的次数；
    usRegInputBuf[NUMBER_FLASH_WRITE_TOTAL_R_REG] = mSystemParamInfo.FlashWriteCount; //MODBUS设置的总次数；
    

    usRegHoldingBuf[MACHINE_CTRL_WR_REG] = OFF_MODE_PARAMS;           //上电默认风机关闭
    usRegHoldingBuf[MODBUS_ADDRESS_WR_REG] = mSystemParamInfo.Modbus_Address;     //
    usRegHoldingBuf[WORK_MODE_WR_REG] = OUT_TEST_MODE_PARAMS;         //上电默认退出测试模式
    usRegHoldingBuf[ADDR_CHANGE_NUM_WR_REG] = mCountSetModbusAddr;    //代码测试时的字段，真实代码要删除
    usRegHoldingBuf[FLASH_WRITE_TOTAL_NUM_WR_REG] = mSystemParamInfo.FlashWriteCount;    //代码测试时的字段，真实代码要删除
    
    eMBInit(MB_RTU, (uint8_t) mSystemParamInfo.Modbus_Address, 2, 9600, MB_PAR_NONE);//MB_PAR_NONE
    eMBEnable();
}

void UpdataModbusAddr(void)
{
    UpdataModbusAll();
#if (MODBUS_CTRL == 1)
    usRegInputBuf[NUMBER_CHANGE_ADDRESS_R_REG] = mCountSetModbusAddr;
    if(mCountSetModbusAddr >= 100) {
        return ;
    }
    if(usRegHoldingBuf[MODBUS_ADDRESS_WR_REG] != (mSystemParamInfo.Modbus_Address & 0xff)) {
        if(usRegHoldingBuf[MODBUS_ADDRESS_WR_REG] <=20 && usRegHoldingBuf[MODBUS_ADDRESS_WR_REG] >= 1) {
            mCountSetModbusAddr++;
            // eMBDisable();
            stSystemParamTypeDef info;
            GetSystemParamInfo(&info);
            info.Hardware_Version = HARDWARE_VERSION;
            info.Application0_Version = SOFTWARE_VERSION;
            info.Modbus_Address = usRegHoldingBuf[MODBUS_ADDRESS_WR_REG];
            info.FlashWriteCount = (++mSystemParamInfo.FlashWriteCount);
            info.SystemParamCRC32 = CRC16((uint8_t*) &info, sizeof(stSystemParamTypeDef) - 4);
            UpdataSystemParamInfo(info);
            InitModbus();
        }
    }
#endif
}

uint16_t getCtrlStaModbus(void)
{
    return usRegHoldingBuf[MACHINE_CTRL_WR_REG];
}


// #define MACHINE_AUTO_MODE_WR_REG              0x01
// #define MODBUS_ADDRESS_WR_REG                 0x05
// #define MACHINE_TEST_MODE_WR_REG              0x06


uint16_t getAutoStaModbus(void)
{
    return usRegHoldingBuf[MACHINE_AUTO_MODE_WR_REG];
}

uint16_t getTestStaModbus(void)
{
    return usRegHoldingBuf[MACHINE_TEST_MODE_WR_REG];
}

 void UpdataModbusAll(void)
 {
    usRegInputBuf[MACHINE_WORK_MODE_R_REG] = mMachineModbusSta.machineWorkMode; 
    usRegInputBuf[OUT_WINDOWS_STATUS_R_REG] = mMachineModbusSta.outWindowsSta;
    usRegInputBuf[IN_WINDOWS_STATUS_R_REG] =  mMachineModbusSta.inWindowsSta;
    usRegInputBuf[FAN_STATUS_R_REG] = mMachineModbusSta.fanSta;
    usRegInputBuf[FAN_CURRENT_R_REG] = mMachineModbusSta.fanCurrent;
    usRegInputBuf[COMMUNICATION_STATUS_R_REG] = mMachineModbusSta.communicationSta;
    usRegInputBuf[SORFWARE_VERSION_R_REG] = mMachineModbusSta.softwareVer;
    usRegInputBuf[FAN_RPM_TOTAL_R_REG] = mMachineModbusSta.fanRPMTotal;
    usRegInputBuf[FAN1_RPM_R_REG] = mMachineModbusSta.fan1RPM;
    usRegInputBuf[FAN2_RPM_R_REG] = mMachineModbusSta.fan2RPM;
    usRegInputBuf[TEMPERATURE_VAL_R_REG] = mMachineModbusSta.temperature;
    usRegInputBuf[FAN_STATUS2_R_REG] = mMachineModbusSta.FanSta2;
    usRegInputBuf[NTC_TEMPERATURE_VAL_R_REG] =  mMachineModbusSta.Ntctemperature;

    for(int i = 0 ;i < 10;i++) {
        usRegInputBuf[20 + i] =  usRegHoldingBuf[i];
    }
 }
