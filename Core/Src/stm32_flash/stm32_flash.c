#include "stm32_flash.h"
#include "main.h"
#include "gpio.h"


static FLASH_EraseInitTypeDef EraseInitStruct;

HAL_StatusTypeDef Flash_Erase(uint32_t firstPage, uint32_t nbPages)
{
    HAL_StatusTypeDef re;
    uint32_t pageError = 0;
    HAL_FLASH_Unlock();
    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page        = firstPage;
    EraseInitStruct.NbPages     = nbPages;
    re = HAL_FLASHEx_Erase(&EraseInitStruct, &pageError);
    if(re != HAL_OK) {
        printf("Erase pageError = %d\r\n",pageError);
    }
    HAL_FLASH_Lock();
    return re;
}

HAL_StatusTypeDef FlashProgram(uint32_t address, uint64_t* data, uint32_t size)
{
    HAL_StatusTypeDef re;
    uint32_t addressCur;
    if(address < FLASH_USER_START_ADDR || address > FLASH_USER_END_ADDR)
        return HAL_ERROR;
    HAL_FLASH_Unlock();
    addressCur = address;
    for(uint32_t i = 0; i < size; i++) {
        re = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addressCur, data[i]);
        if(re == HAL_OK) {
            addressCur = addressCur + 8;
        }
        else {
            printf("Program re = %d\r\n",re);
            break;
        }
    }
    HAL_FLASH_Lock();
    return re;
}

void FlashRead_8(uint32_t address, uint8_t* data, uint32_t size)
{
    uint32_t addressCur;
    addressCur = address;
    for(uint32_t i = 0; i < size; i++) {
        data[i] = *(__IO uint8_t *)addressCur;
        addressCur = addressCur + 4;
    }
}

void FlashRead_32(uint32_t address, uint32_t* data, uint32_t size)
{
    uint32_t addressCur;
    addressCur = address;
    for(uint32_t i = 0; i < size; i++) {
        data[i] = *(__IO uint32_t *)addressCur;
        addressCur = addressCur + 4;
    }
}

// int GetNumberSystemParam(void)
// {
//     uint8_t count[64],number1,number2;
//     FlashRead_8(FLASH_SYSTEM_PARAM_1_ADDR, count, 64);
//     for(number1 = 0; number1 < 62; number1++) {
//         if(count[number1] == 0xff) {
//             break;
//         }
//     }
//     FlashRead_8(FLASH_SYSTEM_PARAM_2_ADDR + , count, 64);
//     for(number2 = 0; number2 < 62; number2++) {
//         if(count[number2] == 0xff) {
//             break;
//         }
//     }
//     if(number1 < number2)
//         number1 = number2;
//     if(number1 >= 62) {
//         number1 = 61;    //0~61
//     }
//     return number1;
// }

//返回值    0: 一条记录也没写，info无效
//          1 ~ INFO_NUMBER_PAGES : 已经写的条目数
int GetSystemParamInfo(stSystemParamTypeDef *info)
{
    // uint8_t number;
    // uint32_t crc32Val;
    // int re = HAL_ERROR;
    // number = GetNumberSystemParam();
    // FlashRead_32(FLASH_SYSTEM_PARAM_1_ADDR + 64 * (number / 31)+ (number % 31) * sizeof(stSystemParamTypeDef), (uint32_t*)info, sizeof(stSystemParamTypeDef) >> 2);
    // crc32Val = CRC32((uint8_t*)info, sizeof(stSystemParamTypeDef));
    // if(crc32Val != 0) {
    //     return HAL_ERROR;
    // }
    // return HAL_OK;

    int re = 0;
    IWDG_FeedDog();
    stSystemParamTypeDef infoTemp;
    FlashRead_32(FLASH_SYSTEM_PARAM_1_ADDR, (uint32_t*) &infoTemp, sizeof(stSystemParamTypeDef) >> 2);
    if(infoTemp.Hardware_Version == 0xffffffff) {  //第一个位置还没写，那说明就是最后一个位置是有效的，否则就读到最后一个有效的数据为止
        IWDG_FeedDog();
        FlashRead_32(FLASH_SYSTEM_PARAM_1_ADDR + (INFO_NUMBER_PAGES - 1) * sizeof(stSystemParamTypeDef), (uint32_t*)&infoTemp, sizeof(stSystemParamTypeDef) >> 2);
        if(infoTemp.Hardware_Version != 0xffffffff) {
            info[0] = infoTemp;            
            return INFO_NUMBER_PAGES;
        }
        else {  //说明一条记录都没写呢。
            return 0;
        }
    }
    else {
        info[0] = infoTemp;
        re = 1;
    }

    for(int32_t i = 1; i < INFO_NUMBER_PAGES; i++) {
        IWDG_FeedDog();
        
        FlashRead_32(FLASH_SYSTEM_PARAM_1_ADDR + i * sizeof(stSystemParamTypeDef), (uint32_t*)&infoTemp, sizeof(stSystemParamTypeDef) >> 2);
        if(infoTemp.Hardware_Version != 0xffffffff) {
            // printf("FlashRead_32 03_%d, %d\r\n",i, infoTemp.Hardware_Version);
            info[0] = infoTemp;
            re = i + 1;     //最后一条记录
        }
        else {
            break;
        }
    }
    return re;
}

HAL_StatusTypeDef UpdataSystemParamInfo(stSystemParamTypeDef info)
{
    stSystemParamTypeDef infoTemp;
    HAL_StatusTypeDef re;
    uint8_t number;
    number = GetSystemParamInfo(&infoTemp);
    IWDG_FeedDog();
    if(number == INFO_NUMBER_PAGES) {
        number = 0;
    }
    re = FlashProgram(FLASH_SYSTEM_PARAM_1_ADDR + number * sizeof(stSystemParamTypeDef), (uint64_t*) &info, sizeof(stSystemParamTypeDef) >> 3);
    IWDG_FeedDog();
    
    if(number == 2) {
        printf("\r\nFlash_Erase(FLASH_PAGE_31);\r\n");
        re = Flash_Erase(FLASH_PAGE_31,  1);
    }
    else if(number == INFO_NUMBER_PAGES - 1) {
        printf("\r\nFlash_Erase(FLASH_PAGE_30);\r\n");
        re = Flash_Erase(FLASH_PAGE_30,  1);
    }
    return re;
}

// CRC32算法：
void InvertUint8(uint8_t* dBuf, uint8_t* srcBuf)
{
    uint32_t i;
    uint8_t tmp[4];
    tmp[0] = 0;
    for (i = 0; i < 8; i++) {
        if (srcBuf[0] & (1 << i))
            tmp[0] |= 1 << (7 - i);
    }
    dBuf[0] = tmp[0];
}
void InvertUint16(uint16_t* dBuf, uint16_t* srcBuf)
{
    uint32_t i;
    uint16_t tmp[4];
    tmp[0] = 0;
    for (i = 0; i < 16; i++) {
        if (srcBuf[0] & (1 << i))
            tmp[0] |= 1 << (15 - i);
    }
    dBuf[0] = tmp[0];
}

void InvertUint32(uint32_t* dBuf, uint32_t* srcBuf)
{
    uint32_t i;
    uint32_t tmp[4];
    tmp[0] = 0;
    for (i = 0; i < 32; i++) {
    if (srcBuf[0] & (1 << i))
        tmp[0] |= 1 << (15 - i);
    }
    dBuf[0] = tmp[0];
}

uint16_t CRC16(uint8_t* puchMsg, uint32_t usDataLen)
{
    uint16_t wCRCin = 0x0000;
    uint16_t wCPoly = 0x1021;
    uint8_t wChar = 0;

    while (usDataLen--) {
	wChar = *(puchMsg++);
	// InvertUint8(&wChar,&wChar);
	wCRCin ^= (wChar << 8);
	for (uint32_t i = 0; i < 8; i++) {
		if (wCRCin & 0x8000)
		wCRCin = (wCRCin << 1) ^ wCPoly;
	    else
		wCRCin = wCRCin << 1;
	}
    }
    // InvertUint16(&wCRCin,&wCRCin);
    return wCRCin; // return (wCRCin^0xFFFF) ;//
}

uint32_t CRC32(uint8_t* puchMsg, uint32_t usDataLen)
{
    uint32_t i;
    uint32_t wCRCin = 0xFFFFFFFF;
    uint32_t wCPoly = 0x04C11DB7;
    uint32_t wChar = 0;
    while (usDataLen--) {
        wChar = *(puchMsg++);
        InvertUint8((uint8_t*)&wChar, (uint8_t*)&wChar);
        wCRCin ^= (wChar << 24);
        for (i = 0; i < 8; i++) {
            if (wCRCin & 0x80000000)
                wCRCin = (wCRCin << 1) ^ wCPoly;
            else
                wCRCin = wCRCin << 1;
        }
    }
    InvertUint32(&wCRCin, &wCRCin);
    return (wCRCin ^ 0xFFFFFFFF);
}

void CopyByte(uint8_t* dec,uint8_t* src,uint32_t size)
{
    uint32_t i;
    for(i = 0;i < size; i++) {
        dec[i] = src[i];
    }
}

void test(void)
{
    stSystemParamTypeDef info;
    stSystemParamTypeDef infoTemp;
    uint32_t crc32;
    int number = 0;
    HAL_StatusTypeDef re;

    for(int i = 0; i < 128; i++) {



    number = GetSystemParamInfo(&info);
    info.Device_id = number;
    info.Hardware_Version = number;
    info.Application0_Version = number;
    info.Application1_Version = number;
    info.Application0_Status = number;
    info.Application1_Status = number;
    info.Modbus_Key = number;
    IWDG_FeedDog();

    info.Modbus_Address = number;
    info.SystemParamCRC32 = CRC16((uint8_t*) &info, sizeof(stSystemParamTypeDef) - 4);
    re = UpdataSystemParamInfo(info);
    IWDG_FeedDog();
    // printf("re = %d\r\n",re);
    number = GetSystemParamInfo(&infoTemp);
    IWDG_FeedDog();
    // printf("infoTemp.SystemParamCRC32 = 0x%x\r\n",infoTemp.SystemParamCRC32);
    crc32 = CRC16((uint8_t*) &infoTemp, sizeof(stSystemParamTypeDef) - 4);
    if(infoTemp.SystemParamCRC32 == crc32 && infoTemp.SystemParamCRC32 == info.SystemParamCRC32) {
        printf("%d ", infoTemp.Device_id);
    }
    else {
        printf("info = %d,%d,%d,%d,%d,%d,%d,%d,0x%x,0x%x,0x%x  --------  %d \r\n", infoTemp.Device_id,
                                                    infoTemp.Hardware_Version,
                                                    infoTemp.Application0_Version,
                                                    infoTemp.Application1_Version,
                                                    infoTemp.Application0_Status,
                                                    infoTemp.Application1_Status,
                                                    infoTemp.Modbus_Address,
                                                    infoTemp.Modbus_Key,
                                                    infoTemp.SystemParamCRC32,
                                                    crc32,
                                                    info.SystemParamCRC32,
                                                    infoTemp.Modbus_Address);
    }
    IWDG_FeedDog();
    
    }
     printf("\r\n===================================\r\n");
    while(1) {
        // uint8_t flag = 0;
        IWDG_FeedDog();
        HAL_Delay(10);
        // if(flag == 0) {
        //     flag = 1;
        //     Flash_Erase(FLASH_PAGE_31,  1);
        //     Flash_Erase(FLASH_PAGE_30,  1);
        // }
    }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
