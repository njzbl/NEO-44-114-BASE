/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

typedef struct INPUT_STA
{
    GPIO_PinState fg;
    GPIO_PinState faultM;
    GPIO_PinState chk;
    GPIO_PinState startQG;
    GPIO_PinState start485;
    GPIO_PinState start;
    GPIO_PinState yl01;
    GPIO_PinState yl02;
}stINPUT_STA;

typedef struct OUTPUT_STA
{
    uint8_t motorS1;
    uint8_t fanS2;
    uint8_t powerS3;
    uint8_t systemS4;
    uint8_t S5;
    uint8_t S6;
    uint8_t sta1;
    uint8_t sta2;
}stOUTPUT_STA;

#define H_BRIDGE_SIDE_L                     0
#define H_BRIDGE_SIDE_H                     1
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
#if (MACHINE_TYPE_CUSTOMER == NEO_400350_DLK_FB_NC_HW)
void fixInitGPIO(void);
GPIO_PinState getON_STATE1(void);
GPIO_PinState getON_STATE2(void);
#endif
void setLED0(uint8_t sta);
void setACFanCtrl(uint8_t sta);
void setDCFanCtrl(uint8_t sta);
void setFanPWM(uint8_t sta);
void setSysSta(uint8_t sta);
void setSysErr(uint8_t sta);
void setRS485_RE(uint8_t sta);
void setBDCMotorForward(uint8_t snMotor );
void setBDCMotorBack(uint8_t snMotor);
void setBDCMotorStop(uint8_t sn);
GPIO_PinState getChkSta(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

