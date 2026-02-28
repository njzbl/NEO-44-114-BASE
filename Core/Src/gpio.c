/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "common.h"
#include "MainControl.h"
extern uint8_t mBDCSta[2];
extern uint8_t mMotorPowerEnabled;
extern __IO uint32_t mCMD510BPowerFlagA;
extern __IO uint32_t mCMD510BPowerFlagB;
extern __IO uint32_t mCMD510BPowerFlagC;
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_SYS_Pin|FAN_PWM_Pin|DC_FAN_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, SYS_STA_Pin|STS_ERR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_PWR_CTRL1_Pin|HEATER_CTRL_Pin|I2C_SCL_Pin|I2C_SDA_Pin
                          |DIR1_Pin|PWM1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AC_FAN_CTRL_Pin|MOTOR_PWR_CTRL2_Pin|BACK1_Pin|FRONT1_Pin
                          |PWM2_Pin|DIR2_Pin|RS485_RE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, BACK2_Pin|FRONT2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = LED_SYS_Pin|FAN_PWM_Pin|DC_FAN_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = CTRL_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CTRL_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PFPin PFPin */
  GPIO_InitStruct.Pin = SYS_STA_Pin|STS_ERR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin PAPin
                           PAPin PAPin */
  GPIO_InitStruct.Pin = MOTOR_PWR_CTRL1_Pin|HEATER_CTRL_Pin|I2C_SCL_Pin|I2C_SDA_Pin
                          |DIR1_Pin|PWM1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = AC_FAN_CTRL_Pin|MOTOR_PWR_CTRL2_Pin|BACK1_Pin|FRONT1_Pin
                          |PWM2_Pin|DIR2_Pin|RS485_RE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = FAN_FG_Pin|ON_STATE1_Pin|ON_STATE2_Pin|OFF_STATE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = OFF_STATE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OFF_STATE1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin */
  GPIO_InitStruct.Pin = BACK2_Pin|FRONT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 2 */

// #if (MACHINE_TYPE_CUSTOMER == NEO_400350_DLK_FB_NC_HW)
// void fixInitGPIO(void)
// {
//   GPIO_InitTypeDef GPIO_InitStruct = {0};
//   /*Configure GPIO pin : PtPin */
//   GPIO_InitStruct.Pin = ON_STATE1_Pin|ON_STATE2_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
// }

// GPIO_PinState getON_STATE1(void)
// {
//     return  HAL_GPIO_ReadPin(ON_STATE1_GPIO_Port, ON_STATE1_Pin);
// }
// GPIO_PinState getON_STATE2(void)
// {
//     return  HAL_GPIO_ReadPin(ON_STATE2_GPIO_Port, ON_STATE2_Pin);
// }

// #endif

//sta == 1 set LED0_Pin
//sat == 0 reset LED0_Pin
void setLED0(uint8_t sta)
{
    GPIO_PinState pinSta;
    if(sta == 1)
        pinSta = GPIO_PIN_SET;
    else
        pinSta = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, pinSta);
}

//sta == 1 set FAN_CTRL_Pin
//sat == 0 reset FAN_CTRL_Pin
void setACFanCtrl(uint8_t sta)
{
    GPIO_PinState pinSta;
    if(sta == 1)
        pinSta = GPIO_PIN_SET;
    else
        pinSta = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(AC_FAN_CTRL_GPIO_Port, AC_FAN_CTRL_Pin, pinSta);
}

//sta == 1 set FAN_CTRL_Pin
//sat == 0 reset FAN_CTRL_Pin
void setDCFanCtrl(uint8_t sta)
{
    GPIO_PinState pinSta;
    if(sta == 1)
        pinSta = GPIO_PIN_SET;
    else
        pinSta = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(DC_FAN_CTRL_GPIO_Port, DC_FAN_CTRL_Pin, pinSta);
}

//sta == 1 set FAN_CTRL_Pin
//sat == 0 reset FAN_CTRL_Pin
void setFanPWM(uint8_t sta)
{
    GPIO_PinState pinSta;
    if(sta == 1)
        pinSta = GPIO_PIN_SET;
    else
        pinSta = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(FAN_PWM_GPIO_Port, FAN_PWM_Pin, pinSta);
}

//sta == 1 set FEEDBACK_STA_Pin
//sat == 0 reset FEEDBACK_STA_Pin
void setSysSta(uint8_t sta)
{
    GPIO_PinState pinSta;
    if(sta == 1)
        pinSta = GPIO_PIN_SET;
    else
        pinSta = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(SYS_STA_GPIO_Port, SYS_STA_Pin, pinSta);
}

//sta == 1 set SYS_STA_Pin
//sat == 0 reset SYS_STA_Pin
void setSysErr(uint8_t sta)
{
    GPIO_PinState pinSta;
    if(sta == 1)
        pinSta = GPIO_PIN_SET;
    else
        pinSta = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(STS_ERR_GPIO_Port, STS_ERR_Pin, pinSta);
}

//sta == 1 set RS485_RE_Pin
//sat == 0 reset RS485_RE_Pin
void setRS485_RE(uint8_t sta)
{
    GPIO_PinState pinSta;
    if(sta == 1)
        pinSta = GPIO_PIN_SET;
    else
        pinSta = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, pinSta);
}

//sta == 1 set DIR1_Pin
//sat == 0 reset DIR1_Pin
void setDIR1(uint8_t sta)
{
    GPIO_PinState pinSta;
    if(sta == 1)
        pinSta = GPIO_PIN_SET;
    else
        pinSta = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, pinSta);
}

//sta == 1 set DIR2_Pin
//sat == 0 reset DIR2_Pin
void setDIR2(uint8_t sta)
{
    GPIO_PinState pinSta;
    if(sta == 1)
        pinSta = GPIO_PIN_SET;
    else
        pinSta = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, pinSta);
}

//sta == 1 set PWM1_Pin
//sat == 0 reset PWM1_Pin
void setPWM1(uint8_t sta)
{
    GPIO_PinState pinSta;
    if(sta == 1)
        pinSta = GPIO_PIN_SET;
    else
        pinSta = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(PWM1_GPIO_Port, PWM1_Pin, pinSta);
}

//sta == 1 set PWM2_Pin
//sat == 0 reset PWM2_Pin
void setPWM2(uint8_t sta)
{
    GPIO_PinState pinSta;
    if(sta == 1)
        pinSta = GPIO_PIN_SET;
    else
        pinSta = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(PWM2_GPIO_Port, PWM2_Pin, pinSta);
}

//chn : motor channel  0: exFan  1:inFan1  2:inFan2/exFan2
//sta : 0：motor reverse Rotation    1: motor forward rotate
void setDirPwm(uint8_t chn,uint8_t sta)
{
    if(chn == 0){
        setPWM1(0);
        setDIR1(sta);
    }
    else if(chn == 1){
        setPWM2(0);
        setDIR2(sta);
    }
    else if(chn == 2){

#if (MACHINE_THIRD_MOTOR == THIRD_MOTOR_ENABLED)
        setFanPWM(sta);
#endif
    }
}

void setBDCMotorForward(uint8_t sn)
{
    // if(mMotorPowerEnabled == 0) {
    //     setBDCMotorStop(sn);
    //     return;
    // }
    switch (sn)
    {
    case 0: {
        HAL_GPIO_WritePin(FRONT1_GPIO_Port, FRONT1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BACK1_GPIO_Port, BACK1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_PWR_CTRL1_GPIO_Port, MOTOR_PWR_CTRL1_Pin, GPIO_PIN_SET);
        mCMD510BPowerFlagA = 0;
        break;
    }
    case 1: {
        HAL_GPIO_WritePin(FRONT2_GPIO_Port, FRONT2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BACK2_GPIO_Port, BACK2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_PWR_CTRL2_GPIO_Port, MOTOR_PWR_CTRL2_Pin, GPIO_PIN_SET);
        mCMD510BPowerFlagB = 0;
        break;
    }
#if (MACHINE_THIRD_MOTOR == THIRD_MOTOR_ENABLED)

    case 2: {
        HAL_GPIO_WritePin(DC_FAN_CTRL_GPIO_Port, DC_FAN_CTRL_Pin, GPIO_PIN_SET);
        mCMD510BPowerFlagC = 0;
        break;
    }
#endif

    default:
        break;
    }
}

void setBDCMotorBack(uint8_t sn)
{
    // if(mMotorPowerEnabled == 0) {
    //     setBDCMotorStop(sn);
    //     return;
    // }
    switch (sn)
    {
    case 0: {        
        HAL_GPIO_WritePin(BACK1_GPIO_Port, BACK1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(FRONT1_GPIO_Port, FRONT1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_PWR_CTRL1_GPIO_Port, MOTOR_PWR_CTRL1_Pin, GPIO_PIN_SET);
        break;
    }
    case 1: {
        HAL_GPIO_WritePin(BACK2_GPIO_Port, BACK2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(FRONT2_GPIO_Port, FRONT2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_PWR_CTRL2_GPIO_Port, MOTOR_PWR_CTRL2_Pin, GPIO_PIN_SET);
        break;
    }
#if (MACHINE_THIRD_MOTOR == THIRD_MOTOR_ENABLED)
    case 2: {
        HAL_GPIO_WritePin(DC_FAN_CTRL_GPIO_Port, DC_FAN_CTRL_Pin, GPIO_PIN_SET);
        break;
    }
#endif

    default:
        break;
    }
}

void setBDCMotorStop(uint8_t sn)
{
    switch (sn)
    {
    case 0: {
        mMachineSta.motorPowerSta[MOTOR1_LOGIC_CHN] = MOTOR_POWER_DOWN_STA;
        HAL_GPIO_WritePin(MOTOR_PWR_CTRL1_GPIO_Port, MOTOR_PWR_CTRL1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(FRONT1_GPIO_Port, FRONT1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BACK1_GPIO_Port, BACK1_Pin, GPIO_PIN_RESET);
        mCMD510BPowerFlagA = 1;
        break;
    }
    case 1: {
        mMachineSta.motorPowerSta[MOTOR2_LOGIC_CHN] = MOTOR_POWER_DOWN_STA;
        HAL_GPIO_WritePin(MOTOR_PWR_CTRL2_GPIO_Port, MOTOR_PWR_CTRL2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(FRONT2_GPIO_Port, FRONT2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BACK2_GPIO_Port, BACK2_Pin, GPIO_PIN_RESET);
        mCMD510BPowerFlagB = 1;
        break;
    }
#if (MACHINE_THIRD_MOTOR == THIRD_MOTOR_ENABLED)
    case 2: {
        HAL_GPIO_WritePin(DC_FAN_CTRL_GPIO_Port, DC_FAN_CTRL_Pin, GPIO_PIN_RESET);
        mCMD510BPowerFlagC = 1;
        break;
    }
#endif

    default:
        break;
    }
}

// void setBDCMotorForward(uint8_t sn)
// {
//     if(mMotorPowerEnabled == 0) {
//         setBDCMotorStop(0);
//         return;
//     }
//     switch (sn)
//     {
//     case 0: {
//         if(mBDCSta[0] == 0 || mBDCSta[0] == 1 || mBDCSta[0] == 3) {
//             HAL_GPIO_WritePin(MOTOR_PWR_CTRL1_GPIO_Port, MOTOR_PWR_CTRL1_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(FRONT1_GPIO_Port, FRONT1_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(BACK1_GPIO_Port, BACK1_Pin, GPIO_PIN_RESET);
//             mBDCSta[0] = 2;
//             // printf("Forward mBDCSta[0] = 1; \r\n");
//         }
//         else if(mBDCSta[0] != 4) {
//             HAL_GPIO_WritePin(FRONT1_GPIO_Port, FRONT1_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(BACK1_GPIO_Port, BACK1_Pin, GPIO_PIN_SET);
//             HAL_GPIO_WritePin(MOTOR_PWR_CTRL1_GPIO_Port, MOTOR_PWR_CTRL1_Pin, GPIO_PIN_SET);
//             mBDCSta[0] = 4;
//             mCMD510BPowerFlagA = 0;
//             // printf("Forward mBDCSta[0] = 3; \r\n");
//         }
//         break;
//     }
//     case 1:
    
//         mCMD510BPowerFlagB = 0;

//         if(mBDCSta[1] == 0 || mBDCSta[1] == 1 || mBDCSta[1] == 3)  {
//             HAL_GPIO_WritePin(MOTOR_PWR_CTRL2_GPIO_Port, MOTOR_PWR_CTRL2_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(FRONT2_GPIO_Port, FRONT2_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(BACK2_GPIO_Port, BACK2_Pin, GPIO_PIN_RESET);
//             mBDCSta[1] = 2;
//         }
//         else if(mBDCSta[1] != 4) {
//             HAL_GPIO_WritePin(FRONT2_GPIO_Port, FRONT2_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(BACK2_GPIO_Port, BACK2_Pin, GPIO_PIN_SET);
//             HAL_GPIO_WritePin(MOTOR_PWR_CTRL2_GPIO_Port, MOTOR_PWR_CTRL2_Pin, GPIO_PIN_SET);
//             mBDCSta[1] = 4;
//             mCMD510BPowerFlagB = 0;
//         }
//         break;
//     default:
//         break;
//     }
// }

// void setBDCMotorBack(uint8_t sn)
// {
//     if(mMotorPowerEnabled == 0) {
//         setBDCMotorStop(0);
//         return;
//     }
//     switch (sn)
//     {
//     case 0: {
//         if(mBDCSta[0] == 0 || mBDCSta[0] == 2 || mBDCSta[0] == 4) {
//             HAL_GPIO_WritePin(MOTOR_PWR_CTRL1_GPIO_Port, MOTOR_PWR_CTRL1_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(FRONT1_GPIO_Port, FRONT1_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(BACK1_GPIO_Port, BACK1_Pin, GPIO_PIN_RESET);
//             mBDCSta[0] = 1;
//             // printf("Back mBDCSta[0] = 1; \r\n");
//         }
//         else if(mBDCSta[0] != 3) {
//             HAL_GPIO_WritePin(BACK1_GPIO_Port, BACK1_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(FRONT1_GPIO_Port, FRONT1_Pin, GPIO_PIN_SET);
//             HAL_GPIO_WritePin(MOTOR_PWR_CTRL1_GPIO_Port, MOTOR_PWR_CTRL1_Pin, GPIO_PIN_SET);
//             mBDCSta[0] = 3;
//             // printf("Back mBDCSta[0] = 3; \r\n");
//         }
//         break;
//     }
//     case 1:
//         if(mBDCSta[1] == 0 || mBDCSta[1] == 2 || mBDCSta[1] == 4) {
//             HAL_GPIO_WritePin(MOTOR_PWR_CTRL2_GPIO_Port, MOTOR_PWR_CTRL2_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(FRONT2_GPIO_Port, FRONT2_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(BACK2_GPIO_Port, BACK2_Pin, GPIO_PIN_RESET);
//             mBDCSta[1] = 1;
//         }
//         else if(mBDCSta[1] != 3) {
//             HAL_GPIO_WritePin(BACK2_GPIO_Port, BACK2_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(FRONT2_GPIO_Port, FRONT2_Pin, GPIO_PIN_SET);
//             HAL_GPIO_WritePin(MOTOR_PWR_CTRL2_GPIO_Port, MOTOR_PWR_CTRL2_Pin, GPIO_PIN_SET);
//             mBDCSta[1] = 3;
//         }
//         break;
//     default:
//         break;
//     }
// }

// void setBDCMotorStop(uint8_t sn)
// {
//     switch (sn)
//     {
//     case 0:
//         HAL_GPIO_WritePin(MOTOR_PWR_CTRL1_GPIO_Port, MOTOR_PWR_CTRL1_Pin, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(FRONT1_GPIO_Port, FRONT1_Pin, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(BACK1_GPIO_Port, BACK1_Pin, GPIO_PIN_RESET);
//         mBDCSta[0] = 0;
//         // setBRUSH1(0);				//é¿å…æœ�?�äº�?�ç�?�µæœºåœ¨ç�?�µæºæ�?�­ç�?�µåŽé€šè¿‡DIR ç½®é«˜å†å¾®å¼±ä¾�?�ç�?�µå�?�ºçŽ°FGä¿¡å· 0.6V ~ 0.7V çš„BUG
//         mCMD510BPowerFlagA = 1;
//         // printf("Stop mBDCSta[0] = 0; \r\n");
//         break;
//     case 1:
//         HAL_GPIO_WritePin(MOTOR_PWR_CTRL2_GPIO_Port, MOTOR_PWR_CTRL2_Pin, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(FRONT2_GPIO_Port, FRONT2_Pin, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(BACK2_GPIO_Port, BACK2_Pin, GPIO_PIN_RESET);
//         mBDCSta[1] = 0;
//         mCMD510BPowerFlagB = 1;
//         // setBRUSH2(0);				//é¿å…æœ�?�äº�?�ç�?�µæœºåœ¨ç�?�µæºæ�?�­ç�?�µåŽé€šè¿‡DIR ç½®é«˜å†å¾®å¼±ä¾�?�ç�?�µå�?�ºçŽ°FGä¿¡å· 0.6V ~ 0.7V çš„BUG   
//         break;
//     default:
//         break;
//     }
// }

GPIO_PinState getChkSta(void)
{
    return  HAL_GPIO_ReadPin(CTRL_1_GPIO_Port, CTRL_1_Pin);
}

/* USER CODE END 2 */
