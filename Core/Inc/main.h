/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_SYS_Pin GPIO_PIN_13
#define LED_SYS_GPIO_Port GPIOC
#define CTRL_1_Pin GPIO_PIN_14
#define CTRL_1_GPIO_Port GPIOC
#define FAN_PWM_Pin GPIO_PIN_15
#define FAN_PWM_GPIO_Port GPIOC
#define SYS_STA_Pin GPIO_PIN_0
#define SYS_STA_GPIO_Port GPIOF
#define STS_ERR_Pin GPIO_PIN_1
#define STS_ERR_GPIO_Port GPIOF
#define HEATER_TEMP_Pin GPIO_PIN_0
#define HEATER_TEMP_GPIO_Port GPIOA
#define ADC_POWER_Pin GPIO_PIN_1
#define ADC_POWER_GPIO_Port GPIOA
#define MOTOR_PWR_CTRL1_Pin GPIO_PIN_4
#define MOTOR_PWR_CTRL1_GPIO_Port GPIOA
#define HEATER_CURRENT_Pin GPIO_PIN_5
#define HEATER_CURRENT_GPIO_Port GPIOA
#define FAN_CURRENT_Pin GPIO_PIN_6
#define FAN_CURRENT_GPIO_Port GPIOA
#define HEATER_CTRL_Pin GPIO_PIN_7
#define HEATER_CTRL_GPIO_Port GPIOA
#define FAN_CTRL_Pin GPIO_PIN_0
#define FAN_CTRL_GPIO_Port GPIOB
#define MOTOR_CURRENT1_Pin GPIO_PIN_1
#define MOTOR_CURRENT1_GPIO_Port GPIOB
#define MOTOR_CURRENT2_Pin GPIO_PIN_2
#define MOTOR_CURRENT2_GPIO_Port GPIOB
#define T_ADC_Pin GPIO_PIN_10
#define T_ADC_GPIO_Port GPIOB
#define FAN_FG_Pin GPIO_PIN_11
#define FAN_FG_GPIO_Port GPIOB
#define FAN_FG_EXTI_IRQn EXTI4_15_IRQn
#define MOTOR_PWR_CTRL2_Pin GPIO_PIN_12
#define MOTOR_PWR_CTRL2_GPIO_Port GPIOB
#define BACK1_Pin GPIO_PIN_13
#define BACK1_GPIO_Port GPIOB
#define FRONT1_Pin GPIO_PIN_14
#define FRONT1_GPIO_Port GPIOB
#define ON_STATE1_Pin GPIO_PIN_15
#define ON_STATE1_GPIO_Port GPIOB
#define ON_STATE1_EXTI_IRQn EXTI4_15_IRQn
#define OFF_STATE1_Pin GPIO_PIN_8
#define OFF_STATE1_GPIO_Port GPIOA
#define OFF_STATE1_EXTI_IRQn EXTI4_15_IRQn
#define DIR1_Pin GPIO_PIN_11
#define DIR1_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_12
#define PWM1_GPIO_Port GPIOA
#define BACK2_Pin GPIO_PIN_2
#define BACK2_GPIO_Port GPIOD
#define FRONT2_Pin GPIO_PIN_3
#define FRONT2_GPIO_Port GPIOD
#define ON_STATE2_Pin GPIO_PIN_3
#define ON_STATE2_GPIO_Port GPIOB
#define ON_STATE2_EXTI_IRQn EXTI2_3_IRQn
#define OFF_STATE2_Pin GPIO_PIN_4
#define OFF_STATE2_GPIO_Port GPIOB
#define OFF_STATE2_EXTI_IRQn EXTI4_15_IRQn
#define PWM2_Pin GPIO_PIN_5
#define PWM2_GPIO_Port GPIOB
#define DIR2_Pin GPIO_PIN_8
#define DIR2_GPIO_Port GPIOB
#define RS485_RE_Pin GPIO_PIN_9
#define RS485_RE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
