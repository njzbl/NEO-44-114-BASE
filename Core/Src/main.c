/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mb.h"
#include "port.h"
#include "string.h"
#include "stdarg.h"
#include "stdio.h"
#include "MainControl.h"
#include "adcSampling.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern void InitModbus(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#pragma import(__use_no_semihosting)    //ȷ��û�д�C������ʹ�ð������ĺ��������뵼�����__use_no_semihosting
void _sys_exit(int x)                   //����_sys_exit()�Ա���ʹ�ð�����ģʽ
{
  x = x;
}
struct __FILE                           //��׼����Ҫ��֧�ֺ���
{
  int handle;
};
/* FILE is typedefed in stdio.h*/
FILE __stdout;
FILE __stdin;

/**
  * @brief     �ض���fputc�������ض���c�⺯��putchar��printf��USART1��
  * @param     ch - �ַ�
               *f - �ļ�ָ��
  * @retval    �ַ�
  * @attention ���ܷ����ж���ʹ�ã�����ɲο������������
  */


int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xffff);
	while ((USART2->ISR & UART_FLAG_TXE) == 0) {}  //�ȴ����ݷ������????????
  return ch;
}

void PrintfVersion(void)
{
#if (MACHINE_TYPE_CUSTOMER == NEO_400350_DW_BASE)  //常开触点机型
	printf("Software Version: NEO_400350_DW_BASE-20241029\r\n");
#endif
#if (MACHINE_TYPE_CUSTOMER == NEO_400350_DW_DCFAN)  //常开触点机型
	printf("Software Version: NEO_400350_DW_DCFAN-20241029\r\n");
#endif
#if(MACHINE_TYPE_CUSTOMER == NEO_320270_MAX_BASE)
    printf("Software Version: NEO_320270_MAX_BASE-V103-20250526-OM\r\n");
#endif
#if(MACHINE_TYPE_CUSTOMER == NEO_400350_DLK_TG_60W_BASE)
    printf("Software Version: NEO_400350_DLK_TG_60W_BASE-V103-20250526-OM\r\n");
#endif
#if(MACHINE_TYPE_CUSTOMER == NEO_400350_DLK_FB_NO_BASE)
    printf("Software Version: NEO_400350_DLK_FB_NO_BASE-V103-20250526-OM\r\n");
#endif
#if (MACHINE_TYPE_CUSTOMER == NEO_400350_DLK_FB_NC_HW)//华为
	printf("Software Version: NEO_400350_DLK_FB_NC_HW-V103-20260209\r\n");
#endif
#if(MACHINE_TYPE_CUSTOMER == NEO_400350_DLK_FB_NO_HY)
    printf("Software Version: NEO_400350_DLK_FB_NO_HY-V103-20250526-OM\r\n");
#endif
}

__IO   uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];
__IO   uint8_t ubDmaTransferStatus = 2;
uint16_t mAdcV[ADC_CONVERTED_DATA_BUFFER_SIZE] = {0};

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
	 UNUSED(hadc);
   ubDmaTransferStatus=1;//��1
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t i = 0,count = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  
#if (MACHINE_TYPE_CUSTOMER == NEO_400350_DLK_FB_NC_HW)
  fixInitGPIO();
#endif
  AHT20_IIC_GPIO_INIT();
  PrintfVersion();
  InitVar();
  setLED0(0);
#if (MACHINE_FEEDBACK_MODE == NORMALLY_CLOSE)  //华为和阳光电源，采用软件强拉常闭触点，更为科学�??
  setSysErr(OUT_STATUS_CLOSE);
#endif
#if (MACHINE_FEEDBACK_MODE == NORMALLY_OPEN)  //常开触点机型
  setSysErr(OUT_STATUS_OPEN);
#endif
  InitModbus();
    
  for (i = 0; i < ADC_CONVERTED_DATA_BUFFER_SIZE; i++)
  {
    aADCxConvertedData[i] = VAR_CONVERTED_DATA_INIT_VALUE;
  }
	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)//ADC��У׼
  {
    /* Calibration Error */
      printf("HAL_ADCEx_Calibration_Start is error.\r\n");
    while(1);
  }
	if (HAL_ADC_Start_DMA(&hadc1,
                        (uint32_t *)aADCxConvertedData,
                        ADC_CONVERTED_DATA_BUFFER_SIZE
                       ) != HAL_OK)
  {
    /* ADC conversion start error */
      printf("HAL_ADC_Start_DMA is error.\r\n");
    while(1);
  }  
  
	__HAL_TIM_CLEAR_IT(&htim16,TIM_IT_UPDATE);	
  __HAL_TIM_SetCounter(&htim16,0);
  if (HAL_TIM_Base_Start_IT(&htim16) != HAL_OK)
  {
      printf("htim16 is error.\r\n");
	    while(1);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    count++;
    if(count < 50) {
        IWDG_FeedDog();
    }
    else if(count < 100) {
        IWDG_FeedDog();
    }
    else if(count < 150) {
        count = 0;
    }

    MainControl();
    adcSampling();
    
    (void) eMBPoll();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
