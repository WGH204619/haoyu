/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usart.h"
#include "memorymap.h"
#include "tim.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "123OLED.h"
#include "123DS18B20.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if(htim->Instance==TIM16)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_TIM_Base_Stop(&htim16);
}
}
void TIM16_IRQHandler(void)
{
HAL_TIM_IRQHandler(&htim16);
}
void HAL_UART_RxCpltCsllback(UART_HandleTypeDef *huaart)
{
	HAL_Delay(1);
}
void HAL_LPUART_IRQHandler(void)
{
HAL_UART_IRQHandler(&hlpuart1);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t words[]={1,2,3,4,5,6,7,8};
uint8_t data[3];
uint8_t data1[]="太干燥了，你需要浇水\n";
uint8_t data2[]="土壤有点潮湿，请注意通风\n";
uint8_t data3[]="有点暗了，今天的观照充足吗？\n";
uint8_t data4[]="好热，可以浇点水\n";
uint8_t data5[]="好冷，需要一点温暖\n";
uint8_t data6=0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_LPUART1_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  uint8_t Number;
  OLED_Init();
  OLED_ShowChar(1, 1, 'A');
  Number=DS18B20_Init();

  while(DS18B20_Init())
      {
          OLED_ShowString(2, 2, "wait");
          HAL_Delay(500);
      }
//HAL_UART_Transmit_IT(&hlpuart1, data1, sizeof(data1));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_UART_Receive_IT(&hlpuart1, words, sizeof(words));
	  Number=words[0];
//	  OLED_ShowNum(3, 1,Number,2);
	  if(Number==48)
		  {
		  HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
//		  OLED_ShowString(3, 1, "Turn on light");
		  }
	  if(Number==49)
		  {
		  HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
//		  OLED_ShowString(3, 1, "Turn off light");
		  }
	  if(Number==50)
	  	  {
		  HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
//		  OLED_ShowString(4, 1, "water on");
	  	  }
	  if(Number==51)
	  	  {
		  HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
//		  OLED_ShowString(4, 1, "water off");
	  	  }
	  OLED_ShowString(1, 1, "Lig:");
	  OLED_ShowString(2, 1, "Tem:");
	  OLED_ShowString(3, 1, "Hum:");
	 double one =DS18B20_Get_Temperature();
	 double ADC11_value1 = getADCByChannel(&hadc1,1);
	 double ADC11_value2 = getADCByChannel(&hadc1,2);
	 OLED_ShowNum(1, 5, ADC11_value1*10, 3);
	 OLED_ShowNum(2, 5, one/10, 3);
	 OLED_ShowNum(3, 5, ADC11_value2*10, 3);
	 one=one/10;
	 data[1]=ADC11_value1*10;
	 data[2]=ADC11_value2*10;
	 if((one<32&&one>5)&&(data[2]<30&&data[2]>10)&&(data[1]<25))
		 data6=0;
	 if(one>=32&&data6==0)
	 {
		 HAL_UART_Transmit_IT(&hlpuart1, data4, sizeof(data4));
		 one =DS18B20_Get_Temperature();
		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		 HAL_TIM_Base_Start_IT(&htim16);
		 data6=1;
	 }
	 else if(one<=5&&data6==0)
	 {
		 HAL_UART_Transmit_IT(&hlpuart1, data5, sizeof(data5));
		 one =DS18B20_Get_Temperature();
		 OLED_ShowNum(2, 5, one, 3);
		 data6=1;
	 }
	 if(data[2]>30&&data6==0)
	 {
		 HAL_UART_Transmit_IT(&hlpuart1, data1, sizeof(data1));
		 ADC11_value2 = getADCByChannel(&hadc1,2);
		 OLED_ShowNum(3, 5, ADC11_value2*10, 3);
		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		 HAL_TIM_Base_Start_IT(&htim16);
		 data6=1;
	 }
	 else if(data[2]<10&&data6==0)
	 {
		HAL_UART_Transmit_IT(&hlpuart1, data2, sizeof(data2));

		 data6=1;
	 }
	 if(data[1]>25&&data6==0)
	 {
		 HAL_UART_Transmit_IT(&hlpuart1, data3, sizeof(data3));
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);

		 while(data[1]>25)
		 {
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);

			ADC11_value1= getADCByChannel(&hadc1,1);
			data[1]=ADC11_value1*10;
			 OLED_ShowNum(1, 5, ADC11_value1*10, 3);

		 }

		 data6=1;
	 }
	 if(data[1]<25)
	 {

		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	 }
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

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV1);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_MSI);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLLSAI1.PLLN = 24;
  PeriphClkInitStruct.PLLSAI1.PLLP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLQ = RCC_PLLQ_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLR = RCC_PLLR_DIV8;
  PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_USBCLK|RCC_PLLSAI1_ADCCLK;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
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
