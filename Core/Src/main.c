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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

void send_char(char c)
{
	while (!(USART3->ISR & USART_ISR_TXE))
	{
		
	}
	USART3->TDR = c;
}

	
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

  /* USER CODE BEGIN SysInit */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable GPIOB to RCC
	
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; //enable USART3 to RCC
	
	//Setting Alternate Function Mode in the MODERs for PB10 and PB11
	GPIOB->MODER |= (1 << 21) | (1 << 23); //AF mode
	
	//PB10 and PB11 are USART3 capable and on AF4
	
	//Setting up Alternate Functions for pins PB10 and PB11
	GPIOB->AFR[1] &= ~(0xF << 8); //Clear Register bits for PB10 AFR
	GPIOB->AFR[1] &= ~(0xF << 12); //Clear Register bits for PB11 AFR
	GPIOB->AFR[1] |= (1<<10) | (1<<14); //Set PB10 PB11 to AF4
	//Note that in AFR High, AF4 is 0100, and PB10 is bits 8-11, and PB11 is bits 12-15
	
	//you may need this instead of the above
	//GPIOA->AFR[0] |= 0x04 << GPIO_AFRL_AFRL4_Pos; /* (3) */
	//GPIOA->AFR[1] |= (0x02 << GPIO_AFRL_AFRH8_Pos) | (0x02 <<
	//GPIO_AFRL_AFRH9_Pos); /* (4) */
	
	USART3->BRR = HAL_RCC_GetHCLKFreq()/115200;
	USART3->CR1 = USART_CR1_TE | USART_CR1_UE; //Enable TX
	USART3->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE; //Enable RX
	
//	/* Polling idle frame Transmission */
//	while ((!USART3->ISR & USART_ISR_TC) != USART_ISR_TC)
//	{
//	/* add time out here for a robust application */
//	}
//	USART3->ICR |= USART_ICR_TCCF; /* Clear TC flag */
//	USART3->CR1 |= USART_CR1_TCIE; /* Enable TC interrupt */
	
	
//	// ---------Code example
//	//Setting up USART TX
//	/* (1) Oversampling by 16, 9600 baud */
//	/* (2) 8 data bit, 1 start bit, 1 stop bit, no parity */
//	USART3->BRR = 480000 / 96; /* (1) */
//	USART3->CR1 = USART_CR1_TE | USART_CR1_UE; /* (2) */
//	//Setting up USART RX
//	/* (1) oversampling by 16, 9600 baud */
//	/* (2) 8 data bit, 1 start bit, 1 stop bit, no parity, reception mode */
//	USART3->BRR = 480000 / 96; /* (1) */
//	USART3->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE; /* (2) */
//  // ------------

	
	

	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//HAL_Delay(1000);
		char gfpizzaisgood = 'y';
		send_char(gfpizzaisgood);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
