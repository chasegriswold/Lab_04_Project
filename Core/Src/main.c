/* USER CODE BEGIN Header */
/** CHASE GRISWOLD
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

volatile char rx;
volatile uint8_t rx_flag;
volatile uint8_t led_flag;
volatile uint8_t toggle_flag;

volatile char led;



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void init_LEDs(void);
void send_char(char c);
void tx_string(char str[]);
void rx_char_1(void);
void check_input(void);
void turn_off_led(void);
void turn_on_led(void);
void toggle_led(void);


/**
  * @brief  The application entry point.
  * @retval int
  */


int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();

	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //enable GPIOC to RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable GPIOB to RCC
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; //enable USART3 to RCC
	
	//NOTE: PB10 and PB11 were selected based on being directly next to a ground pin.
	
	//Setting Alternate Function Mode in the MODERs for PB10 and PB11
	GPIOB->MODER |= (1 << 21) | (1 << 23); //AF mode
	//PB10 and PB11 are USART3 capable and on AF4
	
	//Setting up Alternate Functions for pins PB10 and PB11
	GPIOB->AFR[1] &= ~(0xF << 8); //Clear Register bits for PB10 AFR
	GPIOB->AFR[1] &= ~(0xF << 12); //Clear Register bits for PB11 AFR
	GPIOB->AFR[1] |= (1<<10) | (1<<14); //Set PB10 PB11 to AF4
	//Note that in AFR High, AF4 is 0100, and PB10 is bits 8-11, and PB11 is bits 12-15
	
	USART3->BRR = HAL_RCC_GetHCLKFreq()/115200;
	USART3->CR1 = 0;
	USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE); //Enable Tx/Rx
	
	// Enable the USART peripheral
	USART3->CR1 |= USART_CR1_UE;
	
	init_LEDs();
	
  rx_flag = 0;
	rx = NULL;
	led_flag = 0;
	toggle_flag = 0;
	
	// ---------- Second Checkoff setup here-------
	//Configure NVIC to enable USART3 interrupt, define interrupt priority.
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 2);
	
	

	
 //infinite while loop
 tx_string("CMD? Enter r, o, g, or b, followed by 0, 1, or 2");
	while (1)
  {
		// Check off part one
		//rx_char_1();

		if(rx_flag){
			//process the data
			check_input();
		}
		
		// If we have both flags, we can reset
		if ((led_flag == 1) && (toggle_flag == 1)) {
			rx_flag = 0;
			led_flag = 0;
			toggle_flag = 0;
			tx_string("Please enter another command.");
			tx_string("CMD? Enter r, o, g, or b, followed by 0, 1, or 2");
		}
  }
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

// Initialize LEDs function
void init_LEDs(void) {	
	GPIOC->MODER = 0; //Clear to start
	//Initialize LEDs for toggling
	GPIOC->MODER |= (1 << 12); //Set RED pin general purpose output (PC6)
	GPIOC->MODER |= (1 << 14); //Set BLUE pin general purpose output (PC7)
	GPIOC->MODER |= (1 << 16); //Set ORANGE pin general purpose output (PC8)
	GPIOC->MODER |= (1 << 18); //Set GREEN pin general purpose output (PC9)
	GPIOC->ODR = 0;
	GPIOC->OTYPER = 0; //Set push pull
	GPIOC->OSPEEDR = 0; //Set low speed
}

//USART3 Handler Function
void USART3_4_IRQHandler(void)
{
	// Ignore input if previous information has not been processed yet
	if(rx_flag == 0) {
		rx = USART3->RDR;
		rx_flag = 1;
	}
}


void send_char(char c)
{
	while (!(USART3->ISR & USART_ISR_TXE))
	{
	}
	USART3->TDR = c;
}

void tx_string(char str[])
{
	for (int i = 0; str[i] != '\0'; i++)
		{
			send_char(str[i]);
		}
}	

void rx_char_1(void) {
	char c;
	
	if((USART3->ISR & USART_ISR_RXNE)) {
		c = USART3->RDR; // store rx read value
		
	 //Toggle LED based on input
		switch(c) {
			case 'r':
				GPIOC->ODR ^= GPIO_ODR_6;
				break;
			case 'b':
				GPIOC->ODR ^= GPIO_ODR_7;
				break;
			case 'o':
				GPIOC->ODR ^= GPIO_ODR_8;
				break;
			case 'g':
				GPIOC->ODR ^= GPIO_ODR_9;
				break;
			default:
				tx_string("Error, You didn't push r, o, g, or b. Push one of those instead ");
				break;
		}
	}
}

void check_input(void) {
	// Check first character input to verify LED
	if(led_flag == 0 ) {
		switch(rx) {
			case 'r':
			case 'b':
			case 'o':
			case 'g':
				led = rx;
				led_flag = 1;
				rx_flag = 0; // Ready to receive next character
				tx_string("LED selection received. Please enter 0, 1, or 2.");
				break;
			default:
				tx_string("Error: LED char not inputted, please enter r, b, g, or o.");
				rx_flag = 0;
				break;
		}
	}
	// LED char has been received, check number
	else {
		switch(rx) {
			case '0':
				turn_off_led();
				toggle_flag = 1;
				break;
			case '1':
				turn_on_led();
				toggle_flag = 1;
				break;
			case '2':
				toggle_led();
				toggle_flag = 1;
				break;
			default:
				tx_string("Error: Number for LED control not inputted: please enter 0, 1, or 2.");
				rx_flag = 0;
				led_flag = 0;
				break;
		}
	}
}

void turn_off_led(void) {
	switch(led) {
		case 'r':
			GPIOC->ODR &= ~(1<<6);
			tx_string("Red LED has been turned off.");
			break;
		case 'b':
			GPIOC->ODR &= ~(1<<7);;
			tx_string("Blue LED has been turned off.");
			break;
		case 'o':
			GPIOC->ODR &= ~(1<<8);
			tx_string("Orange LED has been turned off.");
		break;
		case 'g':
			GPIOC->ODR &= ~(1<<9);
			tx_string("Green LED has been turned off.");
			break;
		default:
			break;
	}
}

void turn_on_led(void) {
		switch(led) {
		case 'r':
			GPIOC->ODR |= (1<<6);
			tx_string("Red LED has been turned on.");
			break;
		case 'b':
			GPIOC->ODR |= (1<<7);
			tx_string("Blue LED has been turned on.");
			break;
		case 'o':
			GPIOC->ODR |= (1<<8);
			tx_string("Orange LED has been turned on.");
		break;
		case 'g':
			GPIOC->ODR |= (1<<9);
			tx_string("Green LED has been turned on.");
			break;
		default:
			break;
	}
}

void toggle_led(void) {
		switch(led) {
		case 'r':
			GPIOC->ODR ^= (1<<6);
			tx_string("Red LED has been turned Toggled.");
			break;
		case 'b':
			GPIOC->ODR ^= (1<<7);
			tx_string("Blue LED has been turned Toggled.");
			break;
		case 'o':
			GPIOC->ODR ^= (1<<8);
			tx_string("Orange LED has been turned Toggled.");
		break;
		case 'g':
			GPIOC->ODR ^= (1<<9);
			tx_string("Green LED has been turned Toggled.");
			break;
		default:
			break;
	}
}

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


