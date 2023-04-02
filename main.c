/* Includes ------------------------------------------------------------------*/
#include "main.h"
void SystemClock_Config(void);
void USART3_4_IRQHandler(void);


/*
			FRONT
		1				2
		
		
		
		3				4
			BACK
*/

// this number ranges from -100 to 100 for each wheel. -100 is max backwards and 100 is max forwards
int8_t wheelSpeed[4] = {0,0,0,0};

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	int baudRate = 9600;
	
	// RCC
  SystemClock_Config();
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // For TIM3
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // For TIM2
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // for GPIOC
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // for USART3
	
	// Setup USART3
	USART3->BRR = HAL_RCC_GetHCLKFreq() / baudRate;
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
	
	// Setup PC4 = TX
	GPIO_InitTypeDef pc4 = {GPIO_PIN_4, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_LOW, GPIO_AF1_USART3};
	HAL_GPIO_Init(GPIOC, &pc4);
	
	// Setup PC5 = RX
	GPIO_InitTypeDef pc5 = {GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_LOW, GPIO_AF1_USART3};
	HAL_GPIO_Init(GPIOC, &pc5);
	
	// Red LED
	GPIO_InitTypeDef pc6 = {GPIO_PIN_6, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW};
	HAL_GPIO_Init(GPIOC, &pc6);
	
	// USART3 interrupt
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 3);
	
  while (1)
  {
		
  }
}


int currentWheel = 0;
void USART3_4_IRQHandler(void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	wheelSpeed[currentWheel] = USART3->RDR;
	currentWheel = currentWheel == 3 ? 0 : currentWheel + 1;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);	
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
