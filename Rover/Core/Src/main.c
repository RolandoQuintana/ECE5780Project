/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "OV2640_regs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_MAX_SIZE 4069
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

int tsfr_len;
int sendlen;
int have_rcvd;

uint8_t send_OK = 0;
uint8_t rcv_OK = 0;
uint8_t done_rcv = 0;

uint8_t	Buf1[BUFFER_MAX_SIZE]={0}, Buf2[BUFFER_MAX_SIZE]={0};

uint8_t	*picbuf = 0;

/* USER CODE BEGIN PV */

void Transmit_Character(char tx_char) {
	
	while(!(USART3->ISR & USART_ISR_TXE)){ //While the transmit data register is not empty, do nothing.
	}
	
	USART3->TDR = tx_char;
	
}
void Transmit_Chars(char* tx_str) {
	
	while(*tx_str != 0){ //While the transmit data register is empty, do nothing.
		Transmit_Character(*tx_str);
		tx_str++;
	}
}
void Transmit_String(char* tx_str) {
	
	while(*tx_str != 0){ //While the transmit data register is empty, do nothing.
		Transmit_Character(*tx_str);
		tx_str++;
	}
	Transmit_Character('\n');
	Transmit_Character('\r');
}

    /**
    * @brief This function handles DMA RX interrupt request. 
    * @param None
    * @retval None 
    */
    void
    USART3_DMA_RX_IRQHandler(
    void
    )
    {
    HAL_DMA_IRQHandler(huart3.hdmarx);
    }
    /**
    * @brief This function handles DMA TX interrupt request.
    * @param None
    * @retval None 
    */
    void
    USART3_DMA_TX_IRQHandler(
    void
    )
    {
    HAL_DMA_IRQHandler(huart3.hdmatx);
    }
    /**
    * @brief This function handles USARTx interrupt request.
    * @param None
    * @retval None
    */
    void
    USART3_IRQHandler(
    void
    )
    {
    HAL_UART_IRQHandler(&huart3);
    }
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void CAM_CS_Init(void);
uint8_t read_reg(uint8_t addr);
void write_reg(uint8_t addr, uint8_t data);
static void delay_10us(void);
static uint8_t wrSensorReg8_8(uint8_t regID, uint8_t regDat);
static uint8_t rdSensorReg8_8(uint8_t regID, uint8_t* regDat);
static void ArduCAM_Init(void);
static void SingleCapTransfer(void);
static void DMA1_SendtoUsart(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	CAM_CS_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Transmit_String("ACK CMD ArduCAM Start!");
		
		while(1)
		{
			write_reg(0x00, 0x55);
			uint8_t temp = read_reg(0x00);
			if (temp != 0x55)
			{
				Transmit_String("ACK CMD SPI interface Error!");
				Transmit_Character(temp);
				HAL_Delay(1000);
				continue;
			}
			else
			{
				Transmit_String("ACK CMD SPI interface OK!");
				break;
			}
		}
		while(1)
		{
			uint8_t vid, pid;
			wrSensorReg8_8(0xff, 0x01);
			rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
			rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
			if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 )))
				Transmit_String("ACK CMD Can't find OV2640 module!");
			else
			{
				Transmit_String("ACK CMD OV2640 detected.");   
				break;
			}
		}
		uint8_t start_capture = 1;
		ArduCAM_Init();
		while(1)
		{
			if(start_capture){
				start_capture = 0;
				Transmit_String("ACK CMD Beginning single capture.");
				SingleCapTransfer();
			}
			else if(rcv_OK){
				rcv_OK = 0;
				DMA1_SendtoUsart();
			}
			else if(send_OK){
				send_OK = 0;
				Transmit_String("ACK CMD Captured!");
				start_capture = 1;
			}
			//else {
				//HAL_Delay(1);
				//Transmit_String("Waiting.");  
			//}
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/**
	*
	*
	*
	*/

static void delay_10us(){
		for (volatile uint16_t i=0; i!=0x5; i++);
}

static void CAM_CS_Init(void)
{
	GPIO_InitTypeDef initStr = {GPIO_PIN_4,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_HIGH,
	GPIO_NOPULL};
	HAL_GPIO_Init(GPIOA, &initStr);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}


uint8_t bus_write(uint8_t address,uint8_t value)
{	
	uint8_t retval;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);//Write cs pin low
	delay_10us();
	HAL_SPI_TransmitReceive(&hspi2, &address, &retval, 1, 100);
	HAL_SPI_TransmitReceive(&hspi2, &value, &retval, 1, 100);
	delay_10us();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);//Write cs pin high
	return 1;
}

uint8_t bus_read(uint8_t address)
{
  uint8_t value;
	uint8_t dummy_data = 0x00;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);//Write cs pin low
	delay_10us();
	HAL_SPI_TransmitReceive(&hspi2, &address, &value, 1, 100);
	HAL_SPI_TransmitReceive(&hspi2, &dummy_data, &value, 1, 100);
	delay_10us();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);//Write cs pin high
	return value;
}


uint8_t read_reg(uint8_t addr)
{
	uint8_t data;
	data = bus_read(addr & 0x7F);
	return data;
}
void write_reg(uint8_t addr, uint8_t data)
{
	 bus_write(addr | 0x80, data); 
}

uint8_t wrSensorReg8_8(uint8_t regID, uint8_t regDat)
{
	uint8_t id_dat[2] = {regID, regDat};
	HAL_I2C_Master_Transmit(&hi2c1, OVADDR, id_dat, 2, 100);                                   
	return 0;
}

uint8_t rdSensorReg8_8(uint8_t regID, uint8_t* regDat)
{
	HAL_I2C_Master_Transmit(&hi2c1, OVADDR, &regID, 2, 100);  
	HAL_I2C_Master_Receive(&hi2c1, OVADDR|0x01, regDat, 1, 100);                                    
	return 0;                
}

//I2C Array Write 8bit address, 8bit data
int wrSensorRegs8_8(const struct sensor_reg reglist[])
{
  int err = 0;
  uint16_t reg_addr = 0;
  uint16_t reg_val = 0;
  const struct sensor_reg *next = reglist;
  while ((reg_addr != 0xff) | (reg_val != 0xff))
  {
    reg_addr = next->reg;
    reg_val = next->val;
    err = wrSensorReg8_8(reg_addr, reg_val);
    next++;
  }
	return err;
}

void ArduCAM_Init()
{
		wrSensorReg8_8(0xff, 0x01);
		wrSensorReg8_8(0x12, 0x80);
		if(1)//Use JPEG formatting
		{
			wrSensorRegs8_8(OV2640_JPEG_INIT);
			wrSensorRegs8_8(OV2640_YUV422);
			wrSensorRegs8_8(OV2640_JPEG);
			wrSensorReg8_8(0xff, 0x01);
			wrSensorReg8_8(0x15, 0x00);
			wrSensorRegs8_8(OV2640_160x120_JPEG); //OV2640_160x120_JPEG? Resolutions described in OV2640_regs.h
		}
		else//Use QVGA formatting
		{
			wrSensorRegs8_8(OV2640_QVGA);
		}
	}			
			
	//Get corresponding bit status
uint8_t get_bit(uint8_t addr, uint8_t bit)
{
  uint8_t temp;
  temp = read_reg(addr);
  temp = temp & bit;
  return temp;
}

uint32_t read_fifo_length(void)
{
	uint32_t len = 0;
	uint32_t len1,len2,len3;
	len1 = read_reg(FIFO_SIZE1);
	len2 = read_reg(FIFO_SIZE2);
	len3 = read_reg(FIFO_SIZE3) & 0x7f;
	len = ((len3 << 16) | (len2 << 8) | len1) & 0x007fffff;
	return len;	
}

void DMA1_RX(uint8_t *p )
{		
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);//Write cs pin low
	uint8_t value = BURST_FIFO_READ;
	uint8_t dummy_data = 0x00;
	HAL_SPI_TransmitReceive(&hspi2, &value, &dummy_data, 1, 100);//set_fifo_burst();
	
	HAL_SPI_Receive_DMA(&hspi2, p, sendlen);
}

void DMA1_SendtoUsart(void)
{		
	uint8_t	*sdbuf;
	have_rcvd += sendlen;
  if(have_rcvd < tsfr_len)
	{	
		if(picbuf == Buf1)
		{		
			sdbuf = Buf1;	  picbuf = Buf2;	
		}
		else
		{
			sdbuf = Buf2;	  picbuf = Buf1;
		}
		HAL_UART_Transmit_DMA(&huart3, sdbuf, sendlen);
		int have_left	= tsfr_len - have_rcvd;		
		sendlen	= (have_left>=BUFFER_MAX_SIZE) ? BUFFER_MAX_SIZE : have_left;	
		DMA1_RX(picbuf);	
	}
	else
	{
		done_rcv = 1;
		HAL_UART_Transmit_DMA(&huart3, picbuf, sendlen);
	}			
}

void SingleCapTransfer(void)
{
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK); //Flush FIFO
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK); //Clear FIFO Flag
	write_reg(ARDUCHIP_FIFO, FIFO_START_MASK); //Start capture
	while(!get_bit(ARDUCHIP_TRIG , CAP_DONE_MASK)){;}
		
	Transmit_String("ACK CMD capture done!");
	tsfr_len = read_fifo_length();
	have_rcvd = 0;
	//Transmit_Chars("ACK CMD the length is ");
	//Transmit_Character((uint8_t )tsfr_len + '0');
	//Transmit_Character((uint8_t )(tsfr_len >> 8) + '0' );
	//Transmit_Character((uint8_t )(tsfr_len >> 16) + '0' );
	//Transmit_Character((uint8_t )(tsfr_len >> 24) + '0' );
	//Transmit_String(".");
		
	sendlen = (tsfr_len>=BUFFER_MAX_SIZE) ? BUFFER_MAX_SIZE : tsfr_len;
	picbuf = Buf1;
	DMA1_RX(Buf1);
}

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
