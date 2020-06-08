
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;



void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);


#define HTU21_Addr 0x40
#define Temp_reg 0xE3
#define Humi_reg 0xE5


float HTU21_Get_Temp(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData);
float HTU21_Get_Humi(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData);

uint8_t Temp_receive[3];
uint8_t Humi_receive[3];
float Temp, Humi;
uint8_t send_data[5];
uint8_t receive_data[4];
uint8_t temp_reg[1];



int main(void)
{


  HAL_Init();

 
  SystemClock_Config();


  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
	send_data[0]=0x23;
	temp_reg[0]= 0xE3;
	
	
	HAL_UART_Receive_DMA(&huart3, (uint8_t *)receive_data,3);
	
  
	while (1)
  {
		//Get Temp & Humi
			Temp = HTU21_Get_Temp(&hi2c1, HTU21_Addr, Temp_receive);
			HAL_Delay(50);
			Humi = HTU21_Get_Humi(&hi2c1, HTU21_Addr, Humi_receive);
			HAL_Delay(50);
			
		
				
		

		//Send data through UART 3
			send_data[1]= Temp_receive[0];
			send_data[2] = Temp_receive[1];
			send_data[3]= Humi_receive[0];
			send_data[4] = Humi_receive[1];
			HAL_UART_Transmit(&huart3, (uint8_t *)send_data, 5, 10);
			HAL_Delay(1000);
		
	
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
			uint8_t i,j;
			if(huart->Instance == huart3.Instance){
				if(receive_data[0]=='A'){
					j = receive_data[1] -48;
					i = receive_data[2] -48;
					switch (j)
					{
						case 1:
							if (i==0) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
							else HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
							break;
						case 2:
							if (i==0) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
							else HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
							break;
						case 3: 
							if (i==0) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
							else HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
							break;
						case 4:
							if (i==0) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
							else HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
							break;
					}	
				
			}

		
		}
}





float HTU21_Get_Temp(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData){
		uint8_t temp_reg[1];
		float t;
		temp_reg[0]=Temp_reg;
		HAL_I2C_Master_Transmit(hi2c,  DevAddress<<1, temp_reg, 1, 10);
		HAL_Delay(50);
		HAL_I2C_Master_Receive(hi2c, DevAddress<<1, pData,3,10);
		t = (Temp_receive[1] & 0xFC) | (uint16_t)Temp_receive[0]<<8;
		t*=175.72f;
		t/=65536.0f;
		t -=46.85f;
		return t;
}

float HTU21_Get_Humi(I2C_HandleTypeDef *hi2c,uint16_t DevAddress, uint8_t *pData ){
		uint8_t humi_reg[1];
		float h;
		humi_reg[0]=Humi_reg;
		HAL_I2C_Master_Transmit(hi2c, DevAddress<<1,humi_reg,1,10);
		HAL_Delay(50);
		HAL_I2C_Master_Receive(&hi2c1, DevAddress<<1,pData,3,10);
		h = (uint16_t)Humi_receive[0]<<8 | (Humi_receive[1] & 0xFC);
		h*=125.0f;
		h/=65536.0f;
		h-=6.0f;
		return h;
		
}


void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
