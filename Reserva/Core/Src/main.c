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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dshot.h"
#include "string.h"
#include "nrf24l01.h"
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
int RxData[6] = {0,0,0,0,0,0};
int Ack_data = 1;
int vet_senhas[6]= {111,112,113,114,115,116};
int senhas[2] = {0,0};
int old_mensagem[6]={0,0,0,0,0,0};
uint32_t contador =0;
float velocidade[4]={0,0,0,0};
float ref[4] = {0,0,0,0};
uint16_t motores[4] = {0,0,0,0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Tx_mode(uint8_t Adress[5]){

	if(NRF_Init(&hspi1, GPIOG, GPIO_PIN_12, GPIOG, GPIO_PIN_14) != NRF_OK){
		Error_Handler();
	}

	NRF_Reset();
	NRF_WriteRegister(NRF_REG_TX_ADDR,Adress,5);
	//Para enviar a mensagem usar função transmitandwait
}

void Rx_mode(uint8_t Adress[5]){

	if(NRF_Init(&hspi1, GPIOG, GPIO_PIN_12, GPIOG, GPIO_PIN_14) != NRF_OK){
		Error_Handler();
	}

	NRF_Reset();

	NRF_WriteRegister(NRF_REG_RX_ADDR_P0,Adress,5);

	NRF_WriteRegisterByte(NRF_REG_RX_PW_P0,sizeof(RxData));

	NRF_EnterMode(NRF_MODE_RX);
}

NRF_Status ReceiveData (uint8_t *data, uint32_t len){
	NRF_Status ret = NRF_ERROR;
	uint8_t status = NRF_ReadStatus();
	uint8_t STATUS_REGISTER_RX_DR_BIT = 6;
	if(status & (1<<STATUS_REGISTER_RX_DR_BIT)){
		NRF_ReadPayload(data,len);
		ret = NRF_OK;
		NRF_SetRegisterBit(NRF_REG_STATUS, 6);
	} else {
		ret = NRF_ERROR;
	}
	return ret;
}
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
  //HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim8, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
  if (HAL_TIM_Base_Start_IT(&htim15) != HAL_OK)
    	    {
    	      /* Starting Error */
    	      Error_Handler();
    	    }
    	  dshot_init(DSHOT300);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  GPIO_PinState PinState[2];

      //definicao do robo por meio da entrada de tensao no pinc10 e Pinc11//
      PinState[0]= HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10);
      PinState[1]= HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11);

      uint8_t TxAdress0[] = {1,2,3,4,5};
      Rx_mode(TxAdress0);
      if((PinState[0]==0)&&(PinState[1]==0)){
    	  NRF_WriteRegisterByte(NRF_REG_RF_CH,0x07); // Canal 3
    	  for(uint8_t k=0;k<2;k++){
    		  senhas[k] = vet_senhas[k];
    	  }
      }
      if(PinState[0]==1){
    	  NRF_WriteRegisterByte(NRF_REG_RF_CH,0x03); // Canal 4
    	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0);
    	  for(uint8_t k=0;k<2;k++){
    		  senhas[k] = vet_senhas[k+2];
    	  	  }
      }
      if(PinState[1]==1){
    	  NRF_WriteRegisterByte(NRF_REG_RF_CH,0x05); //Canal 5
    	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);
    	  for(uint8_t k=0;k<2;k++){
    		  	  senhas[k] = vet_senhas[k+4];
    	  	  }
      }
      NRF_Status ret = NRF_OK;
      char message[100]={'\0'};
          //Inicializa referencia como zero
	  HAL_Delay(5000);

	  uint32_t Leitura= 0;
	  float Leitura2 = 0;
	  extern volatile float vel[4];
	  //extern volatile float u[4];
	  //extern volatile float error[4];
	  uint32_t contador = 0;
	  extern volatile uint32_t count;
  while (1)
  {
	 ret = ReceiveData(RxData, sizeof(RxData));
	 if(ret == NRF_OK){
		 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	 }
	 if((RxData[0]==senhas[0])&&(RxData[5]==senhas[1])){
	  		for(uint8_t n=0;n<6;n++){
	  		  old_mensagem[n] = RxData[n];
	  		}
	  	  }
	 for(uint8_t n=0; n<4;n++){
		 ref[n] = (float)old_mensagem[n+1]/100;
		 //ref[n] = 3;
	   	  }
	 //print para o puttyW
	  sprintf(message, "%f %f %f %f\n \r",vel[0],vel[1],vel[2],vel[3]);
	  CDC_Transmit_FS(message,sizeof(message));
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_Delay(5);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1|RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.PLL2.PLL2M = 8;
  PeriphClkInitStruct.PLL2.PLL2N = 150;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
