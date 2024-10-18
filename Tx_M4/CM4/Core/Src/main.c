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
#include "nrf24l01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
struct shared_data
{
	uint8_t sts_4to7; // status: 0 = empty, 1 = has data, 2 = locked (CM4-CM7)
	uint8_t sts_7to4; // status: 0 = empty, 1 = has data, 2 = locked (CM7-CM4)
	int M4toM7[9]; // 256 bytes from CM4 to CM7
	int M7toM4[12]; // 256 bytes from CM7 to CM4
};

// pointer to shared_data struct (inter-core buffers and status)
volatile struct shared_data * const xfr_ptr = (struct shared_data *)0x38001000;

int * get_M7() // get data from M4 to M7 buffer
{
	static int buffer[12]; // buffer to receive data
	if (xfr_ptr->sts_7to4 == 1) // if M4 to M7 buffer has data
	{
		xfr_ptr->sts_7to4 = 2; // lock the M4 to M7 buffer
		for(int n = 0; n < 12; n++)
		{
			buffer[n] = xfr_ptr->M4toM7[n]; // transfer data
			xfr_ptr->M7toM4[n] = 0; // clear M4 to M7 buffer
		}
		xfr_ptr->sts_7to4 = 0; // M4 to M7 buffer is empty
	}
	return buffer; // return the buffer (pointer)
}

uint8_t TxAdress0[] = {1,2,3,4,5};
uint8_t TxAdress1[] = {2,21,15,5,6};
uint8_t TxAdress2[] = {5,1,8,2,7};

int TxData[6]={111,0,0,0,0,112};
uint8_t RxData[1];
uint8_t ReadMemManco = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
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
	NRF_WriteRegister(NRF_REG_RX_ADDR_P0, Adress, 5);
}

void changeAddress(uint8_t n){
	NRF_EnterMode(NRF_MODE_STANDBY1);
	if(n==0){
		NRF_WriteRegister(NRF_REG_TX_ADDR,TxAdress0,5);
		NRF_WriteRegister(NRF_REG_RX_ADDR_P0, TxAdress0, 5);
	}
	if(n==1){
		NRF_WriteRegister(NRF_REG_TX_ADDR,TxAdress1,5);
		NRF_WriteRegister(NRF_REG_RX_ADDR_P0, TxAdress1, 5);
	}
	if(n==2){
		NRF_WriteRegister(NRF_REG_TX_ADDR,TxAdress2,5);
		NRF_WriteRegister(NRF_REG_RX_ADDR_P0, TxAdress2, 5);
	}
	NRF_EnterMode(NRF_MODE_TX);
}

void Rx_mode(uint8_t Adress[5]){

	if(NRF_Init(&hspi1, GPIOG, GPIO_PIN_12, GPIOG, GPIO_PIN_14) != NRF_OK){
		Error_Handler();
	}

	NRF_Reset();
	NRF_WriteRegister(NRF_REG_RX_ADDR_P0,Adress,5);

	NRF_WriteRegisterByte(NRF_REG_RX_PW_P0,32); //00111111 - 32 bytes

	NRF_EnterMode(NRF_MODE_RX);
}

NRF_Status ReceiveData (uint8_t *data){
	NRF_Status ret = NRF_ERROR;
	uint8_t status = NRF_ReadStatus();
	uint8_t STATUS_REGISTER_RX_DR_BIT = 6;
	if(status & (1<<STATUS_REGISTER_RX_DR_BIT)){
		NRF_ReadPayload(data, 32);
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

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  Tx_mode(TxAdress0);

  NRF_Status ret = NRF_OK;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  xfr_ptr->sts_4to7 = 0;
  xfr_ptr->sts_7to4 = 0;
  int *xfr_dataM4;
  int Valores[12];
  int Returns[9];
  uint32_t acumulador[3];
  while (1)
  {
	 if(xfr_ptr->sts_7to4 == 1){
		xfr_dataM4 = get_M7();
	  }
	  for(uint8_t i=0; i<12;i++){
		Valores[i] = xfr_dataM4[i];
	  }
	 for(uint8_t i=0; i<3;i++){
		 changeAddress(i);
		 for(uint8_t n=0; n<4;n++){
			 TxData[n+1] = Valores[n+i*4];
		 }
		 uint32_t Start = HAL_GetTick();
		 ret = NRF_TransmitAndWait(TxData, sizeof(TxData));
		 uint32_t End = HAL_GetTick();
		 acumulador[i]+= End - Start;
		 uint8_t ploss = NRF_ReadPacketLoss();
		 if(ret == NRF_OK){

			 //Pino de confirmação
			 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			 //Lê o ACK payload e printa no serial
			 NRF_ReadPayload(&Returns[i],4); //Verificar este linhas !!!!!!!!!!!!!!!!!!!!
			 Returns[i+3] = acumulador[i];
			 Returns[i+6] = ploss;
			 acumulador[i] = 0;
		 } else if(ret = NRF_MAX_RT) {
			 HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);

	 } else {
		 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	 }
	 if(xfr_ptr->sts_4to7 == 0){
		 for(uint8_t n = 0 ;n<9;n++){
			 xfr_ptr->M4toM7[n] = Returns[i];
		 }
	 	 xfr_ptr->sts_4to7 = 1;
	 }

	  HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}
}
/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, CS_Pin|CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_Pin CE_Pin */
  GPIO_InitStruct.Pin = CS_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
