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
#include "gpio.h"

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

/* USER CODE BEGIN PV */

struct shared_data
{
	uint8_t sts_4to7; // status: 0 = empty, 1 = has data, 2 = locked (CM4-CM7)
	uint8_t sts_7to4; // status: 0 = empty, 1 = has data, 2 = locked (CM7-CM4)
	uint8_t M4toM7[32]; // 256 bytes from CM4 to CM7
	uint8_t M7toM4[32]; // 256 bytes from CM7 to CM4
};

// pointer to shared_data struct (inter-core buffers and status)
volatile struct shared_data * const xfr_ptr = (struct shared_data *)0x38001000;

uint8_t TxAdress[] = {1,2,3,4,5};
uint8_t TxData[]="Pao De Batata";
uint8_t RxAdress[] = {0xEE,0xDD,0xCC,0xBB,0xAA};
uint8_t RxData[32];
uint8_t ReadMemManco = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
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
  MX_DMA_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  Rx_mode(TxAdress);
  int count = 0;
  int num = 1000;
  NRF_Status ret = NRF_OK;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 ret = ReceiveData(RxData);
	 if(ret == NRF_OK){
		 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
		 count = count+1;
	 } else {
		 //HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
	 }

	 if(count >= num - (num*0.01)){
		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, 1);
	 }
	 if(xfr_ptr->sts_4to7 == 0){
		 for(int n = 0; n < 32; n++){
		 	xfr_ptr->M4toM7[n] = RxData[n];
		 	}
		 xfr_ptr->sts_4to7 =1;
	 }



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
