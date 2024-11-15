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
//aaa lula meu presidentee
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

// Struct que será compartilhada entre cores
struct shared_data
{
	uint8_t sts_4to7; // status: 0 = Sem dados, 1 = Com dados, 2 = Em uso (CM4-CM7)
	uint8_t sts_7to4; // status: 0 = Sem dados, 1 = Com dados, 2 = Em uso (CM7-CM4)
	int M4toM7[6]; // 6 inteiros (24 bytes) do núcleo CM4 para o núcleo CM7
	int M7toM4[6]; // 6 inteiros (24 bytes) do núcleo CM4 para o núcleo CM7
	int Password[2]; // Senha utilizada na comunição recuperada do CM4
};

//Declaração da struct por meio de um ponteiro em um ponto de memória comum entre os cores
volatile struct shared_data * const xfr_ptr = (struct shared_data *)0x38001000;

int RxData[6] = {0,0,0,0,0,0}; //Dados recebidos por comunicação
int Ack_data = 1; //Dado a ser retornado (não funcionou)
int vet_senhas[6]= {111,112,113,114,115,116}; //Vetor de senhas dos robos: (R1S1,R1S2,R2S1,R2S2,R3S1,R3S2)
int senhas[2] = {0,0}; //Senha do robô selecionado
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * Troca de modo do NRF24L01 para modo de transmissão
 * Parâmetros:
 * Endereço de transmissão
 */
void Tx_mode(uint8_t Adress[5]){

	if(NRF_Init(&hspi1, GPIOG, GPIO_PIN_12, GPIOG, GPIO_PIN_14) != NRF_OK){
		Error_Handler();
	}

	NRF_Reset();
	NRF_WriteRegister(NRF_REG_TX_ADDR,Adress,5);
	//Para enviar a mensagem usar função transmitandwait
}

/*
 * Troca de modo do NRF24L01 para modo de recepção
 * Parâmetros:
 * Endereço de recepção
 */
void Rx_mode(uint8_t Adress[5]){

	if(NRF_Init(&hspi1, GPIOG, GPIO_PIN_12, GPIOG, GPIO_PIN_14) != NRF_OK){
		Error_Handler();
	}

	NRF_Reset();

	NRF_WriteRegister(NRF_REG_RX_ADDR_P0,Adress,5);

	NRF_WriteRegisterByte(NRF_REG_RX_PW_P0,sizeof(RxData));

	NRF_EnterMode(NRF_MODE_RX);
}

/*
 * Função para precebimento de dados
 * Parâmetros:
 * Ponteiro para variavel on serão salvo os dados
 * Tamanho em bytes dos dados
 */
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




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /*
   * Definição de endereço de transmissão
   */
  uint8_t TxAdress0[] = {1,2,3,4,5};
   Rx_mode(TxAdress0);

  /*
   * Selecionador de robôs
   */
  GPIO_PinState PinState[2];
  //Leitura dos pinos PC10 e PC11
  PinState[0]= HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10);
  PinState[1]= HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11);


  if((PinState[0]==0)&&(PinState[1]==0)){
	  NRF_WriteRegisterByte(NRF_REG_RF_CH,0x07); // Canal 3
	  for(uint8_t k=0;k<2;k++){
		  xfr_ptr->Password[k] = vet_senhas[k];
	  }
  }
  if(PinState[0]==1){
	  NRF_WriteRegisterByte(NRF_REG_RF_CH,0x03); // Canal 4
	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0);
	  for(uint8_t k=0;k<2;k++){
		  xfr_ptr->Password[k] = vet_senhas[k+2];
	  	  }
  }
  if(PinState[1]==1){
	  NRF_WriteRegisterByte(NRF_REG_RF_CH,0x05); //Canal 5
	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);
	  for(uint8_t k=0;k<2;k++){
		  	  xfr_ptr->Password[k] = vet_senhas[k+4];
	  	  }
  }


  NRF_Status ret = NRF_OK;
  while (1)
  {
	 ret = ReceiveData(RxData, sizeof(RxData));//Recebimento de dados
	 //Lede de confirmação de envio
	 if(ret == NRF_OK){
		 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	 }
	 //Salvamento de dados para utilização em outro core
	 if(xfr_ptr->sts_4to7 == 0){
		 for(int n = 0; n < 6; n++){
		 	xfr_ptr->M4toM7[n] = RxData[n];
		 	}
		 xfr_ptr->sts_4to7 =1;
	 }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 9;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 3072;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
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
