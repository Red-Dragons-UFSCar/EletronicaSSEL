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

// Struct que será compartilhada entre cores
struct shared_data
{
	uint8_t sts_4to7; // status: 0 = Sem dados, 1 = Com dados, 2 = Em uso (CM4-CM7)
	uint8_t sts_7to4; // status: 0 = Sem dados, 1 = Com dados, 2 = Em uso (CM7-CM4)
	int M4toM7[9]; // 9 inteiros (36 bytes) do núcleo CM4 para o núcleo CM7
	int M7toM4[12]; // 12 inteiros (48 bytes) do núcleo CM4 para o núcleo CM7
};

//Declaração da struct por meio de um ponteiro em um ponto de memória comum entre os cores
volatile struct shared_data * const xfr_ptr = (struct shared_data *)0x38001000;

/*
 * Função para obter dados do core M7
 * Parâmetros:
 * Ponteiro para a variável a ser modificado
 */
void get_M7(int *data)
{
	if (xfr_ptr->sts_7to4 == 1) // if M4 to M7 buffer has data
	{
		xfr_ptr->sts_7to4 = 2; // lock the M4 to M7 buffer
		for(uint8_t n = 0; n <12; n++)
		{
			data[n] = xfr_ptr->M7toM4[n]; // transfer data
			xfr_ptr->M7toM4[n] = 0; // clear M4 to M7 buffer
		}
		xfr_ptr->sts_7to4 = 0; // M4 to M7 buffer is empty
	}
}

uint8_t TxAdress0[] = {1,2,3,4,5}; //Endereço de envio
int TxData[6]={111,0,0,0,0,112}; //Vetor enviado
uint8_t RxData[1]; //Mensagem retornada (Descontinuada)
int vet_senhas[6]= {111,112,113,114,115,116}; //Vetor de senhas dos robos: (R1S1,R1S2,R2S1,R2S2,R3S1,R3S2)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
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
	NRF_WriteRegister(NRF_REG_RX_ADDR_P0, Adress, 5);
}

/*
 * Função para mudar o canal de comunicação
 * Parâmetros: Robô alvo
 */
void changeChannel(uint8_t n){
	NRF_EnterMode(NRF_MODE_STANDBY1);
	if(n==0){
		NRF_WriteRegisterByte(NRF_REG_RF_CH,0x02); //Canal 3
	}
	if(n==1){
		NRF_WriteRegisterByte(NRF_REG_RF_CH,0x03); //Canal 4
	}
	if(n==2){
		NRF_WriteRegisterByte(NRF_REG_RF_CH,0x04); //Canal 5
	}
	NRF_EnterMode(NRF_MODE_TX);
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
  Tx_mode(TxAdress0); //Inicialização do NRF em modo de transmissão

  NRF_Status ret = NRF_OK;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //Inicialização dos ponteiros entre núcleos
  xfr_ptr->sts_4to7 = 0;
  xfr_ptr->sts_7to4 = 0;

  int Valores[12] = {0,0,0,0,0,0,0,0,0,0,0,0}; //Vetor obtido do core M7
  int Returns[9]={0,0,0,0,0,0,0,0,0}; //Retornos ao serial
  uint32_t acumulador[3] = {0,0,0};

  while (1)
  {
	  //Obtem os valores do core M7
	 if(xfr_ptr->sts_7to4 == 1){
		get_M7(Valores);
	  }
	 //Loop entre Robos
	 for(uint8_t i=0; i<3;i++){
		 changeChannel(i); //Troca o canal para o  do robo especifico
		 //Salva a variável para envio em seu respectivo vetor
		 for(uint8_t n=0; n<4;n++){
			 TxData[n+1] = Valores[n+4*i];
		 }
		 TxData[0] = vet_senhas[i*2]; //Senha do robô especifico
		 TxData[5] = vet_senhas[1+i*2];//Senha do robô especifico

		 uint32_t Start = HAL_GetTick(); //Tempo de início de transmissão
		 ret = NRF_TransmitAndWait(TxData, sizeof(TxData)); //Transmissão da mensagem
		 uint32_t End = HAL_GetTick(); //Tempo de fim de transmissão
		 acumulador[i]+= End - Start; //Acumulador de tempo de latência
		 uint8_t ploss = NRF_ReadPacketLoss();//Leitura da perda de pacotes
		 Returns[i+3] = acumulador[i];//Salvamento de tempo acumulado no vetor de retorno ao python
		 if(ret == NRF_OK){
			 //Pino de confirmação
			 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			 int retorno = 0;//Gambiarra - Retirar
			 Returns[i] = retorno;//Gambiarra - Retirar
			 Returns[i+6] = ploss;
			 acumulador[i] = 0;
		 } else if(ret == NRF_MAX_RT) {//Numero máximo de retransmissões
			 HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);

	 } else {//Sucesso - Necessario deixar função de transmissão mais robusta( Timeout e retorno de erro)
		 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	 }
	 }
	 //Retorno de dados ao core M7
	 if(xfr_ptr->sts_4to7 == 0){
	 		 for(uint8_t n = 0 ;n<9;n++){
	 			 xfr_ptr->M4toM7[n] = Returns[n];
	 		 }
	 		xfr_ptr->M4toM7[1] = Valores[0];
	 	 	 xfr_ptr->sts_4to7 = 1;
	 	 }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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
