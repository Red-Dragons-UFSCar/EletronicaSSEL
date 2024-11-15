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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dshot.h"
#include "string.h"

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

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */
int new_mensagem[6]= {0,0,0,0,0,0}; //Mensagem que será confirmar da utilizada para referência
int old_mensagem[6]={0,0,0,0,0,0}; //Mensagem que foi recebida anteriormente
float ref[4] = {0,0,0,0}; //Referência que será utilizada para os motores

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


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * Função para obter dados do core M4
 * Parâmetros:
 * Ponteiro para a variável a ser modificado
 */
void  get_M4(int *data)
{
	if (xfr_ptr->sts_4to7 == 1) // Verifica se há dados a serem lidos
	{
		xfr_ptr->sts_4to7 = 2; // Determina que os dados estão em uso
		for(int n = 0; n < 6; n++)
		{
			data[n] = xfr_ptr->M4toM7[n]; // Salva os dados na variável
			xfr_ptr->M4toM7[n] = 0; // Limpa o dado em questão na struct
		}
		xfr_ptr->sts_4to7 = 0; // Libera o status para o salvemento de novos dados
	}
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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM15_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  //Inicialização da leituras dos encoders
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim8, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);


  //Inicialização dos status de dados
  xfr_ptr->sts_4to7 = 0;
  xfr_ptr->sts_7to4 = 0;

  //Inicialização da interrupção a cada 10 ms
  if (HAL_TIM_Base_Start_IT(&htim15) != HAL_OK)
	{
	  /* Starting Error */
	  Error_Handler(); //Codigo é direcionado ao Error Handles caso não seja possível a inicialização
	}
  dshot_init(DSHOT300);//Inicialiazação do protocolo dshot para comunicação com ESC
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //Vetor de caracteres para utilizar no serial (com objetivo de obtenção de dados dos motores)
  char message[100]={'\0'};

  //Variaveis de leitura de ADC para sensoriamento de corrente
  uint32_t Leitura= 0;
  float Leitura2 = 0;

  //Variaveis de leitura vindas do arquivo de interrupção
  extern volatile float vel[4];
  extern volatile float u[4];
  extern volatile float error[4];

  // Contador para utilização de tempo em testes sem comunicação
  uint32_t contador = 0;

  //Delay para devia inicialização dos motores
  HAL_Delay(7000);

  //Loop principal
  while (1)
  {

	  //comunicacao entre cores
	  if (xfr_ptr->sts_4to7 == 1)
	  {
		 get_M4(new_mensagem); // Obtem dados do core M4 na variável new_mensagem
	  }

	  //validacao da mensagem, utilizamos senhas especificas definidas no core M4 para cada um dos robos
	  if((new_mensagem[0]==xfr_ptr->Password[0])&&(new_mensagem[5]==xfr_ptr->Password[1]))
	  {
		//Loop para salvamento de dados confirmados por senha
		for(uint8_t n=0;n<6;n++)
		{
		  old_mensagem[n] = new_mensagem[n];
		}
	  }

	  for(uint8_t n=0; n<4;n++)
	  {
		 //Cálculo de referência
		 ref[n] = (float)old_mensagem[n+1]/100;

		 //Codigo comentado para testes sem comunicação !!
		 /*
	  	 if(contador<400)
	  	 {
	  	   ref[n] =0;
		 } else {
		   ref[0] = 4;
		   ref[1] = 4;
		   ref[2] = -4;
		   ref[3] = -4;
		  }
	  	 */
	  }
	  	  //Retorno de variáveis para o serial
	  	  sprintf(message, "%f %f %f %f\n \r",vel[0],vel[1],vel[2],vel[3]);
	  	  CDC_Transmit_FS(message,sizeof(message));

	  	  //Inicialização e leitura do ADC (Não completamente implementada)
	  	  HAL_ADC_Start(&hadc1);
	  	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  	  Leitura = HAL_ADC_GetValue(&hadc1);
	  	  Leitura2 = (Leitura*3.3)/65535;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//Contador de tempo utilizado (Opção alternativa: utilização da HAL_GetTick();
	contador++;
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
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
