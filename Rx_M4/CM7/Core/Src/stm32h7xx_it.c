/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32h7xx.h"
#include "dshot.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//Controlador

//Referência de velocidade (Vindo da main)
extern float ref[4];
//Constantes de Controle
const float Kc[4] ={150,150,150,150};
const float Ki[4] = {400,400,400,400} ;
const float Kd[4] = {0,0,0,0};
// Erro
volatile float error[4] = {0,0,0,0};
//Varição da ação de controle
//volatile float deltaU[4] = {0,0,0,0};
//Ação de Controle
//volatile float uM[4] = {0,0,0,0};
//Velocidades e erros anteriores
//volatile float prevspeed[4]={0,0,0,0};
//volatile float prevspeed2[4] = {0,0,0,0};
//volatile float preverror[4]= {0,0,0,0};
//Contador para salvar as variaveis
uint8_t cont = 0;
//Valores de DSHOT enviado para os motores
uint16_t D[4]= {0,0,0,0};
//Variavejs de valor de encoder e calculo de velocidade
uint32_t Enc[4] = {0,0,0,0};
volatile int vel[4] = {0,0,0,0};
volatile float speed[4] = {0,0,0,0};

//Novo controlador
volatile float u[4] = {0,0,0,0};
volatile float u_k1[4] = {0,0,0,0};
volatile float preverror[4]= {0,0,0,0};
volatile float preverror2[4] = {0,0,0,0};
float q0[4] = {0,0,0,0};
float q1[4] = {0,0,0,0};
float q2[4] = {0,0,0,0};
uint8_t once =0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Função de mapeamento
uint16_t map(float x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Controle(){
	for(uint8_t n=0;n<4;n++){
		if(once ==0){
			q0[n] = Kc[n] + Kd[n]/0.01 +ki[n]*0.01;
			q1[n] = -Kc[n] - 2*Kd[n]/0.01;
			q2[n] = Kd[n]/0.01;
		}
		//Calculo de erro
		error[n] =ref[n] -  speed[n];
		//Variação da ação de controle para esta iteração
		//deltaU[n] = Kc*(error[n]- preverror[n]) + error[n]*Ki -Kd*(speed[n]-2*prevspeed[n] + prevspeed2[n]);
		u[n] = u_k1[n] + q0[n]*error[n] +q1[n]*preverror[n] +q2[n]*preverror2[n];
		//Ação de controle
		//uM[n] = uM[n] + deltaU[n];

		//Saturado para evitar que a ação de controle ultrapasse o limite
		if( u[n] < -1023){
			u[n]= -1023;
		}
		if(u[n]>1023){
			u[n]= 1023;
		}
		//Mapeamento da variavel de ação de controle no alcançe dado
		if(ref[n]==0){
			D[n]=0;
			u[n] = 0;
		}else if(u[n]>=0 ){
			D[n] = map(u[n],0,1023,0,1023);

		}else if(u[n]<0){
			D[n]= map(u[n],-1023,0,2047,1024);
		}
		u_k1[n] = u[n];

	//Logica para salvar o erro e a velocidade anterior
	cont = cont +1;
	if(cont == 1){
		for(uint8_t n=0;n<4;n++){
			prevspeed[n] = speed[n];
			preverror[n] = error[n];
		}
	} else if(cont ==2){
		for(uint8_t n=0;n<4;n++){
			prevspeed2[n] = prevspeed[n];
			prevspeed[n] = speed[n];
			preverror[n] = error[n];
		}
		cont = 1;
	}
}
	once=1;
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_tim2_ch1;
extern DMA_HandleTypeDef hdma_tim2_ch3;
extern DMA_HandleTypeDef hdma_tim5_ch4;
extern DMA_HandleTypeDef hdma_tim5_ch2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim15;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern float velocidade[4];
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim2_ch3);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim5_ch4);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim5_ch2);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim2_ch1);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  BSP_PB_IRQHandler(BUTTON_USER);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief This function handles TIM15 global interrupt.
  */
void TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM15_IRQn 0 */

  /* USER CODE END TIM15_IRQn 0 */
  HAL_TIM_IRQHandler(&htim15);
  /* USER CODE BEGIN TIM15_IRQn 1 */
  //
  //Enc[0] = TIM4->CNT;
  Enc[0] = TIM1->CNT;
  //Enc[1] = TIM1->CNT;
  Enc[2] = TIM3->CNT;
  Enc[3] = TIM4->CNT;
  Enc[1] = TIM8->CNT;

  TIM4->CNT = 0;
  TIM1->CNT = 0;
  TIM8->CNT = 0;
  TIM3->CNT = 0;
  static float last_vel[4] ={0,0,0,0};

  for(uint8_t i=0;i<4;i++){
	  vel[i] = Enc[i];
	  if(vel[i]>60000){
			  vel[i] = vel[i] - 65355;
	  }
	  speed[i] = -1*vel[i]/(163.84);
	  if(speed[i]>10 || speed[i]<-10){
		  speed[i] = last_vel[i];
	  } else{
		  last_vel[i] = speed[i];
	  }
  }


  //velocidade = speed[0];
  Controle();

  dshot_write(D);

  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
  /* USER CODE END TIM15_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
