/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
 #include "stdio.h"
 #include "stm32g4xx_ll_exti.h"
 #include "usart.h"
 #include "commissioning.h"
#if defined(F1_PRODUCT_MODBUS)
 #include "product_modbus.h"
#endif
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t RecData;

int DataBits[8];
int ConType=0;
int MotorLevel=0;
int TIM2HeatLevel=0;
int TIM3HeatLevel=0;
int TotolSwitch=0;
int HeatBarSwitch=0;
volatile uint32_t HeatZeroCount, HeatTimerStartCount, HeatTimerExpireCount;
/* PB14 is not connected in the F1_POWER_V004 Altium design. This legacy
 * software channel must remain disabled unless a physical board revision and
 * continuity test prove a second heater path. See docs/HARDWARE_FACTS.md. */
int HeatRollSwitch=0;

int MotorDir=0;//�������λ

extern int HeatPowerTable[30][2];

extern int LowPowerTable[13][2];
extern int HighPowerTable[13][2];


extern volatile uint8_t TIM6SendSpeedFlag; 


//485通信调试变量
volatile uint32_t Debug485TxStatus = 0xFF;
volatile uint32_t Debug485TxCount = 0;


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  if((TIM2->SR & TIM_SR_UIF) != 0U && (TIM2->DIER & TIM_DIER_UIE) != 0U)
    HeatTimerExpireCount++;
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	if(HeatBarSwitch==1&&TotolSwitch!=0)
  {
      if(ConType==2)
      {
				if(TIM2HeatLevel==0)
				{
					HAL_GPIO_WritePin(HeatBar_GPIO_Port,HeatBar_Pin,GPIO_PIN_RESET);
					HeatBarSwitch=0;
				}
      }
      if(HeatBarSwitch==1)
      {
        HAL_GPIO_WritePin(HeatBar_GPIO_Port,HeatBar_Pin,GPIO_PIN_SET);	
        HAL_TIM_Base_Stop(&htim2);
        __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
        TIM2->CNT = 0; 
      }
  }
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	/* Legacy HeatRoll timing path: it targets unconnected PB14 on F1_POWER_V004. */
	if(HeatRollSwitch==1&&TotolSwitch!=0)
  {
      if(ConType==4)
      {
				if(TIM3HeatLevel==0)
				{
					HAL_GPIO_WritePin(HeatRoll_GPIO_Port,HeatRoll_Pin,GPIO_PIN_RESET);
          HeatRollSwitch=0;
				}
        
      }
      if(HeatRollSwitch==1)
      {
        HAL_GPIO_WritePin(HeatRoll_GPIO_Port,HeatRoll_Pin,GPIO_PIN_SET);	
        HAL_TIM_Base_Stop(&htim3);
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
        TIM3->CNT = 0; 
      }
		}
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
#if !defined(F1_PRODUCT_MODBUS)
  Commissioning_RxByteISR(RecData);
  HAL_UART_Receive_IT(&huart2, &RecData, sizeof(RecData));
#endif
  /* USER CODE END USART2_IRQn 1 */
}

void DMA1_Channel1_IRQHandler(void)
{
#if defined(F1_PRODUCT_MODBUS)
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
#endif
}

/**
  * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */

  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
	if(LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_12))
  {
    LL_EXTI_ClearFlag_0_31 (LL_EXTI_LINE_12);
    HeatZeroCount++;
    /* Level 30 bypasses phase delay and stays high across zero crossings. */
#if !defined(F1_PRODUCT_MODBUS) || defined(F1_TEST_BOARD)
    HAL_GPIO_WritePin(HeatRoll_GPIO_Port, HeatRoll_Pin,
      TotolSwitch != 0 && HeatRollSwitch != 0 && TIM3HeatLevel == 30
      ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(HeatRoll_GPIO_Port, HeatRoll_Pin, GPIO_PIN_RESET);
#endif
    HAL_GPIO_WritePin(HeatBar_GPIO_Port, HeatBar_Pin,
      TotolSwitch != 0 && HeatBarSwitch != 0 && TIM2HeatLevel == 30
      ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Stop_IT(&htim3);
		__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
		__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
		HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
		HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
		TIM2->CNT = 0;
		TIM3->CNT = 0;

		
		/* Product Modbus controls the heater channels independently; the legacy
		 * appliance loop selected one function through ConType. Load the phase
		 * delay whenever the corresponding product command is active. */
#if defined(F1_PRODUCT_MODBUS)
		if((TotolSwitch != 0) && (HeatBarSwitch != 0) &&
		   (TIM2HeatLevel >= 1) && (TIM2HeatLevel <= 30))
		{
			TIM2->ARR=HeatPowerTable[TIM2HeatLevel-1][0];
			TIM2->PSC=HeatPowerTable[TIM2HeatLevel-1][1];
		}
#else
		//�������
		if((ConType==2)&&(TIM2HeatLevel!=0))
		{
			if((TIM2HeatLevel>=1)&&(TIM2HeatLevel<=30))
			{
				TIM2->ARR=HeatPowerTable[TIM2HeatLevel-1][0];
				TIM2->PSC=HeatPowerTable[TIM2HeatLevel-1][1];
			}
		}
#endif
		
#if defined(F1_PRODUCT_MODBUS) && defined(F1_TEST_BOARD)
		if((TotolSwitch != 0) && (HeatRollSwitch != 0) &&
		   (TIM3HeatLevel >= 1) && (TIM3HeatLevel <= 30))
		{
			TIM3->ARR=HeatPowerTable[TIM3HeatLevel-1][0];
			TIM3->PSC=HeatPowerTable[TIM3HeatLevel-1][1];
		}
#elif !defined(F1_PRODUCT_MODBUS)
		//�������
		if((ConType==4)&&(TIM3HeatLevel!=0))
		{
			if((TIM3HeatLevel>=1)&&(TIM3HeatLevel<=30))
			{
				TIM3->ARR=HeatPowerTable[TIM3HeatLevel-1][0];
				TIM3->PSC=HeatPowerTable[TIM3HeatLevel-1][1];
			}
		}
#endif
		
		

		
    /* PSC is buffered. Load it with UG while interrupts are disabled, then
     * clear UG's update flag before arming the actual delay. */
    TIM2->EGR = TIM_EGR_UG;
    TIM3->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    TIM2->CNT = 0U;
    TIM3->CNT = 0U;
		if((TotolSwitch != 0) && (HeatBarSwitch != 0) &&
		   (TIM2HeatLevel>=1)&&(TIM2HeatLevel<=13)&&(HAL_GPIO_ReadPin(Zero_GPIO_Port,Zero_Pin)==0))
		{
			if(HAL_TIM_Base_Start_IT(&htim2) == HAL_OK) HeatTimerStartCount++;
		}
		if((TotolSwitch != 0) && (HeatBarSwitch != 0) &&
		   (TIM2HeatLevel>=14)&&(TIM2HeatLevel<=29)&&(HAL_GPIO_ReadPin(Zero_GPIO_Port,Zero_Pin)==1))
		{
			if(HAL_TIM_Base_Start_IT(&htim2) == HAL_OK) HeatTimerStartCount++;
		}
		
		
#if !defined(F1_PRODUCT_MODBUS) || defined(F1_TEST_BOARD)
		if((TotolSwitch != 0) && (HeatRollSwitch != 0) &&
		   (TIM3HeatLevel>=1)&&(TIM3HeatLevel<=13)&&(HAL_GPIO_ReadPin(Zero_GPIO_Port,Zero_Pin)==0))
		{
			HAL_TIM_Base_Start_IT(&htim3);
		}
		if((TotolSwitch != 0) && (HeatRollSwitch != 0) &&
		   (TIM3HeatLevel>=14)&&(TIM3HeatLevel<=29)&&(HAL_GPIO_ReadPin(Zero_GPIO_Port,Zero_Pin)==1))
		{
			HAL_TIM_Base_Start_IT(&htim3);
		}
#endif

	}
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        TIM6SendSpeedFlag = 1;
    }
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
