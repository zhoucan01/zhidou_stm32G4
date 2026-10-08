/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "cordic.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include "stdint.h"
  #include "stdio.h"

  /* Bus-voltage sampling. */
#include "mc_config.h"
#include "bus_voltage_sensor.h"
#include "commissioning.h"
#if defined(F1_PRODUCT_MODBUS)
#include "product_modbus.h"
#endif
#if defined(F1_CN2_ASCII)
#include "cn2_ascii.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
	uint8_t RecData;
  extern int ConType;                                                                                                     
  extern int MotorLevel;																																																	
  extern int TotolSwitch;																																															    
  extern int IsRec;																																																				
  int SpeedTable[15]={1000,2000,3000,4000,5000,6000,7000,8000,9000,10000,11000,12000,13000,14000,18000};								  
	uint8_t Motor_flag=0;
  int Isrun=0;
	int RealSpeed=0;
	extern int MotorDir;                																																								    
	
	
	
	#define MOTOR_SPEED_ERROR_RPM  200U
	#define MOTOR_CHECK_TIME_MS    2000
	#define MOTOR_RAMP_TIME_MS     3000

	static uint16_t MotorLockedMaxRpm = 0;
	static uint16_t MotorSetRpm = 0;
	static uint8_t MotorLastDir = 0;
	static uint32_t MotorCheckTick = 0;
	
	
	volatile uint8_t TIM6SendSpeedFlag = 0;
	uint8_t SpeedTxBuffer[2];


	

	
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



/* Latest bus ADC sample converted to a 12-bit value (0..4095). */
volatile uint16_t Debug_Vbus_ADC = 0;

/* Filtered bus voltage in whole volts. */
volatile uint16_t Debug_Vbus_V = 0;

/* Debug-value update time. */
static uint32_t Debug_Vbus_Tick = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CORDIC_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_MotorControl_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
#if !defined(F1_CN2_ASCII)
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
#endif

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	/*
	 * Hardware-safe startup intent for F1_POWER_V004. PB13 is the documented
	 * HEAT channel. PB14/HeatRoll has no schematic wire or named PCB net; keeping
	 * it low avoids treating that firmware-only alias as a real heater output.
	 */
	HAL_GPIO_WritePin(Relay_GPIO_Port,Relay_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, HeatBar_Pin|HeatRoll_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Stop_IT(&htim3);
  HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim6);
#if defined(F1_PRODUCT_MODBUS)
  ProductModbus_Init();
#elif defined(F1_CN2_ASCII)
  Cn2Ascii_Init();
#else
  Commissioning_Init();
  HAL_UART_Receive_IT(&huart2,&RecData,sizeof(RecData));
#endif


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if defined(F1_PRODUCT_MODBUS)
    ProductModbus_Task();
#elif defined(F1_CN2_ASCII)
    Cn2Ascii_Task();
#else
    Commissioning_Task();
#endif
    continue; /* Legacy appliance policy below is retained only for baseline recovery. */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		


    /* Refresh debug values every 100 ms without changing the MCSDK ADC rate. */
if ((uint32_t)(HAL_GetTick() - Debug_Vbus_Tick) >= 100U)
{
    Debug_Vbus_Tick = HAL_GetTick();

    /* ADC data is 12-bit left-aligned; shift right four bits to obtain 0..4095. */
    Debug_Vbus_ADC = VBS_GetBusVoltage_d(&RealBusVoltageSensorParamsM1._Super) >> 4;

    /* Read the library-converted bus voltage in volts. */
    Debug_Vbus_V = VBS_GetAvBusVoltage_V(&RealBusVoltageSensorParamsM1._Super);
}



		

		
		State_t MotorState;
    uint16_t TargetRpm;
    uint16_t ActualRpm;
    int32_t ActualSpeed;
    int16_t SpeedCommand;

    MotorState = MC_GetSTMStateMotor1();

    /* Legacy motor control below is unreachable in the current series. */
   
		
		if ((TotolSwitch != 0) && (ConType == 6) && (MotorLevel >= 1) && (MotorLevel <= 15))
		{

				TargetRpm = SpeedTable[MotorLevel - 1];
			
				if ((MotorLockedMaxRpm != 0) && (TargetRpm > MotorLockedMaxRpm))
				{
						TargetRpm = MotorLockedMaxRpm;
				}
				
				SpeedCommand = (MotorDir == 0)? -(int16_t)(TargetRpm / 6):  (int16_t)(TargetRpm / 6);

				
				if (MotorState == FAULT_NOW)
				{
				}
				
				else if (MotorState == FAULT_OVER)
				{
						MC_AcknowledgeFaultMotor1();
						MotorSetRpm = 0;
				}
			
				else if (MotorState == IDLE)
				{
						MC_ProgramSpeedRampMotor1(SpeedCommand,MOTOR_RAMP_TIME_MS);

						if (MC_StartMotor1())
						{
								MotorSetRpm = TargetRpm;
								MotorLastDir = MotorDir;
								MotorCheckTick = 0;
						}
				}
				else if (MotorState == RUN)
				{
						
						if (MotorDir != MotorLastDir)
						{
								MC_StopMotor1();
								MotorSetRpm = 0;
								MotorCheckTick = 0;
						}
						
						else if (TargetRpm != MotorSetRpm)
						{
								MC_ProgramSpeedRampMotor1(SpeedCommand,MOTOR_RAMP_TIME_MS);

								MotorSetRpm = TargetRpm;
								MotorCheckTick = 0;
						}
						
						else if ((MotorLockedMaxRpm == 0) && MC_HasRampCompletedMotor1())
						{
								ActualSpeed = MC_GetMecSpeedAverageMotor1();
							
								if (ActualSpeed < 0)
								{
										ActualSpeed = -ActualSpeed;
								}

								ActualRpm = (uint16_t)(ActualSpeed * 6);

								if ((ActualRpm > 0) && (TargetRpm >ActualRpm + MOTOR_SPEED_ERROR_RPM))
								{
										if (MotorCheckTick == 0)
										{
												MotorCheckTick = HAL_GetTick();
										}
										else if ((HAL_GetTick() - MotorCheckTick) >= MOTOR_CHECK_TIME_MS)
										{
												MotorLockedMaxRpm = ActualRpm;
												MotorSetRpm = ActualRpm;
												SpeedCommand = (MotorDir == 0)? -(int16_t)(ActualRpm / 6):(int16_t)(ActualRpm / 6);
												MC_ProgramSpeedRampMotor1(SpeedCommand,MOTOR_RAMP_TIME_MS);
												MotorCheckTick = 0;
										}
								}
								else
								{
										MotorCheckTick = 0;
								}
						}
				}
			}
		
		if (((ConType == 6) && (MotorLevel == 0)))
    {
        MC_StopMotor1();
        MotorSetRpm = 0;
        MotorCheckTick = 0;
    }
		
		
		/* Legacy speed response path. */
//		if (TIM6SendSpeedFlag != 0)
//		{
//			int ActualRpm;        

//			TIM6SendSpeedFlag = 0;

//			
//			ActualRpm = (int16_t)MC_GetMecSpeedAverageMotor1();
//			ActualRpm *=6; 
//			SpeedTxBuffer[1] = (ActualRpm >> 8) & 0xFF;  // ???
//			SpeedTxBuffer[0] = ActualRpm & 0xFF;         // ???
//			HAL_UART_Transmit(&huart2,SpeedTxBuffer,2,100);
//		}
		

		
		          /* ???? USART3 ????????????? */
        if (TIM6SendSpeedFlag != 0U)
        {
            int16_t speedRpm;
            int txLength;
            char speedTxText[32];

            /* ?? TIM6 ???? */
            TIM6SendSpeedFlag = 0U;

            /*
             * MCSDK ????? 0.1 Hz?
             * 0.1 ?/? ? 60 = 6 RPM,???? 6?
             */
            speedRpm = (int16_t)
                       ((int32_t)MC_GetMecSpeedAverageMotor1() * 6);

            /*
             * ???????????????? ASCII ????
             * \r\n ??????????
             */
            txLength = snprintf(speedTxText,
                                sizeof(speedTxText),
                                ",%d,\r\n",
                                (int)speedRpm);

            if (txLength > 0)
            {
                (void)HAL_UART_Transmit(&huart3,
                                        (uint8_t *)speedTxText,
                                        (uint16_t)txLength,
                                        100U);
            }
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_BRK_TIM15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 4, 1);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  /* TIM1_UP_TIM16_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 3, 1);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
