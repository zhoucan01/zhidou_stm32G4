/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "motorcontrol.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/**
 * F1_POWER_V004 hardware notes (authoritative detail: docs/HARDWARE_FACTS.md):
 * - M1_CURR_AMPL_U/V/W are schematic nets M_IA/M_IB/M_IC from three 0.1-ohm
 *   shunts and gain-2.2 current amplifiers.
 * - M1_OCP is PA11/TIM1_BKIN2, driven by the inverter FAULT conditioning path;
 *   verify active polarity before changing break-input configuration.
 * - Zero is the PB12 mains zero-cross input. Both schematic detector circuits
 *   carry the same ZERO net; fitted population is not established by design data.
 * - HeatBar/PB13 is connected to the single documented HEAT optotriac channel.
 * - HeatRoll/PB14 exists in firmware but is unconnected in MCU.schdoc and has no
 *   named PCB net. Do not energize or repurpose it without continuity evidence.
 * - Start_Stop/PC13 and USART3 PC10/PC11 have no confirmed external connection.
 * - ST-LINK power supports low-voltage MCU/interface tests only; it does not
 *   energize the HV bus, inverter, heater, relay, or fan power stages.
 */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Start_Stop_Pin GPIO_PIN_13
#define Start_Stop_GPIO_Port GPIOC
#define Start_Stop_EXTI_IRQn EXTI15_10_IRQn
#define M1_CURR_AMPL_W_Pin GPIO_PIN_0
#define M1_CURR_AMPL_W_GPIO_Port GPIOC
#define M1_CURR_AMPL_V_Pin GPIO_PIN_1
#define M1_CURR_AMPL_V_GPIO_Port GPIOC
#define M1_CURR_AMPL_U_Pin GPIO_PIN_0
#define M1_CURR_AMPL_U_GPIO_Port GPIOA
#define DE_RE_Pin GPIO_PIN_4
#define DE_RE_GPIO_Port GPIOA
#define M1_PWM_UL_Pin GPIO_PIN_7
#define M1_PWM_UL_GPIO_Port GPIOA
#define M1_PWM_VL_Pin GPIO_PIN_0
#define M1_PWM_VL_GPIO_Port GPIOB
#define M1_PWM_WL_Pin GPIO_PIN_1
#define M1_PWM_WL_GPIO_Port GPIOB
#define Relay_Pin GPIO_PIN_11
#define Relay_GPIO_Port GPIOB
#define Zero_Pin GPIO_PIN_12
#define Zero_GPIO_Port GPIOB
#define Zero_EXTI_IRQn EXTI15_10_IRQn
#define HeatBar_Pin GPIO_PIN_13
#define HeatBar_GPIO_Port GPIOB
#define HeatRoll_Pin GPIO_PIN_14
#define HeatRoll_GPIO_Port GPIOB
#define M1_PWM_UH_Pin GPIO_PIN_8
#define M1_PWM_UH_GPIO_Port GPIOA
#define M1_PWM_VH_Pin GPIO_PIN_9
#define M1_PWM_VH_GPIO_Port GPIOA
#define M1_PWM_WH_Pin GPIO_PIN_10
#define M1_PWM_WH_GPIO_Port GPIOA
#define M1_OCP_Pin GPIO_PIN_11
#define M1_OCP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define UART_TX_Pin GPIO_PIN_6
#define UART_TX_GPIO_Port GPIOB
#define UART_RX_Pin GPIO_PIN_7
#define UART_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
