/**
  ******************************************************************************
  * @file    pmsm_motor_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure the motor to drive.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PMSM_MOTOR_PARAMETERS_H
#define __PMSM_MOTOR_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#if defined(F1_PRODUCT_MODBUS_M1) || defined(F1_CN2_ASCII_M1)
#define POLE_PAIR_NUM          4 /* M1: operator-confirmed pole pairs */
#define RS                     2.25 /* M1: phase resistance, ohm; balanced-Y assumption */
#define LS                     0.00555 /* M1: Lq phase inductance, H
                                                 For I-PMSM it is equal to Lq */
#else
#define POLE_PAIR_NUM          2 /* M3/Zhongbai: validated generated baseline */
#define RS                     0.85 /* M3/Zhongbai: phase resistance, ohm */
#define LS                     0.005765 /* M3/Zhongbai: validated equivalent phase inductance, H
                                                 For I-PMSM it is equal to Lq */
#endif

/* When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the
   PID for speed regulation (i.e. reference torque).
   Transformation of real currents (A) into int16_t format must be done accordingly with
   formula:
   Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

#define NOMINAL_CURRENT         10922
#define MOTOR_MAX_SPEED_RPM     20000 /*!< Maximum rated speed  */
#if defined(F1_PRODUCT_MODBUS_M1) || defined(F1_CN2_ASCII_M1)
#define MOTOR_VOLTAGE_CONSTANT  8.9286 /*!< M1: Volts RMS ph-ph /kRPM */
#else
#define MOTOR_VOLTAGE_CONSTANT  9.2 /*!< M3/Zhongbai validated baseline: Vrms ph-ph/kRPM */
#endif
#define ID_DEMAG                -10922 /*!< Demagnetization current */

/***************** MOTOR SENSORS PARAMETERS  ******************************/
/* Motor sensors parameters are always generated but really meaningful only
   if the corresponding sensor is actually present in the motor         */

/*** Hall sensors ***/
#define HALL_SENSORS_PLACEMENT  DEGREES_120 /*!<Define here the
                                                 mechanical position of the sensors
                                                 withreference to an electrical cycle.
                                                 It can be either DEGREES_120 or
                                                 DEGREES_60 */

#define HALL_PHASE_SHIFT        300 /*!< Define here in degrees
                                                 the electrical phase shift between
                                                 the low to high transition of
                                                 signal H1 and the maximum of
                                                 the Bemf induced on phase A */
/*** Quadrature encoder ***/
#define M1_ENCODER_PPR             400  /*!< Number of pulses per
                                            revolution */

#endif /*__PMSM_MOTOR_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
