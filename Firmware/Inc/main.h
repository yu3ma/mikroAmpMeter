/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define FLASH_CS_Pin GPIO_PIN_14
#define FLASH_CS_GPIO_Port GPIOC
#define MC_TXD2_Pin GPIO_PIN_2
#define MC_TXD2_GPIO_Port GPIOA
#define MC_RXD2_Pin GPIO_PIN_3
#define MC_RXD2_GPIO_Port GPIOA
#define ADV_Pin GPIO_PIN_4
#define ADV_GPIO_Port GPIOA
#define ROTB_Pin GPIO_PIN_5
#define ROTB_GPIO_Port GPIOA
#define STLED0_Pin GPIO_PIN_6
#define STLED0_GPIO_Port GPIOA
#define ROTA_Pin GPIO_PIN_7
#define ROTA_GPIO_Port GPIOA
#define ADL_Pin GPIO_PIN_0
#define ADL_GPIO_Port GPIOB
#define ADH_Pin GPIO_PIN_1
#define ADH_GPIO_Port GPIOB
#define ROTP_Pin GPIO_PIN_8
#define ROTP_GPIO_Port GPIOA
#define HIGH_RANGE_Pin GPIO_PIN_15
#define HIGH_RANGE_GPIO_Port GPIOA
#define MX_TXD1_Pin GPIO_PIN_6
#define MX_TXD1_GPIO_Port GPIOB
#define MC_RXD1_Pin GPIO_PIN_7
#define MC_RXD1_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

// **************************************************************************************
// **************************************************************************************
// AD Definicije
// **************************************************************************************
// **************************************************************************************
#define AD_REFERENCE  2.5000 // U Voltima
#define AD_RESOLUTION 4096ul // U Digitima

typedef struct
{
	unsigned short ADCLChRaw;
	unsigned short ADCHChRaw;
	float ADCLChAverage;
	float ADCHChAverage;
	
	unsigned char LRange;
	unsigned char HRange;
	
} _CurrentMeasurement_type;

typedef struct
{
	unsigned short ADCRaw;
	float ADCAverage;
	
} _VoltageMeasurement_type;


extern _VoltageMeasurement_type	VoltageMeasurement;
extern _CurrentMeasurement_type	CurrentMeasurement;

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
