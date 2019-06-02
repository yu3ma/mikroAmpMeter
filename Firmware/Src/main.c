
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "ssd1306.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim22;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t OldTick100ms = 0;

///////// ADC Variables and Settings
#define ADC_NUMBER_OF_CHANNELS 5u
#define ADC_NUMBER_OF_SAMPLES 32u
#define ADC_ARRAY_SIZE (ADC_NUMBER_OF_CHANNELS * ADC_NUMBER_OF_SAMPLES) // 3 Channels and 32 samples 

#define ADC_ADV 			0u 	// Voltage 						PA4 / AD CHANNEL 4
#define ADC_ADL				1u	// Current Low range 	PB0 / AD CHANNEL 8 
#define ADC_ADH				2u 	// Current High range	PB9 / AD CHANNEL 9
#define ADC_INTVREF		3u	// Internal Channel
#define ADC_INTEMPE		4u	// Internal Channel


unsigned long ADCDmaBuffer[ADC_ARRAY_SIZE];
unsigned char ADCDone = 0;

_VoltageMeasurement_type	VoltageMeasurement;
_CurrentMeasurement_type	CurrentMeasurement;

// Display Variables
unsigned char DisplayPage = MAIN_DISPLAY_PAGE0;
unsigned char DisplayClrScr = 0;

volatile unsigned long SecondsTimer = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM22_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
unsigned char GetANFHighRangeStatus(void);
void SetStatusLED(unsigned char LedState);


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void HAL_SYSTICK_Callback(void)
{
	static unsigned int Count1sec = 0;
	
	if(++Count1sec >= 1000)
	{
		Count1sec = 0;
	  SecondsTimer++;
	}
}
		
// ADC ConvCallback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
	HAL_ADC_Stop_DMA(&hadc);
	ADCDone = 1;	
}

// Ovde radimo merenje napona sa AD konvertora
void ADTasks(void)
{
	unsigned long ADCRawAcu[ADC_NUMBER_OF_CHANNELS];
	unsigned short ADCRawFiltered[ADC_NUMBER_OF_CHANNELS];
	unsigned char Pointer, Sample;
	
	if(ADCDone == 0)
		return;
	
	ADCDone = 0;
			
	// Get results from DMA buffer
	ADCRawAcu[ADC_ADV] = 0;
	ADCRawAcu[ADC_ADL] = 0;
	ADCRawAcu[ADC_ADH] = 0;
	
	ADCRawAcu[ADC_INTEMPE] = 0;
	ADCRawAcu[ADC_INTVREF] = 0;

	Pointer = 0;
	
	for(Sample=0; Sample<ADC_NUMBER_OF_SAMPLES; Sample++)
	{
		
		ADCRawAcu[ADC_ADV] += ADCDmaBuffer[Pointer + 0];
		ADCRawAcu[ADC_ADL] += ADCDmaBuffer[Pointer + 1];
		ADCRawAcu[ADC_ADH] += ADCDmaBuffer[Pointer + 2];
		ADCRawAcu[ADC_INTEMPE] += ADCDmaBuffer[Pointer + 3];
		ADCRawAcu[ADC_INTVREF] += ADCDmaBuffer[Pointer + 4];
		
		Pointer += ADC_NUMBER_OF_CHANNELS;
	}

	ADCRawFiltered[ADC_ADV] = ADCRawAcu[ADC_ADV] / ADC_NUMBER_OF_SAMPLES;
	ADCRawFiltered[ADC_ADL] = ADCRawAcu[ADC_ADL] / ADC_NUMBER_OF_SAMPLES;
	ADCRawFiltered[ADC_ADH] = ADCRawAcu[ADC_ADH] / ADC_NUMBER_OF_SAMPLES;
	ADCRawFiltered[ADC_INTEMPE] = ADCRawAcu[ADC_INTEMPE] / ADC_NUMBER_OF_SAMPLES;
	ADCRawFiltered[ADC_INTVREF] = ADCRawAcu[ADC_INTVREF] / ADC_NUMBER_OF_SAMPLES;
	
	VoltageMeasurement.ADCRaw = ADCRawFiltered[ADC_ADV];
	CurrentMeasurement.ADCLChRaw = ADCRawFiltered[ADC_ADL];	
	CurrentMeasurement.ADCHChRaw = ADCRawFiltered[ADC_ADH];
	
	/////////////////
	
	// This starts the ADC in interrupt mode
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)&ADCDmaBuffer, ADC_ARRAY_SIZE);
	
	/////////////////
	
	// Measure Voltage Calculation 
	#define RESISTOR_DIVIDER_UP	62.00 // 62k 0.1%
	#define RESISTOR_DIVIDER_DN	 4.70 // 4k7 0.1%
	
	float DividerRatio = (float)RESISTOR_DIVIDER_DN/((float)RESISTOR_DIVIDER_UP + (float)RESISTOR_DIVIDER_DN); 
	VoltageMeasurement.ADCAverage = (float)VoltageMeasurement.ADCRaw * ((float)AD_REFERENCE/(float)AD_RESOLUTION) / (float) DividerRatio; // 

	VoltageMeasurement.Cumulative += VoltageMeasurement.ADCAverage;

	// Measure Current Calculation ADL
	#define ADC_LO_RANGE_CURRENT	0.005 // 5mA
	#define ADC_LO_RANGE_VOLTAGE	2.500 // 2.5V

	#define ADC_HI_RANGE_CURRENT	2.000 // 2A
	#define ADC_HI_RANGE_VOLTAGE	2.500 // 2.5V

	CurrentMeasurement.ADCLChAverage = (float)CurrentMeasurement.ADCLChRaw * ((float)AD_REFERENCE/(float)AD_RESOLUTION) * (float)ADC_LO_RANGE_CURRENT / (float)ADC_LO_RANGE_VOLTAGE; // 

	// Measure Current Calculation ADL
	CurrentMeasurement.ADCHChAverage = (float)CurrentMeasurement.ADCHChRaw * ((float)AD_REFERENCE/(float)AD_RESOLUTION) * (float)ADC_HI_RANGE_CURRENT / (float)ADC_HI_RANGE_VOLTAGE; // 
	
	// Cumulative ADC measurements // Sample period 100ms
	if(GetANFHighRangeStatus() == ANF_HIGH_RANGE)
	{
		// HIGH RANGE

		SetStatusLED(LED_ON);

		CurrentMeasurement.ADCHCumulative += CurrentMeasurement.ADCHChAverage;
	}
	else
	{
		// LOW RANGE

		SetStatusLED(LED_OFF);

		CurrentMeasurement.ADCLCumulative += CurrentMeasurement.ADCLChAverage;
	}

	CurrentMeasurement.Samples ++;

}

void ShowDisplayTask(void)
{
	char LCDBuf[32];
	
	if(DisplayClrScr)
	{
		DisplayClrScr = 0;
		
		ssd1306_Fill(White);
		ssd1306_UpdateScreen();
	}

	switch (DisplayPage)
	{
		case MAIN_DISPLAY_PAGE0:
		{
			ssd1306_SetCursor(5,5);
			ssd1306_WriteString("mikroAMP V1",Font_11x18,Black);
			
			sprintf(LCDBuf, "Ihigh: %3.3fA", CurrentMeasurement.ADCHChAverage);
			ssd1306_SetCursor(5,24);
			ssd1306_WriteString(LCDBuf, Font_7x10,Black);

			sprintf(LCDBuf, "Ilow: %3.3fmA", CurrentMeasurement.ADCLChAverage * 1000.00);
			ssd1306_SetCursor(5,35);
			ssd1306_WriteString(LCDBuf, Font_7x10,Black);

			sprintf(LCDBuf, "Volt:%3.3fV", VoltageMeasurement.ADCAverage);
			ssd1306_SetCursor(5,46);
			ssd1306_WriteString(LCDBuf, Font_7x10,Black);
		}
		break;
		
		case MAIN_DISPLAY_PAGE1:
		{
			sprintf(LCDBuf, "Ih: %3.3fA", CurrentMeasurement.ADCHChAverage);
			ssd1306_SetCursor(5,10);
			ssd1306_WriteString(LCDBuf, Font_11x18,Black);

			sprintf(LCDBuf, "Il: %3.3fmA", CurrentMeasurement.ADCLChAverage * 1000.00);
			ssd1306_SetCursor(5,38);
			ssd1306_WriteString(LCDBuf, Font_11x18,Black);


		}
		break;		
		
		case MAIN_DISPLAY_PAGE2:
		{
			sprintf(LCDBuf, "Ihigh [A]");
			ssd1306_SetCursor(5, 5);
			ssd1306_WriteString(LCDBuf, Font_11x18, Black);

			sprintf(LCDBuf, "%3.3f", CurrentMeasurement.ADCHChAverage);
			ssd1306_SetCursor(5, 32);
			ssd1306_WriteString(LCDBuf, Font_16x26, Black);
		}
		break;			

		case MAIN_DISPLAY_PAGE3:
		{
			sprintf(LCDBuf, "Ilow [mA]");
			ssd1306_SetCursor(5, 5);
			ssd1306_WriteString(LCDBuf, Font_11x18, Black);

			sprintf(LCDBuf, "%3.3f", CurrentMeasurement.ADCLChAverage * 1000.00);
			ssd1306_SetCursor(5, 32);
			ssd1306_WriteString(LCDBuf, Font_16x26, Black);
		}
		break;			

		case MAIN_DISPLAY_PAGE4:
		{
			sprintf(LCDBuf, "Voltage [V]");
			ssd1306_SetCursor(5, 5);
			ssd1306_WriteString(LCDBuf, Font_11x18, Black);

			sprintf(LCDBuf, "%3.3f", VoltageMeasurement.ADCAverage);
			ssd1306_SetCursor(5, 32);
			ssd1306_WriteString(LCDBuf, Font_16x26,Black);
		}
		break;		
		
		case MAIN_DISPLAY_PAGE5: // Average Current summ in period of time
		{
			sprintf(LCDBuf, "Average:");
			ssd1306_SetCursor(5, 5);
			ssd1306_WriteString(LCDBuf, Font_11x18, Black);
			
			float CurrentCumulativeCalc;
			CurrentCumulativeCalc = ((float)CurrentMeasurement.ADCHCumulative + (float)CurrentMeasurement.ADCLCumulative) * 1000.00;
			CurrentCumulativeCalc /= (float)CurrentMeasurement.Samples; // We have 100ms sampling period, so must divide for 1sec resolution
			
			sprintf(LCDBuf, "%3.3f mA", CurrentCumulativeCalc);
			ssd1306_SetCursor(5, 25);
			ssd1306_WriteString(LCDBuf, Font_11x18,Black);

			unsigned char TimeHours;
			unsigned char TimeMinutes;
			unsigned char TimeSeconds;
			
			TimeHours = SecondsTimer / 3600;
			TimeMinutes = (SecondsTimer / 60) % 60;
			TimeSeconds = SecondsTimer % 60;
			
			sprintf(LCDBuf, "%02d:%02d:%02d", TimeHours, TimeMinutes, TimeSeconds);
			ssd1306_SetCursor(5, 43);
			ssd1306_WriteString(LCDBuf, Font_11x18,Black);
		}
		break;			

		case MAIN_DISPLAY_PAGE6:
		{
			ssd1306_SetCursor(5,5);
			ssd1306_WriteString(DEVICE_NAME,Font_11x18,Black);
			
			sprintf(LCDBuf, FIRMWARE_VERSION);
			ssd1306_SetCursor(5,24);
			ssd1306_WriteString(LCDBuf, Font_7x10,Black);

			sprintf(LCDBuf, HARDWARE_VERSION);
			ssd1306_SetCursor(5,35);
			ssd1306_WriteString(LCDBuf, Font_7x10,Black);

			sprintf(LCDBuf, WEBSITE_INFO);
			ssd1306_SetCursor(5,46);
			ssd1306_WriteString(LCDBuf, Font_7x10,Black);
		}
		break;
		
		default:
			sprintf(LCDBuf, "Unknown page");
			ssd1306_SetCursor(0,5);
			ssd1306_WriteString(LCDBuf, Font_7x10,Black);
			break;
		
	}
	
	ssd1306_UpdateScreen();
}

void ButtonTask(void)
{
	static unsigned char Debounce = 0;
	static unsigned char ButtonIsPressedCnt = 0;
	
	if(HAL_GPIO_ReadPin(ROTP_GPIO_Port, ROTP_Pin) == GPIO_PIN_RESET) // Button is pressed
	{
		if(++ButtonIsPressedCnt > 30) // If button is pressed more than 5 seconds
		{
			ButtonIsPressedCnt = 30;
			
			if(DisplayPage == MAIN_DISPLAY_PAGE5) // Average Current Reset
			{
				// Reset Timer and Cumulative varibles
				SecondsTimer = 0;
				
				CurrentMeasurement.ADCHCumulative = 0.0;
				CurrentMeasurement.ADCLCumulative = 0.0;
				CurrentMeasurement.Samples = 0;
			}
		}
	}
	else
	{
		if((ButtonIsPressedCnt > 0) && (ButtonIsPressedCnt < 20)) // Short Button Press
		{
			if(++DisplayPage > MAIN_DISPLAY_LAST_PAGE)
				DisplayPage = MAIN_DISPLAY_PAGE0;
				
			DisplayClrScr = 1;
		}

		ButtonIsPressedCnt = 0;

	}
}

unsigned char GetANFHighRangeStatus(void)
{	
	if(HAL_GPIO_ReadPin(HIGH_RANGE_GPIO_Port, HIGH_RANGE_Pin) == GPIO_PIN_RESET) // High range is detected
	{
		return(ANF_HIGH_RANGE);
	}
	else
	{
		return(ANF_LOW_RANGE);
	}
}

void SetStatusLED(unsigned char LedState)
{
	if(LedState == LED_ON)
	{
		HAL_GPIO_WritePin(STLED0_GPIO_Port, STLED0_Pin, GPIO_PIN_RESET); // Inverted Becouse LED is connected to Power +VDD
	}
	else
	{
		HAL_GPIO_WritePin(STLED0_GPIO_Port, STLED0_Pin, GPIO_PIN_SET); // Inverted Becouse LED inverted to Power +VDD
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  //MX_IWDG_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM22_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	
	// ADC Callibration and start
	HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);

	// This starts the ADC in interrupt mode
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)&ADCDmaBuffer, ADC_ARRAY_SIZE);

	HAL_TIM_Base_Start(&htim22); // Used for ADC sample time

	// OLED Display Init
	ssd1306_Init();
	
	HAL_Delay(1000);
  ssd1306_Fill(White);
  ssd1306_UpdateScreen();

  HAL_Delay(1000);

	// Variables initialisation
	CurrentMeasurement.ADCHCumulative = 0.0;
	CurrentMeasurement.ADCLCumulative = 0.0;
	VoltageMeasurement.Cumulative = 0.0;
	CurrentMeasurement.Samples = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// ADC tasks
		ADTasks();
		
		if(HAL_GetTick() - OldTick100ms > 100)
		{
			OldTick100ms = HAL_GetTick();
	
			//HAL_GPIO_TogglePin(STLED0_GPIO_Port, STLED0_Pin);

			// Display refresh and show data
			ButtonTask();
			
			// Scan buttons
			ShowDisplayTask();
		}		

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV64;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T22_TRGO;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00300F38;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 32;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 32;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM22 init function */
static void MX_TIM22_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim22.Instance = TIM22;
  htim22.Init.Prescaler = 31;
  htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim22.Init.Period = 999;
  htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim22) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim22, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim22, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STLED0_GPIO_Port, STLED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : FLASH_CS_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROTB_Pin ROTA_Pin */
  GPIO_InitStruct.Pin = ROTB_Pin|ROTA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : STLED0_Pin */
  GPIO_InitStruct.Pin = STLED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STLED0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROTP_Pin HIGH_RANGE_Pin */
  GPIO_InitStruct.Pin = ROTP_Pin|HIGH_RANGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
