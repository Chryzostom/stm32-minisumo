/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SW1 HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin)
#define SW2 HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin)
#define SW3 HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin)
#define SF HAL_GPIO_ReadPin(SF_GPIO_Port, SF_Pin)
#define SR HAL_GPIO_ReadPin(SR_GPIO_Port, SR_Pin)
#define SL HAL_GPIO_ReadPin(SL_GPIO_Port, SL_Pin)
#define START HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t linia[2];
u_int16_t ls[4];

int takt = 1;
extern volatile int start;

volatile int ust_D_P;
volatile int ust_D_L;
volatile int zad_V_L;
volatile int zad_V_P;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void lsens(uint32_t linia[]);
void led(int num);
int wybor(int taktyka);
void LS_tyl(void);
void LS_przod(void);
void LS_stop(void);
void PS_tyl(void);
void PS_przod(void);
void PS_stop(void);
void przod(void);
void tyl(void);
void stoj(void);
void lewy_szybko(void);
void prawy_szybko(void);
void lewy(void);
void prawy(void);
void bialaL();
void zerujE();
void soft_start();
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //////////////////Inicjalizacja czujnikow i ustawianie adresow//////////////////
  led(1);
  stoj(); //silniki stan niski

  HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL); //start enkodery

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); //start pwm silnikow

  led(0); //koniec inicjalizacji
  /////////////////////////////////////////////////////////////////////////////////
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(START == 1)
	  {
		  led(1);
		  zerujE();
		  if(SL == 1) // lewy widzi
		  {
			while(ust_D_L<12000 && ust_D_P<12000)
			{
				lewy();
				zad_V_L = 450;
				zad_V_P = 400;
			}
			zerujE();
			bialaL();
			stoj();
		  }
		  else if(SR == 1) //prawy widzi
		  {
			while(ust_D_L<12000 && ust_D_P<12000)
			{
				prawy();
				zad_V_L = 400;
				zad_V_P = 450;
			}
			zerujE();
			bialaL();
			stoj();
		  }
		  else if(SF == 1) //przod widzi
		  {
			  soft_start();
			  bialaL();
		  }
		  else
		  {
			  prawy();
			  zad_V_L = 400;
			  zad_V_P = 450;
			  bialaL();
		  }
		  zerujE();
	  }
	  else if(START == 0)
	  {
		led(0);
		stoj();
		zad_V_L = 0;
		zad_V_P = 0;
		bialaL();
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void LS_tyl(void)
{
	HAL_GPIO_WritePin(I1_1_GPIO_Port, I1_1_Pin, RESET);
	HAL_GPIO_WritePin(I1_2_GPIO_Port, I1_2_Pin, SET);
}

void LS_przod(void)
{
	HAL_GPIO_WritePin(I1_1_GPIO_Port, I1_1_Pin, SET);
	HAL_GPIO_WritePin(I1_2_GPIO_Port, I1_2_Pin, RESET);
}

void LS_stop(void)
{
	HAL_GPIO_WritePin(I1_1_GPIO_Port, I1_1_Pin, RESET);
	HAL_GPIO_WritePin(I1_2_GPIO_Port, I1_2_Pin, RESET);
}

void PS_tyl(void)
{
	HAL_GPIO_WritePin(I2_1_GPIO_Port, I2_1_Pin, SET);
	HAL_GPIO_WritePin(I2_2_GPIO_Port, I2_2_Pin, RESET);
}

void PS_przod(void)
{
	HAL_GPIO_WritePin(I2_1_GPIO_Port, I2_1_Pin, RESET);
	HAL_GPIO_WritePin(I2_2_GPIO_Port, I2_2_Pin, SET);
}

void PS_stop(void)
{
	HAL_GPIO_WritePin(I2_1_GPIO_Port, I2_1_Pin, RESET);
	HAL_GPIO_WritePin(I2_2_GPIO_Port, I2_2_Pin, RESET);
}

void przod(void)
{
	PS_przod();
	LS_przod();
}

void tyl(void)
{
	PS_tyl();
	LS_tyl();
}

void stoj(void)
{
	PS_stop();
	LS_stop();
}
void lewy_szybko(void)
{
	PS_stop();
	LS_tyl();
}

void prawy_szybko(void)
{
	PS_tyl();
	LS_stop();
}

void lewy(void)
{
	PS_przod();
	LS_tyl();
}

void prawy(void)
{
	PS_tyl();
	LS_przod();
}

void lsens(uint32_t linia[])
{
	int i;
	for(i=0;i<2;i++)
	{
		if(linia[i]<=300) ls[i]=1;
		else ls[i]=0;
	}
}

void led(int num)
{
	switch(num)
	{
	case 0:
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
		break;
	}
}

int wybor(int taktyka)
{
	if(SW3==RESET)
	{
		taktyka++;
		if(taktyka > 3) taktyka = 1;
		HAL_Delay(200);
	}
	else if(SW1==RESET)
	{
		taktyka--;
		if(taktyka < 1) taktyka = 3;
		HAL_Delay(200);
	}
	led(taktyka);
	return taktyka;
}

void zerujE()
{
	ust_D_P = 0;
	ust_D_L = 0;
}

void soft_start()
{
		zerujE();
		while(ust_D_L < 4000 && ust_D_P < 4000)
		{
			przod();
			zad_V_L = 50;
			zad_V_P = 50;
		}
		zerujE();

		while(ust_D_L < 2000 && ust_D_P < 2000)
		{
			przod();
			zad_V_L = 100;
			zad_V_P = 100;
		}
		zerujE();

		while(ust_D_L < 2000 && ust_D_P < 2000)
		{
			przod();
			zad_V_L = 300;
			zad_V_P = 300;
		}
		zerujE();

		while(ust_D_L < 2000 && ust_D_P < 2000)
		{
			przod();
			zad_V_L = 700;
			zad_V_P = 700;
		}
		zerujE();

		while(ust_D_L < 2000 && ust_D_P < 2000)
		{
			przod();
			zad_V_L = 1000;
			zad_V_P = 1000;
		}
		zerujE();
}

void bialaL()
{
	zerujE();
	if(linia[0]<1000 && linia[1]>1000) //prawy widzi linie
	{
		stoj();
		while(ust_D_L<10000 && ust_D_P<10000)
		{
			tyl();
			zad_V_L = 200;
			zad_V_P = 200;
		}
		zerujE();
		while(ust_D_L<20000)
		{
			lewy();
			zad_V_L = 200;
			zad_V_P = 150;
		}
		stoj();
	}
	else if(linia[0]>1000 && linia[1]<1000) //lewy widzi linie
	{
		stoj();
		while(ust_D_L<10000 && ust_D_P<10000)
		{
			tyl();
			zad_V_L = 200;
			zad_V_P = 200;
		}
		zerujE();
		while(ust_D_P<20000)
		{
			prawy();
			zad_V_L = 150;
			zad_V_P = 200;
		}
		stoj();
	}
	else if(linia[0]<1000 && linia[1]<1000) //oba widza linie
	{
		stoj();
		while(ust_D_L<30000 && ust_D_P<30000)
		{
			tyl();
			zad_V_L = 200;
			zad_V_P = 200;
		}
		stoj();
		zerujE();
		while(ust_D_L<60000)
		{
			lewy();
			zad_V_L = 200;
			zad_V_P = 150;
		}
		stoj();
	}
	zerujE();
}

void taktyka2()
{
	zerujE();
	while(ust_D_L<5000 && ust_D_P<5000)
	{
		przod();
		zad_V_L=100;
		zad_V_P=100;
	} //powoli do przodu
	zerujE();
	while(ust_D_L<50000 && ust_D_P<50000)
	{
		przod();
		zad_V_L=300;
		zad_V_P=300;
	} //szybciej do przodu
	stoj();
	zad_V_L=0;
	zad_V_P=0;
	zerujE(); //koniec funkcji, dalej program podstawowy
}
void taktyka3()
{
	int flag = 1;
	while(flag==1)
	{
		zerujE();
		przod();
		zad_V_L=150;
		zad_V_P=150; //jedz do przodu az do linii
		if(linia[0]<1000 && linia[1]>1000)
		{
			stoj();
			while(ust_D_L<10000 && ust_D_P<10000)
			{
				tyl();
				zad_V_L = 200;
				zad_V_P = 200; //delikatne cofniecie sie
			}
			zerujE();
			while(ust_D_L<20000)
			{
				lewy();
				zad_V_L = 200;
				zad_V_P = 150; //obrot w lewo
			}
			stoj();
			flag=0; //koniec funkcji, dalej program podstawowy
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
