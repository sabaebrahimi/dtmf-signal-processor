/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define PI 3.14159265358979

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

const int N = 200;
const long sampling_rate = 5000;
float coef[4][4];
float cosine[4][4];
float sine[4][4];
const int frows[4] = {697, 770, 852, 941};
const int fcols[4] = {1209, 1336, 1477, 1633};
int sampling_cnt = 0;
int adc_values[N];

int Q1[3], Q2[3], Q3[3], Q4[3], Q5[3], Q6[3], Q7[3], Q8[3], Q9[3], Q10[3], Q11[3], Q12[3], Q13[3], Q14[3], Q15[3], Q16[3];
int QT1[2], QT2[2], QT3[2], QT4[2], QT5[2], QT6[2], QT7[2], QT8[2], QT9[2], QT10[2], QT11[2], QT12[2], QT13[2], QT14[2], 
		QT15[2], QT16[2];
double mag_squared[8];
double mag_squared2[8];
double mag[8];
int do_goertzel = 0;
int done = 0;
char btns[4][4] = {{'1', '2', '3', 'A'}, {'4', '5', '6', 'B'}, {'7', '8', '9', 'C'}, {'*', '0', '#', 'D'}};
int btn_num[4][4] = {{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}, {13, 14, 15, 16}};
int timer_cnt = 0;
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void goertzel(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void show_data(char* str) {
	HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 20);
}

void write_sevenseg(int num) {
	int dahgan = num / 10;
	int yekan = num % 10;
	
	HAL_GPIO_WritePin(SS11_GPIO_Port, SS11_Pin, ((dahgan>>3)&0x01));
	HAL_GPIO_WritePin(SS12_GPIO_Port, SS12_Pin, ((dahgan>>2)&0x01));
	HAL_GPIO_WritePin(SS13_GPIO_Port, SS13_Pin, ((dahgan>>1)&0x01));
	HAL_GPIO_WritePin(SS14_GPIO_Port, SS14_Pin, ((dahgan>>0)&0x01));
	
	HAL_GPIO_WritePin(SS21_GPIO_Port, SS21_Pin, ((yekan>>3)&0x01));
	HAL_GPIO_WritePin(SS22_GPIO_Port, SS22_Pin, ((yekan>>2)&0x01));
	HAL_GPIO_WritePin(SS23_GPIO_Port, SS23_Pin, ((yekan>>1)&0x01));
	HAL_GPIO_WritePin(SS24_GPIO_Port, SS24_Pin, ((yekan>>0)&0x01));
}

void init_qs() {
	Q1[1] = 0; Q1[2] = 0; Q1[0] = 0; Q9[0] = 0;  Q9[1] = 0;  Q9[2] = 0;
	Q2[1] = 0; Q2[2] = 0; Q2[0] = 0; Q10[0] = 0; Q10[1] = 0; Q10[2] = 0;
	Q3[1] = 0; Q3[2] = 0; Q3[0] = 0; Q11[0] = 0; Q11[1] = 0; Q11[2] = 0;
	Q4[1] = 0; Q4[2] = 0; Q4[0] = 0; Q12[0] = 0; Q12[1] = 0; Q12[2] = 0;
	Q5[1] = 0; Q5[2] = 0; Q5[0] = 0; Q13[0] = 0; Q13[1] = 0; Q13[2] = 0;
	Q6[1] = 0; Q6[2] = 0; Q6[0] = 0; Q14[0] = 0; Q14[1] = 0; Q14[2] = 0;
	Q7[1] = 0; Q7[2] = 0; Q7[0] = 0; Q15[0] = 0; Q15[1] = 0; Q15[2] = 0;
	Q8[1] = 0; Q8[2] = 0; Q8[0] = 0; Q16[0] = 0; Q16[1] = 0; Q16[2] = 0;
}

void handle_qs() {
	for (int i = 0; i < N; i++) {
		Q1[2] = Q1[1];
		Q1[1] = Q1[0];
		Q1[0] = (coef[0][0] * Q1[1]) - Q1[2] + adc_values[i];
		
		Q2[2] = Q2[1];
		Q2[1] = Q2[0];
		Q2[0] = (coef[0][1] * Q2[1]) - Q2[2] + adc_values[i];
		
		Q3[2] = Q3[1];
		Q3[1] = Q3[0];
		Q3[0] = (coef[0][2] * Q3[1]) - Q3[2] + adc_values[i];
		
		Q4[2] = Q4[1];	
		Q4[1] = Q4[0];
		Q4[0] = (coef[0][3] * Q4[1]) - Q4[2] + adc_values[i];

		Q5[2] = Q5[1];	
		Q5[1] = Q5[0];
		Q5[0] = (coef[1][0] * Q5[1]) - Q5[2] + adc_values[i];
		
		Q6[2] = Q6[1];
		Q6[1] = Q6[0];
		Q6[0] = (coef[1][1] * Q6[1]) - Q6[2] + adc_values[i];
		
		
		Q7[2] = Q7[1];	
		Q7[1] = Q7[0];
		Q7[0] = (coef[1][2] * Q7[1]) - Q7[2] + adc_values[i];
		
		Q8[2] = Q8[1];	
		Q8[1] = Q8[0];
		Q8[0] = (coef[1][3] * Q8[1]) - Q8[2] + adc_values[i];	
		
		Q9[2] = Q9[1];
		Q9[1] = Q9[0];
		Q9[0] = (coef[2][0] * Q9[1]) - Q9[2] + adc_values[i];
		
		Q10[2] = Q10[1];
		Q10[1] = Q10[0];
		Q10[0] = (coef[2][1] * Q10[1]) - Q10[2] + adc_values[i];
		
		Q11[2] = Q11[1];
		Q11[1] = Q11[0];
		Q11[0] = (coef[2][2] * Q11[1]) - Q11[2] + adc_values[i];
		
		Q12[2] = Q12[1];
		Q12[1] = Q12[0];
		Q12[0] = (coef[2][3] * Q12[1]) - Q12[2] + adc_values[i];
		
		Q13[2] = Q13[1];
		Q13[1] = Q13[0];
		Q13[0] = (coef[3][0] * Q13[1]) - Q13[2] + adc_values[i];
		
		Q14[2] = Q14[1];
		Q14[1] = Q14[0];
		Q14[0] = (coef[3][1] * Q14[1]) - Q14[2] + adc_values[i];
		
		Q15[2] = Q15[1];
		Q15[1] = Q15[0];
		Q15[0] = (coef[3][2] * Q15[1]) - Q15[2] + adc_values[i];
		
		Q16[2] = Q16[1];
		Q16[1] = Q16[0];
		Q16[0] = (coef[3][3] * Q16[1]) - Q16[2] + adc_values[i];

	}
	
	QT1[0] = Q1[1]; QT1[1] = Q1[2];  QT9[1] = Q9[2];   QT9[0] = Q9[1];
	QT2[0] = Q2[1]; QT2[1] = Q2[2];  QT10[1] = Q10[2]; QT10[0] = Q10[1];
	QT3[0] = Q3[1]; QT3[1] = Q3[2];  QT11[1] = Q11[2]; QT11[0] = Q11[1];
	QT4[0] = Q4[1]; QT4[1] = Q4[2];  QT12[1] = Q12[2]; QT12[0] = Q12[1];
	QT5[0] = Q5[1]; QT5[1] = Q5[2];  QT13[1] = Q13[2]; QT13[0] = Q13[1];
	QT6[0] = Q6[1]; QT6[1] = Q6[2];  QT14[1] = Q14[2]; QT14[0] = Q14[1];
	QT7[0] = Q7[1]; QT7[1] = Q7[2];  QT15[1] = Q15[2]; QT15[0] = Q15[1];
	QT8[0] = Q8[1]; QT8[1] = Q8[2];  QT16[1] = Q16[2]; QT16[0] = Q16[1];
	
	Q1[1] = 0; Q1[2] = 0; Q1[0] = 0;  Q9[2]  = 0;  Q9[1]  = 0;  Q9[0]  = 0; 
	Q2[1] = 0; Q2[2] = 0; Q2[0] = 0;  Q10[2] = 0;  Q10[1] = 0;  Q10[0] = 0;
	Q3[1] = 0; Q3[2] = 0; Q3[0] = 0;  Q11[2] = 0;  Q11[1] = 0;  Q11[0] = 0;
	Q4[1] = 0; Q4[2] = 0; Q4[0] = 0;  Q12[2] = 0;  Q12[1] = 0;  Q12[0] = 0;
	Q5[1] = 0; Q5[2] = 0; Q5[0] = 0;  Q13[2] = 0;  Q13[1] = 0;  Q13[0] = 0;
	Q6[1] = 0; Q6[2] = 0; Q6[0] = 0;  Q14[2] = 0;  Q14[1] = 0;  Q14[0] = 0;
	Q7[1] = 0; Q7[2] = 0; Q7[0] = 0;  Q15[2] = 0;  Q15[1] = 0;  Q15[0] = 0;
	Q8[1] = 0; Q8[2] = 0; Q8[0] = 0;  Q16[2] = 0;  Q16[1] = 0;  Q16[0] = 0;
	
	sampling_cnt = 0;
	done = 1;
	memset(adc_values, 0, N);
	
	goertzel();
}

void adc_handler(int adc_value){
	char str[10];
	sprintf(str, "%d", adc_value);
	strcat(str, "\r");
	show_data(str);
	
	if (sampling_cnt < N) {
		adc_values[sampling_cnt] = adc_value;
		sampling_cnt++;
	} else {
		handle_qs();
	}
		
}

void TIM2_IRQHandler() {
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
		int adc_value = (int)HAL_ADC_GetValue(&hadc1);
		adc_handler(adc_value);
	}
	HAL_ADC_Stop(&hadc1); 
}

float coefi(int k) {
	return 2 * cos((float)(2.0*PI*k)/(float)N);
}

float sinei(int k) {
	return sin((float)(2.0*PI*k)/(float)N);
}

float cosinei(int k) {
	return cos((float)(2.0*PI*k)/(float)N);
}

void calc_coef() {
	int k = 0;
	for (int i = 0; i < 4; i++) {
		k = round(0.5 + ((float)(N * frows[i])/(float)sampling_rate));
		coef[0][i] = coefi(k);
		cosine[0][i] = cosinei(k);
		sine[0][i] = sinei(k);
		k = round(0.5 + ((float)(N * fcols[i])/(float)sampling_rate));
		coef[1][i] = coefi(k);
		cosine[1][i] = cosinei(k);
		sine[1][i] = sinei(k);
		k = round(0.5 + ((float)(N * 2 * frows[i])/(float)sampling_rate));
		coef[2][i] = coefi(k);
		cosine[2][i] = cosinei(k);
		sine[2][i] = sinei(k);
		k = round(0.5 + ((float)(N * 2 * fcols[i])/(float)sampling_rate));
		coef[3][i] = coefi(k);
		cosine[3][i] = cosinei(k);
		sine[3][i] = sinei(k);
	}
	/*
	for (int i = 0; i< 4; i++) {
		char str[10];
		sprintf(str, "%f", coef[0][i]);
		strcat(str, "\r");
		show_data(str);
		char str1[10];
		sprintf(str1, "%f", coef[1][i]);
		strcat(str1, "\r");
		show_data(str1);
	}
	*/
}

void goertzel() {
	if (done) {
		done = 0;
		show_data("dooonnnneeee\r");
		double scalling = (N / 2);
		
		double reals[8], imags[8], reals2[8], imags2[8];
		
		reals[0] = (QT1[0] - QT1[1] * cosine[0][0]) / scalling;
		reals[1] = (QT2[0] - QT2[1] * cosine[0][1]) / scalling;
		reals[2] = (QT3[0] - QT3[1] * cosine[0][2]) / scalling;
		reals[3] = (QT4[0] - QT4[1] * cosine[0][3]) / scalling;
		reals[4] = (QT5[0] - QT5[1] * cosine[1][0]) / scalling;
		reals[5] = (QT6[0] - QT6[1] * cosine[1][1]) / scalling;
		reals[6] = (QT7[0] - QT7[1] * cosine[1][2]) / scalling;
		reals[7] = (QT8[0] - QT8[1] * cosine[1][3]) / scalling;
		
		reals2[0] = (QT9[0] - QT9[1] * cosine[2][0]) / scalling;
		reals2[1] = (QT10[0] - QT10[1] * cosine[2][1]) / scalling;
		reals2[2] = (QT11[0] - QT11[1] * cosine[2][2]) / scalling;
		reals2[3] = (QT12[0] - QT12[1] * cosine[2][3]) / scalling;
		reals2[4] = (QT13[0] - QT13[1] * cosine[3][0]) / scalling;
		reals2[5] = (QT14[0] - QT14[1] * cosine[3][1]) / scalling;
		reals2[6] = (QT15[0] - QT15[1] * cosine[3][2]) / scalling;
		reals2[7] = (QT16[0] - QT16[1] * cosine[3][3]) / scalling;
		
		imags[0] = (QT1[1] * sine[0][0]) / scalling;
		imags[1] = (QT2[1] * sine[0][1]) / scalling;
		imags[2] = (QT3[1] * sine[0][2]) / scalling;
		imags[3] = (QT4[1] * sine[0][3]) / scalling;
		imags[4] = (QT5[1] * sine[1][0]) / scalling;
		imags[5] = (QT6[1] * sine[1][1]) / scalling;
		imags[6] = (QT7[1] * sine[1][2]) / scalling;
		imags[7] = (QT8[1] * sine[1][3]) / scalling;
		
		imags2[0] = (QT9[1]  * sine[2][0]) / scalling;
		imags2[1] = (QT10[1] * sine[2][1]) / scalling;
		imags2[2] = (QT11[1] * sine[2][2]) / scalling;
		imags2[3] = (QT12[1] * sine[2][3]) / scalling;
		imags2[4] = (QT13[1] * sine[3][0]) / scalling;
		imags2[5] = (QT14[1] * sine[3][1]) / scalling;
		imags2[6] = (QT15[1] * sine[3][2]) / scalling;
		imags2[7] = (QT16[1] * sine[3][3]) / scalling;
		
		for(int i = 0; i < 8; i ++) {
			mag_squared[i] = reals[i] * reals[i] + imags[i] * imags[i];
			mag_squared2[i] = reals2[i] * reals2[i] + imags2[i] * imags2[i];
		}
		char mag_str[8][20];
		for (int i = 0; i < 8; i ++) {
			sprintf(mag_str[i], "%lf\r",  mag_squared[i]);
			show_data(mag_str[i]);
		}
		
		double sum_squared2 = 0;
		for(int i = 0; i < 8; i ++) {
			sum_squared2 += mag_squared2[i];
		}
		
		//if (sum_squared2 > 100000.0 || sum_squared2 < 1) {
		//	write_sevenseg(0);
		//	show_data("invalid signals ");
		//} 
		
		//else {
			int max_rows = 0, max_cols = 4;
			for (int i = 0; i < 4; i ++) {
				if (mag_squared[max_rows] < mag_squared[i]) 
					max_rows = i;
			}
			for (int i = 4; i < 8; i++) {
				if (mag_squared[max_cols] < mag_squared[i])
					max_cols = i;
			}
			
			
			int result_num = btn_num[max_rows][max_cols - 4];
			char xx[4];
			sprintf(xx, "%d\r", result_num);
			show_data(xx);
			write_sevenseg(result_num);
		
			char* result = " result: ";
			show_data(result);
			
			char btn = btns[max_rows][max_cols - 4];
			char schar[3];
			sprintf(schar, "%c\r", btn);
			show_data(schar);
		}
	//}
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	init_qs();
	calc_coef();
	
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	
	int x, y;
	int z, t;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//x = DWT->CYCCNT;
		

		//y = DWT->CYCCNT;
		//show_data("duration: ");
		//char timechar[10];
		//sprintf(timechar, "%d\r", (y - x));
		//show_data(timechar);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 159999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SS11_Pin|SS12_Pin|SS13_Pin|SS14_Pin
                          |SS21_Pin|SS22_Pin|SS23_Pin|SS24_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SS11_Pin SS12_Pin SS13_Pin SS14_Pin
                           SS21_Pin SS22_Pin SS23_Pin SS24_Pin */
  GPIO_InitStruct.Pin = SS11_Pin|SS12_Pin|SS13_Pin|SS14_Pin
                          |SS21_Pin|SS22_Pin|SS23_Pin|SS24_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

