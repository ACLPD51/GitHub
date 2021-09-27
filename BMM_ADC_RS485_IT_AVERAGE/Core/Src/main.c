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
#include <math.h>
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
ADC_ChannelConfTypeDef sConfig = {0};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//*************************funtion creation to select de channels ADC*******************************************
void SelectCh4 (void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}


void SelectCh5 (void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_5;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }


}

void SelectCh6 (void){
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_6;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }


}


void SelectCh7 (void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_7;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}

void SelectCh8 (void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_8;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}
void SelectCh9 (void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_9;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}

void SelectCh12 (void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_12;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}

void SelectCh14 (void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_14;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}


void SelectCh15 (void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_15;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}
//******************************************GPIO INITIALIZATION BEFORE START LOOP.............
void UART_BMM_INIT (void){


	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);



	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
}

#define buffer 100 //BUFFER TO READ VALUES OF ADC
//BUFFERS 9 BUFFERS OF 100 VALUES
int ADC_VAL[buffer];
int ADC_VAL2[buffer];
int ADC_VAL3[buffer];
int ADC_VAL4[buffer];
int ADC_VAL5[buffer];
int ADC_VAL6[buffer];
int ADC_VAL7[buffer];
int ADC_VAL8[buffer];
int ADC_VAL9[buffer];
//*********FLAG TO INDICATION OF BUFFERS ARE FILL*************
int FLAG_ADC=0;
//************STRINGS TO SEND FOR RS485************
char DATA[30];
char DATA2[30];
char DATA3[30];
char DATA4[30];
char DATA5[30];
char DATA6[30];
char DATA7[30];
char DATA8[30];
char DATA9[30];
//*******************AVERAGE TO ADC'S  100 VALUES*************
double AVG1=0;
double AVG2=0;
double AVG3=0;
double AVG4=0;
double AVG5=0;
double AVG6=0;
double AVG7=0;
double AVG8=0;
double AVG9=0;

//**************STANDARD DEVIATION OF adc'S*****************

double SD1=0;
double SD2=0;
double SD3=0;
double SD4=0;
double SD5=0;
double SD6=0;
double SD7=0;
double SD8=0;
double SD9=0;
//*****************VARIABLES TO TRANSFORM DOUBLE  INTO INTERGET
int SD1_INT=0;
int SD2_INT=0;
int SD3_INT=0;
int SD4_INT=0;
int SD5_INT=0;
int SD6_INT=0;
int SD7_INT=0;
int SD8_INT=0;
int SD9_INT=0;

int AVG_INT1=0;
int AVG_INT2=0;
int AVG_INT3=0;
int AVG_INT4=0;
int AVG_INT5=0;
int AVG_INT6=0;
int AVG_INT7=0;
int AVG_INT8=0;
int AVG_INT9=0;


//*********INDICE TO TRAVEL ON THE BUFFFER*************
int indice=0;
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  UART_BMM_INIT();	//inicializamos pins para rs485

  HAL_TIM_Base_Start_IT(&htim2); // iniciamos timer2




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if (FLAG_ADC == 1) {
			FLAG_ADC=0;

			//calculate the average of each channel


			for(int j=0;j<=99;j++){
							AVG1=AVG1+ADC_VAL[j];


						}
			AVG1=AVG1/100;
			//avegare calculated
			//caculate de standard deviation


			for(int j=0;j<=99;j++){
			  SD1+=pow(ADC_VAL[j]-AVG1,2);
			  SD1=sqrt(SD1/100);
			 }


			AVG_INT1=AVG1;
			SD1_INT=SD1*1000;
			for(int j=0;j<=99;j++){
							AVG2=AVG2+ADC_VAL2[j];

						}
			AVG2=AVG2/100;
			//avegare calculated
						//caculate de standard deviation


						for(int j=0;j<=99;j++){
						  SD2+=pow(ADC_VAL2[j]-AVG2,2);
						  SD2=sqrt(SD2/100);
						 }


						AVG_INT2=AVG2;
						SD2_INT=SD2*1000;

			for(int j=0;j<=99;j++){
							AVG3=AVG3+ADC_VAL3[j];

						}
			AVG3=AVG3/100;
			//avegare calculated
						//caculate de standard deviation


						for(int j=0;j<=99;j++){
						  SD3+=pow(ADC_VAL3[j]-AVG3,2);
						  SD3=sqrt(SD3/100);
						 }


						AVG_INT3=AVG3;
						SD3_INT=SD3*1000;

			for(int j=0;j<=99;j++){
							AVG4=AVG4+ADC_VAL4[j];

						}
			AVG4=AVG4/100;
			//avegare calculated
						//caculate de standard deviation


						for(int j=0;j<=99;j++){
						  SD4+=pow(ADC_VAL4[j]-AVG4,2);
						  SD4=sqrt(SD4/100);
						 }


						AVG_INT4=AVG4;
						SD4_INT=SD4*1000;
			for(int j=0;j<=99;j++){
							AVG5=AVG5+ADC_VAL5[j];

						}
			AVG5=AVG5/100;
			//avegare calculated
						//caculate de standard deviation


						for(int j=0;j<=99;j++){
						  SD5+=pow(ADC_VAL5[j]-AVG5,2);
						  SD5=sqrt(SD5/100);
						 }


						AVG_INT5=AVG5;
						SD5_INT=SD5*1000;

			for(int j=0;j<=99;j++){
							AVG6=AVG6+ADC_VAL6[j];

						}
			AVG6=AVG6/100;
			//avegare calculated
						//caculate de standard deviation


						for(int j=0;j<=99;j++){
						  SD6+=pow(ADC_VAL6[j]-AVG6,2);
						  SD6=sqrt(SD6/100);
						 }


						AVG_INT6=AVG6;
						SD6_INT=SD6*1000;

			for(int j=0;j<=99;j++){
							AVG7=AVG7+ADC_VAL7[j];

						}
			AVG7=AVG7/100;
			//avegare calculated
						//caculate de standard deviation


						for(int j=0;j<=99;j++){
						  SD7+=pow(ADC_VAL7[j]-AVG7,2);
						  SD7=sqrt(SD7/100);
						 }

//***********************************TEMPERATURE MEASUREMENT*************************
						 double Rntc=0;
						 double Vtemp=(AVG7*2.5)/4096;

						 Rntc=(11000/(Vtemp+0.189268/2.8571425))-11000;

						 //Rntc=((27500/(Vtemp/0.9530))-27500)/2.5;

						double TempNTC=pow((((Rntc/10000)/3976)+(1/298.15)),-1);
						 TempNTC=TempNTC-273.15;

//******************************************************************
						AVG_INT7=AVG7;
						SD7_INT=SD7*1000;

			for(int j=0;j<=99;j++){
							AVG8=AVG8+ADC_VAL8[j];

						}
			AVG8=AVG8/100;
			//avegare calculated
						//caculate de standard deviation


						for(int j=0;j<=99;j++){
						  SD8+=pow(ADC_VAL8[j]-AVG8,2);
						  SD8=sqrt(SD8/100);
						 }


						AVG_INT8=AVG8;
						SD8_INT=SD8*1000;

			for(int j=0;j<=99;j++){
							AVG9=AVG9+ADC_VAL9[j];

									}
			AVG9=AVG9/100;
			//avegare calculated
						//caculate de standard deviation


						for(int j=0;j<=99;j++){
						  SD9+=pow(ADC_VAL9[j]-AVG9,2);
						  SD9=sqrt(SD9/100);
						 }


						AVG_INT9=AVG9;
						SD9_INT=SD9*1000;




 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_UART_Transmit(&huart1, "V1:", 3, 100);
			//for (int i = 0; i <= 99; i++) {

				sprintf(DATA, ";%d;----SIGMA:;%d", AVG_INT1,SD1_INT);


				HAL_UART_Transmit(&huart1, DATA, sizeof(DATA), HAL_MAX_DELAY);

			//}
			HAL_UART_Transmit(&huart1, "\n\r", 4, 100);

			HAL_UART_Transmit(&huart1, "V2:", 3, 100);
			//for (int i = 0; i <= 99; i++) {

			sprintf(DATA2, ";%d;----SIGMA:;%d", AVG_INT2,SD2_INT);

				HAL_UART_Transmit(&huart1, DATA2, sizeof(DATA2), HAL_MAX_DELAY);

			//}
			HAL_UART_Transmit(&huart1, "\n\r", 4, 100);

			HAL_UART_Transmit(&huart1, "V3:", 3, 100);
			//for (int i = 0; i <= 99; i++) {

			sprintf(DATA3, ";%d;----SIGMA:;%d", AVG_INT3,SD3_INT);

				HAL_UART_Transmit(&huart1, DATA3, sizeof(DATA3), HAL_MAX_DELAY);

			//}
			HAL_UART_Transmit(&huart1, "\n\r", 4, 100);

			HAL_UART_Transmit(&huart1, "V4:", 3, 100);
			//for (int i = 0; i <= 99; i++) {

			sprintf(DATA4, ";%d;----SIGMA:;%d", AVG_INT4,SD4_INT);

				HAL_UART_Transmit(&huart1, DATA4, sizeof(DATA4), HAL_MAX_DELAY);

			//}
			HAL_UART_Transmit(&huart1, "\n\r", 4, 100);

			HAL_UART_Transmit(&huart1, "V5:", 3, 100);
			//for (int i = 0; i <= 99; i++) {

			sprintf(DATA5, ";%d;----SIGMA:;%d", AVG_INT5,SD5_INT);

				HAL_UART_Transmit(&huart1, DATA5, sizeof(DATA5), HAL_MAX_DELAY);

			//}
			HAL_UART_Transmit(&huart1, "\n\r", 4, 100);

			HAL_UART_Transmit(&huart1, "V6:", 3, 100);
			//for (int i = 0; i <= 99; i++) {

			sprintf(DATA6, ";%d;----SIGMA:;%d", AVG_INT6,SD6_INT);

				HAL_UART_Transmit(&huart1, DATA6, sizeof(DATA6), HAL_MAX_DELAY);

			//}
			HAL_UART_Transmit(&huart1, "\n\r", 4, 100);

			HAL_UART_Transmit(&huart1, "V7:", 3, 100);
			//for (int i = 0; i <= 99; i++) {

			sprintf(DATA7, ";%d;----SIGMA:;%d", AVG_INT7,SD7_INT);

				HAL_UART_Transmit(&huart1, DATA7, sizeof(DATA7), HAL_MAX_DELAY);

			//}
			HAL_UART_Transmit(&huart1, "\n\r", 4, 100);

			HAL_UART_Transmit(&huart1, "V8:", 3, 100);
			//for (int i = 0; i <= 99; i++) {

			sprintf(DATA8, ";%d;----SIGMA:;%d", AVG_INT8,SD8_INT);

				HAL_UART_Transmit(&huart1, DATA8, sizeof(DATA8), HAL_MAX_DELAY);

			//}
			HAL_UART_Transmit(&huart1, "\n\r", 4, 100);

			HAL_UART_Transmit(&huart1, "V9", 3, 100);
			//for (int i = 0; i <= 99; i++) {

			sprintf(DATA9, ";%d;----SIGMA:;%d", AVG_INT9,SD9_INT);

				HAL_UART_Transmit(&huart1, DATA9, sizeof(DATA9), HAL_MAX_DELAY);

			//}
			HAL_UART_Transmit(&huart1, "\n\r", 4, 100);




		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
		  HAL_TIM_Base_Start_IT(&htim2);

//reset variables
		  AVG1=0;
		  AVG2=0;
		  AVG3=0;
		  AVG4=0;
		  AVG5=0;
		  AVG6=0;
		  AVG7=0;
		  AVG8=0;
		  AVG9=0;
		  SD1=0;
		  SD2=0;
		  SD3=0;
		  SD4=0;
		  SD5=0;
		  SD6=0;
		  SD7=0;
		  SD8=0;
		  SD9=0;




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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 9;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 9;
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
  htim2.Init.Prescaler = 8400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  huart1.Init.BaudRate = 460800;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE9 PE10 PE13 PE14
                           PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void 	HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim){




		SelectCh4();
	  	  	HAL_ADC_Start(&hadc1);
	  	  	HAL_ADC_PollForConversion(&hadc1, 100);
	  	    ADC_VAL[indice]=HAL_ADC_GetValue(&hadc1);
	  	    HAL_ADC_Stop(&hadc1);

	  	  SelectCh5();
	  	  	  	  	HAL_ADC_Start(&hadc1);
	  	  	  	  	HAL_ADC_PollForConversion(&hadc1, 100);
	  	  	  	    ADC_VAL2[indice]=HAL_ADC_GetValue(&hadc1);
	  	  	  	    HAL_ADC_Stop(&hadc1);
	  	  	  	SelectCh6();
	  	  	  		  	  	HAL_ADC_Start(&hadc1);
	  	  	  		  	  	HAL_ADC_PollForConversion(&hadc1, 100);
	  	  	  		  	    ADC_VAL3[indice]=HAL_ADC_GetValue(&hadc1);
	  	  	  		  	    HAL_ADC_Stop(&hadc1);
	  	  	  		  SelectCh7();
	  	  	  		  	  	  	HAL_ADC_Start(&hadc1);
	  	  	  		  	  	  	HAL_ADC_PollForConversion(&hadc1, 100);
	  	  	  		  	  	    ADC_VAL4[indice]=HAL_ADC_GetValue(&hadc1);
	  	  	  		  	  	    HAL_ADC_Stop(&hadc1);
	  	  	  		  	SelectCh8();
	  	  	  		  		  	  	HAL_ADC_Start(&hadc1);
	  	  	  		  		  	  	HAL_ADC_PollForConversion(&hadc1, 100);
	  	  	  		  		  	    ADC_VAL5[indice]=HAL_ADC_GetValue(&hadc1);
	  	  	  		  		  	    HAL_ADC_Stop(&hadc1);
	  	  	  		  		SelectCh9();
	  	  	  		  			  	  	HAL_ADC_Start(&hadc1);
	  	  	  		  			  	  	HAL_ADC_PollForConversion(&hadc1, 100);
	  	  	  		  			  	    ADC_VAL6[indice]=HAL_ADC_GetValue(&hadc1);
	  	  	  		  			  	    HAL_ADC_Stop(&hadc1);
	  	  	  		  			SelectCh12();
	  	  	  		  				  	  	HAL_ADC_Start(&hadc1);
	  	  	  		  				  	  	HAL_ADC_PollForConversion(&hadc1, 100);
	  	  	  		  				  	    ADC_VAL7[indice]=HAL_ADC_GetValue(&hadc1);
	  	  	  		  				  	    HAL_ADC_Stop(&hadc1);
	  	  	  		  				SelectCh14();
	  	  	  		  					  	  	HAL_ADC_Start(&hadc1);
	  	  	  		  					  	  	HAL_ADC_PollForConversion(&hadc1, 100);
	  	  	  		  					  	    ADC_VAL8[indice]=HAL_ADC_GetValue(&hadc1);
	  	  	  		  					  	    HAL_ADC_Stop(&hadc1);


	  	  	  		  					SelectCh15();
	  	  	  		  						  	  	HAL_ADC_Start(&hadc1);
	  	  	  		  						  	  	HAL_ADC_PollForConversion(&hadc1, 100);
	  	  	  		  						  	    ADC_VAL9[indice]=HAL_ADC_GetValue(&hadc1);
	  	  	  		  						  	    HAL_ADC_Stop(&hadc1);





	  	    indice++;
	  	    if (indice>=buffer){
	  	    	HAL_TIM_Base_Stop_IT(&htim2);
	  	    	FLAG_ADC=1;
 	  	    		indice=0;
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
