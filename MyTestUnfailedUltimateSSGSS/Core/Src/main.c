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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
//#define ARM_MATH_CM4 
//#include "arm_math.h"

#include "MCUFRIEND_kbv.h"
#include "math.h"
#include "ball.h"

//#include "MCUFRIEND_kbv.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SNR_THRESHOLD_F32    75.0f
#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
/* Must be a multiple of 16 */
#define NUM_TAPS_ARRAY_SIZE              32
#else
#define NUM_TAPS_ARRAY_SIZE              29
#endif
#define NUM_TAPS              29


#define TEST_LENGTH_SAMPLES  320


//#define BLOCK_SIZE            1
//#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
//static float32_t firStateF32[2 * BLOCK_SIZE + NUM_TAPS - 1];
//#else
//static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
//#endif 

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

///////////////////////LAB4 VARIABLES
int hour = 0;
int min = 0;
int sec = 0;
int tauxRafraichissement = 200; //en ms
volatile int milli = 0;
volatile int token = 1;

///////////////////////LAB6 VARIABLES
//float tab_value[256]; 
float tab_value_30inputs[30]; 
float tab_output[256]; 
float FFT_value[256];
float abs_value[128];
//uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = 256;

///////////////////////LAB7 VARIABLES
//volatile int flag_done = -1;
volatile int current_state = 0;
volatile int key = 1;		
volatile float tempsDePesee = 0;
//volatile int local_time = 0;
const float pi = 3.14159265358979323846;

volatile float positionXball; 
volatile float positionYball; //Ball
volatile float positionYball_initiale;
volatile float positionXball_initiale;
float positionXpig;
float positionYpig;  // Pig
volatile float vx ;	//is related to y coordinate for LCD
volatile float vy ;	//is related to x coordinate for LCD	
volatile float ay = -0.01;
volatile int compteur = 0;		
volatile int compteur2 = 0;		
float v_initial;	
volatile int flag_state = 0;
volatile int compteur3 = 0;		

///////////////////////LAB8 VARIABLES
#define TABLE_LENGTH 8000 
uint32_t tab_value1[TABLE_LENGTH];
uint32_t tab_value2[TABLE_LENGTH];
volatile uint32_t * tab = tab_value1;
volatile int flag_done = 0;
float A = 0.0;
float k = 0.65;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#define SUPPORT_ECRAN_1


void selectRow (int r) 
{
	if (r==1) HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);
	if (r==2) HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);
	if (r==3) HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);
	if (r==4) HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);
}

int readCol() 
{
	int result = 0;
	if (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin) == GPIO_PIN_RESET) result += 1;
	if (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin) == GPIO_PIN_RESET) result += 2;
	if (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin) == GPIO_PIN_RESET) result += 4;
	if (HAL_GPIO_ReadPin(C4_GPIO_Port, C4_Pin) == GPIO_PIN_RESET) result += 8;
	return result;
}


int keyPressed() 
{
	int rowPos=1; //Current row positiom
	int rowValue; //Value read from the current row
	for (rowPos=1; rowPos <= 4; rowPos++) 
	{
		selectRow(rowPos);
		HAL_Delay(10);
//		while (token2 == 0); //attendre 10ms
//		token2 = 0;
		rowValue=readCol();
		if (rowValue != 0) 
		{ // a key is pressed
			int result = 4*(rowPos-1);
			while (!(rowValue & 1)) 
			{ // test if bit #0 is false
				result++;
				rowValue>>=1;
			}
			while (readCol() != 0); // key no more pressed
			return result;
		}
	}
	return -1;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	// LCD initialization
	LCD_Begin();
	HAL_Delay(20);
	LCD_SetRotation(0);
	LCD_FillScreen(BLACK);
	//LCD_DrawFastHLine(0, 160, 240, YELLOW);
//	LCD_DrawCircle(N/2, N/2, 15, WHITE);
	//LCD_DrawRect(20, 40, 202, 240, RED);
	LCD_Printf("! HELLO ! ");
//	HAL_TIM_Base_Start(&htim2); 
//	HAL_TIM_Base_Start_IT(&htim2);

//	HAL_ADC_Start_DMA(&hadc1, tab, TABLE_LENGTH);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	
	int i;

	
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t *)tab, TABLE_LENGTH);		
	tab = tab_value2;
		
	
	double periode = 100.00;
	int a = 0;
	int zz= 0;
	double constante = pow(5, 0.1666666666667);
	
  while (1)
  {
		
		flag_done = 0;
		
		for (i=0;i<TABLE_LENGTH;i++) {
				double modulo = (i%((int)(periode)));
				A = 1+k*sin(4*pi*((float) i)*(1/8000.0));
				double value = 1000*A;
				if(modulo < periode/2)
					value = 0*A;
				tab[i]=(int) value;
		}
		
		while (flag_done == 0);
		
		if(a==5){
			periode/=constante;
			a=0;
			zz++;
		}
		
		if(zz==6){
			constante = 1/constante; 
		}
		
		
		if(zz==12){
			constante = 1/constante; 
			zz=0;
			periode=100.00;
		}
		
		a++;
		
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
/**
* @brief Retargets the C library printf function to the USART.
* @param None
* @retval None
*/
PUTCHAR_PROTOTYPE
{
/* Place your implementation of fputc here */
/* e.g. write a character to the USART2 and Loop until the end
of transmission */
HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
 if (htim->Instance == TIM2) 
	{
		HAL_ADC_Start(&hadc1);
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
//		vx = 1/10; //constante
//		positionYball = positionYball + 2.5;// vx*10;
//		//if (vy !=0)	vy = vy + ay;
//		vy = vy + ay;
//		positionXball = positionXball + vy*10;
//		
//		//Did the user win?
//		if (
//			((positionYpig-positionYball)*(positionYpig-positionYball)
//			+ (positionXpig-positionXball)*(positionXpig-positionXball)) <= 900
//		) 	flag_done = 1;// Win
//		else if (positionYball > 320 || positionXball > 240 || positionXball < 0) flag_done = 2;// Sort de l'écran donc loose
//		else if (positionYball >= 150 && positionYball <= 170 && positionXball <= 120) flag_done = 2;// Toucher le mur donc loose 
//		else flag_done = -1; //rien ne s'est passé
	}
}

void HAL_SYSTICK_Callback(void) 
{    
    static int local_time= 0;
    local_time++;
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t *)tab, TABLE_LENGTH);

	if (tab == tab_value1) tab = tab_value2;
	else tab = tab_value1;
	
  flag_done	= 1;
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
