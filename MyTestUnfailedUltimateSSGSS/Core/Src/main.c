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
#include "shapes.h"
#include "tetris.h"

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
//	LCD_Printf("! HELLO ! ");
//	HAL_TIM_Base_Start(&htim2); 
//	HAL_TIM_Base_Start_IT(&htim2);

//	HAL_ADC_Start_DMA(&hadc1, tab, TABLE_LENGTH);
	LCD_FillScreen(DARKGREY); 
	initializeField();
	displayField();
	
	//PWM TRIG START
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	
	
  while (1)
  {
				
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		//Random movements for Tetris
		if (updateField()){ // Quand "updateField" renvoie 1, la partie est perdue.
        LCD_FillScreen(RED); 
        while(1);
    }
    HAL_Delay(25);
//    randomDisplaceOrRotate(); // Mouvements aléatoires pour simuler une partie.
		selectRow(1);
    HAL_Delay(10);

    HAL_Delay(75);
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

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
//{
// if (htim->Instance == TIM2) 
//	{
//		HAL_ADC_Start(&hadc1);
//		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
//	}
//}

void HAL_SYSTICK_Callback(void) 
{    
	 static bool left = false;
    static bool right = false;
    static bool rotate = false;
    int row = readCol();
    if (row == 1){
        if (!left)
            displaceTetrimino(-1, 0, &tetrimino);
        left = true;
    }
    else {
        left = false;
    }
    if (row == 2){
        if (!right)
            displaceTetrimino(1, 0, &tetrimino);
        right = true;
    }
    else {
        right = false;
    }
    if (row == 4){
        if (!rotate)
            rotateTetrimino(&tetrimino);
        rotate = true;
    }
    else {
        rotate = false;
    }
}


//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
//	
//	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t *)tab, TABLE_LENGTH);

//	if (tab == tab_value1) tab = tab_value2;
//	else tab = tab_value1;
//	
//  flag_done	= 1;
//}
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
