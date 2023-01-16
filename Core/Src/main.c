/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    LED_OFF,
    LED_ON_START,
    LED_DIM_ON,
    LED_DIM_OFF,
    LED_ON
}led_state_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static led_state_t led_state = LED_OFF;
static uint32_t led_dim = 0;
static uint32_t led_target = 0;
static uint32_t led_delay = 100;
static uint32_t pulsos = 0;
static uint32_t cont = 0;

TIM_HandleTypeDef  TimHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);


/* USER CODE BEGIN PFP */
static void GPIO_init(void);
static void EXTI15_10_IRQHandler_Config(void);
static void TIM_Init(void);
static void generate_Dimmer(void);
static void LED_CTRL_vfnChangeIntensity (void);
static void LED_CTRL_vfnPeriodElapsedCallback (void);
//void HAL_TIM_PeriodElapsedCallback(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
__IO uint32_t uwPrescalerValue = 0;
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
  /* USER CODE BEGIN 2 */
  EXTI15_10_IRQHandler_Config();
  GPIO_init();
  TIM_Init();
  generate_Dimmer();
  /* USER CODE END 2 */
  // led 1
  while (cont <32)
  {
	  LED_CTRL_vfnChangeIntensity();
	  HAL_Delay(1000);
	  cont++;

  }

//    //led 2
//    LED_CTRL_vfnChangeIntensity();
//    HAL_Delay(4000);
//    //led 3
//    LED_CTRL_vfnChangeIntensity();
//    HAL_Delay(4000);
//    // led 4
//    LED_CTRL_vfnChangeIntensity();
//    HAL_Delay(4000);
//    //led 5
//    LED_CTRL_vfnChangeIntensity();
//    HAL_Delay(4000);
//    //led 6
//    LED_CTRL_vfnChangeIntensity();
//    HAL_Delay(4000);
//    // led 7
//     LED_CTRL_vfnChangeIntensity();
//     HAL_Delay(4000);
//     //led 8
//     LED_CTRL_vfnChangeIntensity();
//     HAL_Delay(4000);
//     //led 9
//     LED_CTRL_vfnChangeIntensity();
//     HAL_Delay(4000);
//     //led 10
//     LED_CTRL_vfnChangeIntensity();
//     HAL_Delay(4000);
//
//
//
//     // led 1
//     LED_CTRL_vfnChangeIntensity();
//     HAL_Delay(4000);
//     //led 2
//     LED_CTRL_vfnChangeIntensity();
//     HAL_Delay(4000);
//     //led 3
//     LED_CTRL_vfnChangeIntensity();
//     HAL_Delay(4000);
//     // led 4
//     LED_CTRL_vfnChangeIntensity();
//     HAL_Delay(4000);
//     //led 5
//     LED_CTRL_vfnChangeIntensity();
//     HAL_Delay(4000);
//     //led 6
//     LED_CTRL_vfnChangeIntensity();
//     HAL_Delay(4000);
//     // led 7
//      LED_CTRL_vfnChangeIntensity();
//      HAL_Delay(4000);
//      //led 8
//      LED_CTRL_vfnChangeIntensity();
//      HAL_Delay(4000);
//      //led 9
//      LED_CTRL_vfnChangeIntensity();
//      HAL_Delay(4000);
//      //led 10
//      LED_CTRL_vfnChangeIntensity();
//      HAL_Delay(4000);
//
//
//
//
//      // led 1
//      LED_CTRL_vfnChangeIntensity();
//      HAL_Delay(4000);
//      //led 2
//      LED_CTRL_vfnChangeIntensity();
//      HAL_Delay(4000);
//      //led 3
//      LED_CTRL_vfnChangeIntensity();
//      HAL_Delay(4000);
//      // led 4
//      LED_CTRL_vfnChangeIntensity();
//      HAL_Delay(4000);
//      //led 5
//      LED_CTRL_vfnChangeIntensity();
//      HAL_Delay(4000);
//      //led 6
//      LED_CTRL_vfnChangeIntensity();
//      HAL_Delay(4000);
//      // led 7
//       LED_CTRL_vfnChangeIntensity();
//       HAL_Delay(4000);
//       //led 8
//       LED_CTRL_vfnChangeIntensity();
//       HAL_Delay(4000);
//       //led 9
//       LED_CTRL_vfnChangeIntensity();
//       HAL_Delay(4000);
//       //led 10
//       LED_CTRL_vfnChangeIntensity();
//       HAL_Delay(4000);
//
//
//
//       // led 1
//       LED_CTRL_vfnChangeIntensity();
//       HAL_Delay(4000);
//       //led 2
//       LED_CTRL_vfnChangeIntensity();
//       HAL_Delay(4000);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  huart2.Init.BaudRate = 38400;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : button_Pin */
  GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static void TIM_Init(void)
{
//    /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
//    uwPrescalerValue = (uint32_t)(SystemCoreClock / 100000) - 1;
//
//
//
//    /* Set TIMx instance */
//    TimHandle.Instance = TIM3;
//
//
//
//    /* Initialize TIMx peripheral as follows:
//           + Period = 10000 - 1
//           + Prescaler = (SystemCoreClock/10000) - 1
//           + ClockDivision = 0
//           + Counter direction = Up
//     */
//    TimHandle.Init.Period            = 10 - 1; //10 us
//    TimHandle.Init.Prescaler         = uwPrescalerValue;
//    TimHandle.Init.ClockDivision     = 0;
//    TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
//    TimHandle.Init.RepetitionCounter = 0;
//
//
//
//    if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
//    {
//        /* Initialization Error */
//        Error_Handler();
//    }



	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	    TIM_MasterConfigTypeDef sMasterConfig = {0};
	    /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
	    uwPrescalerValue = (uint32_t)(SystemCoreClock / 100000) - 1;



	    /* Set TIMx instance */
	    TimHandle.Instance = TIM3;



	    /* Initialize TIMx peripheral as follows:
	           + Period = 10000 - 1
	           + Prescaler = (SystemCoreClock/10000) - 1
	           + ClockDivision = 0
	           + Counter direction = Up
	     */
	    TimHandle.Init.Period            = 10 - 1; //10 us
	    TimHandle.Init.Prescaler         = uwPrescalerValue;
	    TimHandle.Init.ClockDivision     = 0;
	    TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	    TimHandle.Init.RepetitionCounter = 0;



	    if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
	    {
	        /* Initialization Error */
	        Error_Handler();
	    }



	    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	    if (HAL_TIM_ConfigClockSource(&TimHandle, &sClockSourceConfig) != HAL_OK)
	    {
	        Error_Handler();
	    }



	    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	    if (HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig) != HAL_OK)
	    {
	        Error_Handler();
	    }
}


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim)
{



    if(htim->Instance==TIM2)
    {
        /* USER CODE BEGIN TIM2_MspInit 0 */



        /* USER CODE END TIM2_MspInit 0 */
        /* TIM2 clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();
        /* USER CODE BEGIN TIM2_MspInit 1 */



        /* NVIC configuration for DMA transfer complete interrupt */
        // HAL_NVIC_SetPriority( TIM2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, 0 );
        HAL_NVIC_EnableIRQ( TIM2_IRQn );



        /* USER CODE END TIM2_MspInit 1 */
    }
    else if(htim->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();



        HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
        HAL_NVIC_DisableIRQ(TIM3_IRQn);
    }
}



void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim)
{



    if(htim->Instance==TIM2)
    {
        /* USER CODE BEGIN TIM2_MspDeInit 0 */



        /* USER CODE END TIM2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM2_CLK_DISABLE();
        /* USER CODE BEGIN TIM2_MspDeInit 1 */
        HAL_NVIC_DisableIRQ( TIM2_IRQn );
        /* USER CODE END TIM2_MspDeInit 1 */
    }
    else if(htim->Instance==TIM3)
    {
        /* USER CODE BEGIN TIM2_MspDeInit 0 */



        /* USER CODE END TIM2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM3_CLK_DISABLE();
        /* USER CODE BEGIN TIM2_MspDeInit 1 */
        HAL_NVIC_DisableIRQ(TIM3_IRQn );
        /* USER CODE END TIM2_MspDeInit 1 */
    }
}

void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);
}

static void generate_Dimmer(void)
{
    /*##-2- Start the TIM Base generation in interrupt mode ####################*/
    /* Start Channel1 */
    if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
    {
        /* Starting Error */
        Error_Handler();
    }



}



static void GPIO_init(void)
{
    GPIO_InitTypeDef GPIOinit = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
    GPIOinit.Pin = LED_Pin;
    GPIOinit.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOinit.Pull = GPIO_NOPULL;
    GPIOinit.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIOinit);



}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3)
    {
        LED_CTRL_vfnPeriodElapsedCallback();
    }
}


void LED_CTRL_vfnPeriodElapsedCallback(void)
{
    switch (led_state) {
        case LED_OFF:
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
            led_delay--;
            if(led_delay == 0)
            {
                led_delay = 3;
                led_state = LED_ON_START;
            }



            break;
        }
        case LED_ON_START:
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
            led_delay--;
            if(led_delay == 0)
            {
                led_delay = 100;
                led_state = LED_DIM_OFF;
            }
            //BSP_LED_Off(LED1);



            break;
        }
        case LED_DIM_OFF:
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
            //BSP_LED_Off(LED1);
            if(led_dim == 0)
            {
                led_state = LED_ON;
            }
            else
            {
                led_state = LED_DIM_ON;
                led_dim--;
            }
            break;
        }
        case LED_DIM_ON:
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
            //BSP_LED_On(LED1);
            led_state = LED_DIM_OFF;
            break;
        }
        case LED_ON:
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
            //BSP_LED_On(LED1);
            HAL_NVIC_DisableIRQ(TIM3_IRQn);
            break;
        }
        default:
        {
            break;
        }
    }
}



static void EXTI15_10_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;



  /* Enable GPIOC clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();



  /* Configure PC.13 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);



  /* Enable and set EXTI line 15_10 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_12)
  {
    /* Toggle LED1 */
//      led_target++;
//      if(led_target == 1)
//      {
//          led_target = 0;
//      }
      LED_CTRL_vfnChangeIntensity();



  }
}



void LED_CTRL_vfnChangeIntensity (void)
{
      led_target = pulsos;
      led_state = LED_OFF;
      led_dim = led_target;
      HAL_NVIC_EnableIRQ(TIM3_IRQn);
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
