/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "string.h"

/* USER CODE BEGIN Includes */
#define MPU6050ADRESS 0x68
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t buffrec[1];
char* bufftr;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void TransferData(char* Data);
void Banh1QuayThuan(void);
void Banh1QuayNghich(void);
void Banh2QuayThuan(void);
void Banh2QuayNghich(void);
void MotorStatus(void);
uint8_t temp_uart;
uint8_t i2cBuf[8];
uint16_t ax, ay, az;
float Xaccel, Yaccel, Zaccel;
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();

    /* clear data of arry buffrec[] */
    for (uint8_t i=0; i < 1; i++){
        buffrec[i] = 0;
    }
    temp_uart = 0;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    
  /* USER CODE BEGIN 2 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  /* USER CODE END 2 */

    /* USER CODE BEGIN 3 */
    for (uint8_t i=0; i<255; i++){
        if (HAL_I2C_IsDeviceReady(&hi2c1, i, 1, 10) == HAL_OK){
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            break;
        }
    }
    i2cBuf[0] = 28;
    i2cBuf[1] = 0x08;
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050ADRESS, i2cBuf, 2, 10);
    
    i2cBuf[0] = 28;
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050ADRESS, i2cBuf, 1, 10);
    
    i2cBuf[1] = 0x00;
    HAL_I2C_Master_Receive(&hi2c1, MPU6050ADRESS, &i2cBuf[1], 1, 10);
    
    /* USER CODE END 3 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
    if (temp_uart == 1){
        MotorStatus();
        temp_uart = 0;
    }
    
    i2cBuf[0] = 0x3B;
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050ADRESS, i2cBuf, 1, 10);
    
    // read data
    i2cBuf[1] = 0x00;
    HAL_I2C_Master_Receive(&hi2c1, MPU6050ADRESS, &i2cBuf[1], 6, 10);
    
    ax = -(i2cBuf[1]<<8 | i2cBuf[2]);
    ay = -(i2cBuf[3]<<8 | i2cBuf[4]);
    az =  (i2cBuf[5]<<8 | i2cBuf[6]);
    
    Xaccel = ax/8192.0;
    Yaccel = ay/8192.0;
    Zaccel = az/8192.0;
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 80;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim1);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);

  HAL_TIM_MspPostInit(&htim1);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, INT1_Pin|INT2_Pin|INT4_Pin|INT5_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RIGHT_Pin LEFT_Pin */
  GPIO_InitStruct.Pin = RIGHT_Pin|LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin INT4_Pin INT5_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin|INT4_Pin|INT5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
void TransferData(char* Data){
    bufftr = Data;
}
/* USER CODE END 4 */


/* USER CODE BEGIN 5 */
void Banh1QuayThuan(void){
    HAL_GPIO_WritePin(GPIOB, INT1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, INT2_Pin, GPIO_PIN_RESET);
}
/* USER CODE END 5 */

/* USER CODE BEGIN 6 */
void Banh1QuayNghich(void){
    HAL_GPIO_WritePin(GPIOB, INT1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, INT2_Pin, GPIO_PIN_SET);
}
/* USER CODE END 6 */

/* USER CODE BEGIN 7 */
void Banh2QuayThuan(void){
    HAL_GPIO_WritePin(GPIOB, INT4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, INT5_Pin, GPIO_PIN_RESET);
}
/* USER CODE END 7 */

/* USER CODE BEGIN 8 */
void Banh2QuayNghich(void){
    HAL_GPIO_WritePin(GPIOB, INT4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, INT5_Pin, GPIO_PIN_SET);
}
/* USER CODE END 8 */

/* USER CODE END 9 */
void MotorStatus(void){
    switch(buffrec[0]){
        /* Motor Speed */
        case 48:{TIM1->CCR1 = 0; TIM1->CCR4 = 0; break;}    /* 0% */
        case 49:{TIM1->CCR1 = 10; TIM1->CCR4 = 10; break;}    /* 10% */
        case 50:{TIM1->CCR1 = 20; TIM1->CCR4 = 20; break;}    /* 20% */
        case 51:{TIM1->CCR1 = 30; TIM1->CCR4 = 30; break;}    /* 30% */
        case 52:{TIM1->CCR1 = 40; TIM1->CCR4 = 40; break;}    /* 40% */
        case 53:{TIM1->CCR1 = 50; TIM1->CCR4 = 50; break;}    /* 50% */
        case 54:{TIM1->CCR1 = 60; TIM1->CCR4 = 60; break;}    /* 60% */
        case 55:{TIM1->CCR1 = 70; TIM1->CCR4 = 70; break;}    /* 70% */
        case 56:{TIM1->CCR1 = 80; TIM1->CCR4 = 80; break;}    /* 80% */
        case 57:{TIM1->CCR1 = 90; TIM1->CCR4 = 90; break;}    /* 90% */
        case 113:{TIM1->CCR1 = 100; TIM1->CCR4 = 100; break;}    /* 100% */
        /* END Motor Speed */
        
        /* Move Status */
        case 70:{Banh1QuayThuan(); Banh2QuayThuan(); break;}   /* Forward */
        case 66:{Banh1QuayNghich(); Banh2QuayNghich(); break;}   /* Back */
        case 76:{; break;}   /* Left */
        case 82:{; break;}   /* Right */
        case 71:{; break;}   /* Forward Left */
        case 73:{; break;}   /* Forward Right */
        case 72:{; break;}   /* Back Left */
        case 74:{; break;}   /* Back Right */
        case 83:{; break;}   /* Stop */
        /* END Move Status */
    }
}
/* USER CODE END 9 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
