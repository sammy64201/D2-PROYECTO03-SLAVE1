/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define TXBUFFERSIZE 4
#define RXBUFFERSIZE 4
uint8_t espacios_disponibles = 8;
uint8_t estado_anterior = 0b0000;
uint8_t espacios_libres = 8;

uint8_t aTxBuffer[TXBUFFERSIZE];
uint8_t aRxBuffer[RXBUFFERSIZE];

const uint8_t segmentos[9] = {
      0b00111111, // 0
      0b00000110, // 1
      0b01011011, // 2
      0b01001111, // 3
      0b01100110, // 4
      0b01101101, // 5
      0b01111101, // 6
      0b00000111, // 7
      0b01111111  // 8
  };

uint8_t sensores_remotos = 0xF0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void actualizarDisplay(uint8_t numero) {
    switch (numero) {
        case 0: // a b c d e f
            HAL_GPIO_WritePin(D_A_GPIO_Port, D_A_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_B_GPIO_Port, D_B_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_C_GPIO_Port, D_C_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_D_GPIO_Port, D_D_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_E_GPIO_Port, D_E_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_F_GPIO_Port, D_F_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_G_GPIO_Port, D_G_Pin, GPIO_PIN_RESET);
            break;

        case 1: // b c
            HAL_GPIO_WritePin(D_A_GPIO_Port, D_A_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_B_GPIO_Port, D_B_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_C_GPIO_Port, D_C_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_D_GPIO_Port, D_D_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_E_GPIO_Port, D_E_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_F_GPIO_Port, D_F_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_G_GPIO_Port, D_G_Pin, GPIO_PIN_RESET);
            break;

        case 2: // a b d e g
            HAL_GPIO_WritePin(D_A_GPIO_Port, D_A_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_B_GPIO_Port, D_B_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_C_GPIO_Port, D_C_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_D_GPIO_Port, D_D_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_E_GPIO_Port, D_E_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_F_GPIO_Port, D_F_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_G_GPIO_Port, D_G_Pin, GPIO_PIN_SET);
            break;

        case 3: // a b c d g
            HAL_GPIO_WritePin(D_A_GPIO_Port, D_A_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_B_GPIO_Port, D_B_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_C_GPIO_Port, D_C_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_D_GPIO_Port, D_D_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_E_GPIO_Port, D_E_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_F_GPIO_Port, D_F_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_G_GPIO_Port, D_G_Pin, GPIO_PIN_SET);
            break;

        case 4: // b c f g
            HAL_GPIO_WritePin(D_A_GPIO_Port, D_A_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_B_GPIO_Port, D_B_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_C_GPIO_Port, D_C_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_D_GPIO_Port, D_D_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_E_GPIO_Port, D_E_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_F_GPIO_Port, D_F_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_G_GPIO_Port, D_G_Pin, GPIO_PIN_SET);
            break;

        case 5: // a c d f g
            HAL_GPIO_WritePin(D_A_GPIO_Port, D_A_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_B_GPIO_Port, D_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_C_GPIO_Port, D_C_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_D_GPIO_Port, D_D_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_E_GPIO_Port, D_E_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_F_GPIO_Port, D_F_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_G_GPIO_Port, D_G_Pin, GPIO_PIN_SET);
            break;

        case 6: // a c d e f g
            HAL_GPIO_WritePin(D_A_GPIO_Port, D_A_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_B_GPIO_Port, D_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_C_GPIO_Port, D_C_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_D_GPIO_Port, D_D_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_E_GPIO_Port, D_E_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_F_GPIO_Port, D_F_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_G_GPIO_Port, D_G_Pin, GPIO_PIN_SET);
            break;

        case 7: // a b c
            HAL_GPIO_WritePin(D_A_GPIO_Port, D_A_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_B_GPIO_Port, D_B_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_C_GPIO_Port, D_C_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_D_GPIO_Port, D_D_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_E_GPIO_Port, D_E_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_F_GPIO_Port, D_F_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_G_GPIO_Port, D_G_Pin, GPIO_PIN_RESET);
            break;

        case 8: // a b c d e f g
            HAL_GPIO_WritePin(D_A_GPIO_Port, D_A_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_B_GPIO_Port, D_B_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_C_GPIO_Port, D_C_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_D_GPIO_Port, D_D_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_E_GPIO_Port, D_E_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_F_GPIO_Port, D_F_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(D_G_GPIO_Port, D_G_Pin, GPIO_PIN_SET);
            break;

        default: // Apagar todos los segmentos
            HAL_GPIO_WritePin(D_A_GPIO_Port, D_A_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_B_GPIO_Port, D_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_C_GPIO_Port, D_C_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_D_GPIO_Port, D_D_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_E_GPIO_Port, D_E_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_F_GPIO_Port, D_F_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(D_G_GPIO_Port, D_G_Pin, GPIO_PIN_RESET);
            break;
    }
}

void actualizarLedsSensor(uint8_t estado) {
    // Sensor 1: l1 (rojo), l2 (verde)
    HAL_GPIO_WritePin(l1_GPIO_Port, l1_Pin, (estado & (1 << 0)) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(l2_GPIO_Port, l2_Pin, (estado & (1 << 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Sensor 2: l3 (rojo), l4 (verde)
    HAL_GPIO_WritePin(l3_GPIO_Port, l3_Pin, (estado & (1 << 1)) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(l4_GPIO_Port, l4_Pin, (estado & (1 << 1)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Sensor 3: l5 (rojo), l6 (verde)
    HAL_GPIO_WritePin(l5_GPIO_Port, l5_Pin, (estado & (1 << 2)) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(l6_GPIO_Port, l6_Pin, (estado & (1 << 2)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Sensor 4: l7 (rojo), l8 (verde)
    HAL_GPIO_WritePin(l7_GPIO_Port, l7_Pin, (estado & (1 << 3)) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(l8_GPIO_Port, l8_Pin, (estado & (1 << 3)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // el maestro te puede hablar
  if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK) {
	  Error_Handler();
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  uint8_t estado_parqueo = 0xFF; // Todos libres inicialmente (1111 1111)

	      // Leer sensores locales (bits 0 a 3)
	      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET) estado_parqueo &= ~(1 << 0); // sensor 1
	      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_RESET) estado_parqueo &= ~(1 << 1); // sensor 2
	      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_RESET) estado_parqueo &= ~(1 << 2); // sensor 3
	      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_RESET) estado_parqueo &= ~(1 << 3); // sensor 4

	      // Aplicar bits 4-7 recibidos por I2C (sensores remotos)
	      estado_parqueo = (estado_parqueo & 0x0F) | (sensores_remotos & 0xF0);

	      // Calcular espacios libres (cantidad de bits en 1)
	      uint8_t espacios = 0;
	      for (int i = 0; i < 8; i++) {
	          if (estado_parqueo & (1 << i)) {
	              espacios++;
	          }
	      }

	      // Mostrar cantidad de espacios libres en el display
	      actualizarDisplay(espacios);
	      actualizarLedsSensor(estado_parqueo);


	      // Preparar dato de salida por I2C al maestro si lo solicita
	      aTxBuffer[0] = estado_parqueo;

	      // Imprimir por UART
	      char msg[64];
	      snprintf(msg, sizeof(msg), "Estado parqueo: 0x%02X - Espacios libres: %d\r\n", estado_parqueo, espacios);
	      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	      HAL_Delay(200);  // Antirebote / actualización periódica




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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 200;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|D_G_Pin|D_F_Pin|D_A_Pin
                          |l1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(l8_GPIO_Port, l8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, l4_Pin|l3_Pin|D_E_Pin|l2_Pin
                          |l7_Pin|l6_Pin|l5_Pin|D_B_Pin
                          |D_D_Pin|D_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin D_G_Pin D_F_Pin D_A_Pin
                           l1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|D_G_Pin|D_F_Pin|D_A_Pin
                          |l1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : l8_Pin */
  GPIO_InitStruct.Pin = l8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(l8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC6 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : l4_Pin l3_Pin D_E_Pin l2_Pin
                           l7_Pin l6_Pin l5_Pin D_B_Pin
                           D_D_Pin D_C_Pin */
  GPIO_InitStruct.Pin = l4_Pin|l3_Pin|D_E_Pin|l2_Pin
                          |l7_Pin|l6_Pin|l5_Pin|D_B_Pin
                          |D_D_Pin|D_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c){
	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle) {

}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
    sensores_remotos = aRxBuffer[0] & 0xF0;

    char msg[32];
    snprintf(msg, sizeof(msg), "Dato recibido: 0x%02X\r\n", aRxBuffer[0]);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}



void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode){
    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        if (HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, aRxBuffer, 1, I2C_FIRST_AND_LAST_FRAME) != HAL_OK){
            Error_Handler();
        }
    } else if (TransferDirection == I2C_DIRECTION_RECEIVE){
        if (HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, aTxBuffer, 1, I2C_FIRST_AND_LAST_FRAME) != HAL_OK){
            Error_Handler();
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
