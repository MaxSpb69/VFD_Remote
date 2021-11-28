/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

typedef struct
{

};

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Pulse_Length 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern uint8_t Brightness;


extern uint8_t VideoBuffer[512];
extern SPI_HandleTypeDef hspi1;
extern volatile uint8_t Key_Table[6];
extern DMA_HandleTypeDef hdma_spi1_tx;
extern TIM_HandleTypeDef htim16;

const uint16_t Key_Ports_Pins[4] = {Key_In1_Pin, Key_In2_Pin, Key_In3_Pin, Key_In4_Pin};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void VFD_Int(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern TIM_HandleTypeDef htim16;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 2 and 3 interrupts.
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

  /* USER CODE END DMA1_Channel2_3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */

  /* USER CODE END TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM16_IRQn 1 */
  VFD_Int();
  /* USER CODE END TIM16_IRQn 1 */
}

/**
  * @brief This function handles USB global Interrupt / USB wake-up interrupt through EXTI line 18.
  */
void USB_IRQHandler(void)
{
  /* USER CODE BEGIN USB_IRQn 0 */

  /* USER CODE END USB_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_IRQn 1 */

  /* USER CODE END USB_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void Key_Scan_Int(void)
{

		static uint8_t key_count = 0x7F;
		static uint8_t Key_Vector[4] = {0,0,0,0,};
		static uint8_t n;

  	key_count = (key_count + 1) & 0x3F;

  	if((key_count & 0x0F) == 0)
  	{
  		n = (key_count & 0x30) >> 4;

  		// ------Просканируем состояние всех 4 кнопок ----------------
  		if(n < 2)
  		{
  			if(HAL_GPIO_ReadPin( Key_In1_GPIO_Port,  Key_Ports_Pins[n]))
  	  			if(Key_Vector[n] > 0)
  	  				Key_Vector[n]--;
  	  			else
  	  				Key_Table[n] = 0;
  	  		else
  	  			if(Key_Vector[n] < 6)
  	  				Key_Vector[n]++;
  	  			else
  	  				Key_Table[n] |= 0x01;
  		}
  		else
  		{
  			if(HAL_GPIO_ReadPin( Key_In3_GPIO_Port,  Key_Ports_Pins[n]))
  	  			if(Key_Vector[n] > 0)
  	  				Key_Vector[n]--;
  	  			else
  	  				Key_Table[n] = 0;
  	  		else
  	  			if(Key_Vector[n] < 6)
  	  				Key_Vector[n]++;
  	  			else
  	  				Key_Table[n] |= 0x01;


  		}
  	}
}





void VFD_Int(void)
{

	static uint8_t  Grid_Count = 0;
	static uint8_t Bright_Count = 0;
	static uint8_t row = 0, key_count = 0x7F;
	static uint8_t Key_Vector[3][2] = {0,0,0,0,0,0};

	uint8_t k;

  	 HAL_GPIO_WritePin(GPIOA, Blk1_Pin, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOA, Blk2_Pin, GPIO_PIN_SET);	// Set all Blk pin to 1
	 HAL_GPIO_WritePin(GPIOA, Gblk_Pin, GPIO_PIN_SET);  // Set Grid Blank pin to 1




	 if(Grid_Count == 2 || Grid_Count == 1)
		 HAL_GPIO_WritePin(GPIOA, Gsin_Pin, GPIO_PIN_SET);	// Если это первые 2 импульса GCLK - GSIN устанавливаем в 1
	 else
		 HAL_GPIO_WritePin(GPIOA, Gsin_Pin, GPIO_PIN_RESET);


	 HAL_GPIO_WritePin(GPIOA, Gclk_Pin, GPIO_PIN_RESET);

	 HAL_GPIO_WritePin(GPIOA, Gclk_Pin, GPIO_PIN_SET);	// Формируем отрицательный импульс GCLK
	 HAL_GPIO_WritePin(GPIOA, Gsin_Pin, GPIO_PIN_RESET);


	 HAL_GPIO_WritePin(GPIOA, Latch_Pin, GPIO_PIN_SET);

	 HAL_GPIO_WritePin(GPIOA, Latch_Pin, GPIO_PIN_RESET); // Формируем положительный импульс Latch



	 if(Bright_Count <= Brightness)
		 HAL_GPIO_WritePin(GPIOA, Gblk_Pin, GPIO_PIN_RESET);  // Set Grid Blank pin to 0
	// HAL_GPIO_WritePin(GPIOA, Gblk_Pin, GPIO_PIN_SET);


	 if(Grid_Count & 0x01)														// В зависимости от номера сетки включаем Blk1 или Blk2
		 HAL_GPIO_WritePin(GPIOA, Blk1_Pin, GPIO_PIN_RESET);
	 else
		 HAL_GPIO_WritePin(GPIOA, Blk2_Pin, GPIO_PIN_RESET);




	 HAL_SPI_Transmit_DMA(&hspi1, (VideoBuffer + (Grid_Count * 8)), 8); // Вывод в SPI 8 байт (64 бита по 32 бита в 2 стрках)

	 Grid_Count = (Grid_Count + 1) & 0x3F;
	 if(Grid_Count == 0)
		 Bright_Count  = (Bright_Count + 1) & 0x03;

	 // Закончили с дисплеем, теперь посканируем кнопки

	 Key_Scan_Int();

}




/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
