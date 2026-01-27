/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f7xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_spi2_tx;

extern DMA_HandleTypeDef hdma_spi2_rx;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
  * @brief ADC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hadc->Instance==ADC3)
  {
    /* USER CODE BEGIN ADC3_MspInit 0 */

    /* USER CODE END ADC3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC3 GPIO Configuration
    PF7     ------> ADC3_IN5
    PF6     ------> ADC3_IN4
    PF10     ------> ADC3_IN8
    PF9     ------> ADC3_IN7
    PF8     ------> ADC3_IN6
    PA0/WKUP     ------> ADC3_IN0
    */
    GPIO_InitStruct.Pin = ARDUINO_A4_Pin|ARDUINO_A5_Pin|ARDUINO_A1_Pin|ARDUINO_A2_Pin
                          |ARDUINO_A3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ARDUINO_A0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ARDUINO_A0_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN ADC3_MspInit 1 */

    /* USER CODE END ADC3_MspInit 1 */

  }

}

/**
  * @brief ADC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC3)
  {
    /* USER CODE BEGIN ADC3_MspDeInit 0 */

    /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();

    /**ADC3 GPIO Configuration
    PF7     ------> ADC3_IN5
    PF6     ------> ADC3_IN4
    PF10     ------> ADC3_IN8
    PF9     ------> ADC3_IN7
    PF8     ------> ADC3_IN6
    PA0/WKUP     ------> ADC3_IN0
    */
    HAL_GPIO_DeInit(GPIOF, ARDUINO_A4_Pin|ARDUINO_A5_Pin|ARDUINO_A1_Pin|ARDUINO_A2_Pin
                          |ARDUINO_A3_Pin);

    HAL_GPIO_DeInit(ARDUINO_A0_GPIO_Port, ARDUINO_A0_Pin);

    /* USER CODE BEGIN ADC3_MspDeInit 1 */

    /* USER CODE END ADC3_MspDeInit 1 */
  }

}

/**
  * @brief CRC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hcrc: CRC handle pointer
  * @retval None
  */
void HAL_CRC_MspInit(CRC_HandleTypeDef* hcrc)
{
  if(hcrc->Instance==CRC)
  {
    /* USER CODE BEGIN CRC_MspInit 0 */

    /* USER CODE END CRC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CRC_CLK_ENABLE();
    /* USER CODE BEGIN CRC_MspInit 1 */

    /* USER CODE END CRC_MspInit 1 */

  }

}

/**
  * @brief CRC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hcrc: CRC handle pointer
  * @retval None
  */
void HAL_CRC_MspDeInit(CRC_HandleTypeDef* hcrc)
{
  if(hcrc->Instance==CRC)
  {
    /* USER CODE BEGIN CRC_MspDeInit 0 */

    /* USER CODE END CRC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CRC_CLK_DISABLE();
    /* USER CODE BEGIN CRC_MspDeInit 1 */

    /* USER CODE END CRC_MspDeInit 1 */
  }

}

/**
  * @brief DCMI MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hdcmi: DCMI handle pointer
  * @retval None
  */
void HAL_DCMI_MspInit(DCMI_HandleTypeDef* hdcmi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hdcmi->Instance==DCMI)
  {
    /* USER CODE BEGIN DCMI_MspInit 0 */

    /* USER CODE END DCMI_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DCMI_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**DCMI GPIO Configuration
    PE5     ------> DCMI_D6
    PE6     ------> DCMI_D7
    PD3     ------> DCMI_D5
    PG9     ------> DCMI_VSYNC
    PH14     ------> DCMI_D4
    PH12     ------> DCMI_D3
    PA4     ------> DCMI_HSYNC
    PH9     ------> DCMI_D0
    PH11     ------> DCMI_D2
    PA6     ------> DCMI_PIXCLK
    PH10     ------> DCMI_D1
    */
    GPIO_InitStruct.Pin = DCMI_D6_Pin|DCMI_D7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DCMI_D5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(DCMI_D5_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DCMI_VSYNC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(DCMI_VSYNC_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DCMI_D4_Pin|DCMI_D3_Pin|DCMI_D0_Pin|DCMI_D2_Pin
                          |DCMI_D1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DCMI_HSYNC_Pin|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN DCMI_MspInit 1 */

    /* USER CODE END DCMI_MspInit 1 */

  }

}

/**
  * @brief DCMI MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hdcmi: DCMI handle pointer
  * @retval None
  */
void HAL_DCMI_MspDeInit(DCMI_HandleTypeDef* hdcmi)
{
  if(hdcmi->Instance==DCMI)
  {
    /* USER CODE BEGIN DCMI_MspDeInit 0 */

    /* USER CODE END DCMI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DCMI_CLK_DISABLE();

    /**DCMI GPIO Configuration
    PE5     ------> DCMI_D6
    PE6     ------> DCMI_D7
    PD3     ------> DCMI_D5
    PG9     ------> DCMI_VSYNC
    PH14     ------> DCMI_D4
    PH12     ------> DCMI_D3
    PA4     ------> DCMI_HSYNC
    PH9     ------> DCMI_D0
    PH11     ------> DCMI_D2
    PA6     ------> DCMI_PIXCLK
    PH10     ------> DCMI_D1
    */
    HAL_GPIO_DeInit(GPIOE, DCMI_D6_Pin|DCMI_D7_Pin);

    HAL_GPIO_DeInit(DCMI_D5_GPIO_Port, DCMI_D5_Pin);

    HAL_GPIO_DeInit(DCMI_VSYNC_GPIO_Port, DCMI_VSYNC_Pin);

    HAL_GPIO_DeInit(GPIOH, DCMI_D4_Pin|DCMI_D3_Pin|DCMI_D0_Pin|DCMI_D2_Pin
                          |DCMI_D1_Pin);

    HAL_GPIO_DeInit(GPIOA, DCMI_HSYNC_Pin|GPIO_PIN_6);

    /* USER CODE BEGIN DCMI_MspDeInit 1 */

    /* USER CODE END DCMI_MspDeInit 1 */
  }

}

/**
  * @brief DMA2D MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hdma2d: DMA2D handle pointer
  * @retval None
  */
void HAL_DMA2D_MspInit(DMA2D_HandleTypeDef* hdma2d)
{
  if(hdma2d->Instance==DMA2D)
  {
    /* USER CODE BEGIN DMA2D_MspInit 0 */

    /* USER CODE END DMA2D_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DMA2D_CLK_ENABLE();
    /* DMA2D interrupt Init */
    HAL_NVIC_SetPriority(DMA2D_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2D_IRQn);
    /* USER CODE BEGIN DMA2D_MspInit 1 */

    /* USER CODE END DMA2D_MspInit 1 */

  }

}

/**
  * @brief DMA2D MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hdma2d: DMA2D handle pointer
  * @retval None
  */
void HAL_DMA2D_MspDeInit(DMA2D_HandleTypeDef* hdma2d)
{
  if(hdma2d->Instance==DMA2D)
  {
    /* USER CODE BEGIN DMA2D_MspDeInit 0 */

    /* USER CODE END DMA2D_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DMA2D_CLK_DISABLE();

    /* DMA2D interrupt DeInit */
    HAL_NVIC_DisableIRQ(DMA2D_IRQn);
    /* USER CODE BEGIN DMA2D_MspDeInit 1 */

    /* USER CODE END DMA2D_MspDeInit 1 */
  }

}

/**
  * @brief LTDC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hltdc: LTDC handle pointer
  * @retval None
  */
void HAL_LTDC_MspInit(LTDC_HandleTypeDef* hltdc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hltdc->Instance==LTDC)
  {
    /* USER CODE BEGIN LTDC_MspInit 0 */

    /* USER CODE END LTDC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_LTDC_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOJ_CLK_ENABLE();
    __HAL_RCC_GPIOK_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    /**LTDC GPIO Configuration
    PE4     ------> LTDC_B0
    PJ13     ------> LTDC_B1
    PK7     ------> LTDC_DE
    PK6     ------> LTDC_B7
    PK5     ------> LTDC_B6
    PG12     ------> LTDC_B4
    PJ14     ------> LTDC_B2
    PI10     ------> LTDC_HSYNC
    PK4     ------> LTDC_B5
    PJ15     ------> LTDC_B3
    PI9     ------> LTDC_VSYNC
    PK1     ------> LTDC_G6
    PK2     ------> LTDC_G7
    PI15     ------> LTDC_R0
    PJ11     ------> LTDC_G4
    PK0     ------> LTDC_G5
    PI14     ------> LTDC_CLK
    PJ8     ------> LTDC_G1
    PJ10     ------> LTDC_G3
    PJ7     ------> LTDC_G0
    PJ9     ------> LTDC_G2
    PJ6     ------> LTDC_R7
    PJ4     ------> LTDC_R5
    PJ5     ------> LTDC_R6
    PJ3     ------> LTDC_R4
    PJ2     ------> LTDC_R3
    PJ0     ------> LTDC_R1
    PJ1     ------> LTDC_R2
    */
    GPIO_InitStruct.Pin = LCD_B0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(LCD_B0_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LCD_B1_Pin|LCD_B2_Pin|LCD_B3_Pin|LCD_G4_Pin
                          |LCD_G1_Pin|LCD_G3_Pin|LCD_G0_Pin|LCD_G2_Pin
                          |LCD_R7_Pin|LCD_R5_Pin|LCD_R6_Pin|LCD_R4_Pin
                          |LCD_R3_Pin|LCD_R1_Pin|LCD_R2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LCD_DE_Pin|LCD_B7_Pin|LCD_B6_Pin|LCD_B5_Pin
                          |LCD_G6_Pin|LCD_G7_Pin|LCD_G5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LCD_B4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
    HAL_GPIO_Init(LCD_B4_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LCD_HSYNC_Pin|LCD_VSYNC_Pin|LCD_R0_Pin|LCD_CLK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    /* LTDC interrupt Init */
    HAL_NVIC_SetPriority(LTDC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(LTDC_IRQn);
    /* USER CODE BEGIN LTDC_MspInit 1 */

    /* USER CODE END LTDC_MspInit 1 */

  }

}

/**
  * @brief LTDC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hltdc: LTDC handle pointer
  * @retval None
  */
void HAL_LTDC_MspDeInit(LTDC_HandleTypeDef* hltdc)
{
  if(hltdc->Instance==LTDC)
  {
    /* USER CODE BEGIN LTDC_MspDeInit 0 */

    /* USER CODE END LTDC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_LTDC_CLK_DISABLE();

    /**LTDC GPIO Configuration
    PE4     ------> LTDC_B0
    PJ13     ------> LTDC_B1
    PK7     ------> LTDC_DE
    PK6     ------> LTDC_B7
    PK5     ------> LTDC_B6
    PG12     ------> LTDC_B4
    PJ14     ------> LTDC_B2
    PI10     ------> LTDC_HSYNC
    PK4     ------> LTDC_B5
    PJ15     ------> LTDC_B3
    PI9     ------> LTDC_VSYNC
    PK1     ------> LTDC_G6
    PK2     ------> LTDC_G7
    PI15     ------> LTDC_R0
    PJ11     ------> LTDC_G4
    PK0     ------> LTDC_G5
    PI14     ------> LTDC_CLK
    PJ8     ------> LTDC_G1
    PJ10     ------> LTDC_G3
    PJ7     ------> LTDC_G0
    PJ9     ------> LTDC_G2
    PJ6     ------> LTDC_R7
    PJ4     ------> LTDC_R5
    PJ5     ------> LTDC_R6
    PJ3     ------> LTDC_R4
    PJ2     ------> LTDC_R3
    PJ0     ------> LTDC_R1
    PJ1     ------> LTDC_R2
    */
    HAL_GPIO_DeInit(LCD_B0_GPIO_Port, LCD_B0_Pin);

    HAL_GPIO_DeInit(GPIOJ, LCD_B1_Pin|LCD_B2_Pin|LCD_B3_Pin|LCD_G4_Pin
                          |LCD_G1_Pin|LCD_G3_Pin|LCD_G0_Pin|LCD_G2_Pin
                          |LCD_R7_Pin|LCD_R5_Pin|LCD_R6_Pin|LCD_R4_Pin
                          |LCD_R3_Pin|LCD_R1_Pin|LCD_R2_Pin);

    HAL_GPIO_DeInit(GPIOK, LCD_DE_Pin|LCD_B7_Pin|LCD_B6_Pin|LCD_B5_Pin
                          |LCD_G6_Pin|LCD_G7_Pin|LCD_G5_Pin);

    HAL_GPIO_DeInit(LCD_B4_GPIO_Port, LCD_B4_Pin);

    HAL_GPIO_DeInit(GPIOI, LCD_HSYNC_Pin|LCD_VSYNC_Pin|LCD_R0_Pin|LCD_CLK_Pin);

    /* LTDC interrupt DeInit */
    HAL_NVIC_DisableIRQ(LTDC_IRQn);
    /* USER CODE BEGIN LTDC_MspDeInit 1 */

    /* USER CODE END LTDC_MspDeInit 1 */
  }

}

/**
  * @brief QSPI MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hqspi: QSPI handle pointer
  * @retval None
  */
void HAL_QSPI_MspInit(QSPI_HandleTypeDef* hqspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hqspi->Instance==QUADSPI)
  {
    /* USER CODE BEGIN QUADSPI_MspInit 0 */

    /* USER CODE END QUADSPI_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_QSPI_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**QUADSPI GPIO Configuration
    PE2     ------> QUADSPI_BK1_IO2
    PB6     ------> QUADSPI_BK1_NCS
    PB2     ------> QUADSPI_CLK
    PD12     ------> QUADSPI_BK1_IO1
    PD13     ------> QUADSPI_BK1_IO3
    PD11     ------> QUADSPI_BK1_IO0
    */
    GPIO_InitStruct.Pin = QSPI_D2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(QSPI_D2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = QSPI_NCS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
    HAL_GPIO_Init(QSPI_NCS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = QSPI_D1_Pin|QSPI_D3_Pin|QSPI_D0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USER CODE BEGIN QUADSPI_MspInit 1 */

    /* USER CODE END QUADSPI_MspInit 1 */

  }

}

/**
  * @brief QSPI MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hqspi: QSPI handle pointer
  * @retval None
  */
void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef* hqspi)
{
  if(hqspi->Instance==QUADSPI)
  {
    /* USER CODE BEGIN QUADSPI_MspDeInit 0 */

    /* USER CODE END QUADSPI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_QSPI_CLK_DISABLE();

    /**QUADSPI GPIO Configuration
    PE2     ------> QUADSPI_BK1_IO2
    PB6     ------> QUADSPI_BK1_NCS
    PB2     ------> QUADSPI_CLK
    PD12     ------> QUADSPI_BK1_IO1
    PD13     ------> QUADSPI_BK1_IO3
    PD11     ------> QUADSPI_BK1_IO0
    */
    HAL_GPIO_DeInit(QSPI_D2_GPIO_Port, QSPI_D2_Pin);

    HAL_GPIO_DeInit(GPIOB, QSPI_NCS_Pin|GPIO_PIN_2);

    HAL_GPIO_DeInit(GPIOD, QSPI_D1_Pin|QSPI_D3_Pin|QSPI_D0_Pin);

    /* USER CODE BEGIN QUADSPI_MspDeInit 1 */

    /* USER CODE END QUADSPI_MspDeInit 1 */
  }

}

/**
  * @brief SD MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hsd: SD handle pointer
  * @retval None
  */
void HAL_SD_MspInit(SD_HandleTypeDef* hsd)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hsd->Instance==SDMMC1)
  {
    /* USER CODE BEGIN SDMMC1_MspInit 0 */

    /* USER CODE END SDMMC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SDMMC1_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**SDMMC1 GPIO Configuration
    PC12     ------> SDMMC1_CK
    PC11     ------> SDMMC1_D3
    PC10     ------> SDMMC1_D2
    PD2     ------> SDMMC1_CMD
    PC9     ------> SDMMC1_D1
    PC8     ------> SDMMC1_D0
    */
    GPIO_InitStruct.Pin = SDMMC_CK_Pin|SDMMC_D3_Pin|SDMMC_D2_Pin|GPIO_PIN_9
                          |GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SDMMC_CMD_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(SDMMC_CMD_GPIO_Port, &GPIO_InitStruct);

    /* SDMMC1 interrupt Init */
    HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
    /* USER CODE BEGIN SDMMC1_MspInit 1 */

    /* USER CODE END SDMMC1_MspInit 1 */

  }

}

/**
  * @brief SD MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hsd: SD handle pointer
  * @retval None
  */
void HAL_SD_MspDeInit(SD_HandleTypeDef* hsd)
{
  if(hsd->Instance==SDMMC1)
  {
    /* USER CODE BEGIN SDMMC1_MspDeInit 0 */

    /* USER CODE END SDMMC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SDMMC1_CLK_DISABLE();

    /**SDMMC1 GPIO Configuration
    PC12     ------> SDMMC1_CK
    PC11     ------> SDMMC1_D3
    PC10     ------> SDMMC1_D2
    PD2     ------> SDMMC1_CMD
    PC9     ------> SDMMC1_D1
    PC8     ------> SDMMC1_D0
    */
    HAL_GPIO_DeInit(GPIOC, SDMMC_CK_Pin|SDMMC_D3_Pin|SDMMC_D2_Pin|GPIO_PIN_9
                          |GPIO_PIN_8);

    HAL_GPIO_DeInit(SDMMC_CMD_GPIO_Port, SDMMC_CMD_Pin);

    /* SDMMC1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(SDMMC1_IRQn);
    /* USER CODE BEGIN SDMMC1_MspDeInit 1 */

    /* USER CODE END SDMMC1_MspDeInit 1 */
  }

}

/**
  * @brief SPDIFRX MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hspdifrx: SPDIFRX handle pointer
  * @retval None
  */
void HAL_SPDIFRX_MspInit(SPDIFRX_HandleTypeDef* hspdifrx)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hspdifrx->Instance==SPDIFRX)
  {
    /* USER CODE BEGIN SPDIFRX_MspInit 0 */

    /* USER CODE END SPDIFRX_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPDIFRX;
    PeriphClkInitStruct.PLLI2S.PLLI2SN = 100;
    PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLP_DIV2;
    PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
    PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
    PeriphClkInitStruct.PLLI2SDivQ = 1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SPDIFRX_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**SPDIFRX GPIO Configuration
    PD7     ------> SPDIFRX_IN0
    */
    GPIO_InitStruct.Pin = SPDIF_RX0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
    HAL_GPIO_Init(SPDIF_RX0_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN SPDIFRX_MspInit 1 */

    /* USER CODE END SPDIFRX_MspInit 1 */

  }

}

/**
  * @brief SPDIFRX MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hspdifrx: SPDIFRX handle pointer
  * @retval None
  */
void HAL_SPDIFRX_MspDeInit(SPDIFRX_HandleTypeDef* hspdifrx)
{
  if(hspdifrx->Instance==SPDIFRX)
  {
    /* USER CODE BEGIN SPDIFRX_MspDeInit 0 */

    /* USER CODE END SPDIFRX_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPDIFRX_CLK_DISABLE();

    /**SPDIFRX GPIO Configuration
    PD7     ------> SPDIFRX_IN0
    */
    HAL_GPIO_DeInit(SPDIF_RX0_GPIO_Port, SPDIF_RX0_Pin);

    /* USER CODE BEGIN SPDIFRX_MspDeInit 1 */

    /* USER CODE END SPDIFRX_MspDeInit 1 */
  }

}

/**
  * @brief SPI MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI2)
  {
    /* USER CODE BEGIN SPI2_MspInit 0 */

    /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PI1     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ARDUINO_MISO_D12_Pin|ARDUINO_MOSI_PWM_D11_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SPI2 DMA Init */
    /* SPI2_TX Init */
    hdma_spi2_tx.Instance = DMA1_Stream4;
    hdma_spi2_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi2_tx.Init.Mode = DMA_NORMAL;
    hdma_spi2_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi,hdmatx,hdma_spi2_tx);

    /* SPI2_RX Init */
    hdma_spi2_rx.Instance = DMA1_Stream3;
    hdma_spi2_rx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi2_rx.Init.Mode = DMA_NORMAL;
    hdma_spi2_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi,hdmarx,hdma_spi2_rx);

    /* USER CODE BEGIN SPI2_MspInit 1 */

    /* USER CODE END SPI2_MspInit 1 */

  }

}

/**
  * @brief SPI MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI2)
  {
    /* USER CODE BEGIN SPI2_MspDeInit 0 */

    /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PI1     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    HAL_GPIO_DeInit(GPIOI, GPIO_PIN_1);

    HAL_GPIO_DeInit(GPIOB, ARDUINO_MISO_D12_Pin|ARDUINO_MOSI_PWM_D11_Pin);

    /* SPI2 DMA DeInit */
    HAL_DMA_DeInit(hspi->hdmatx);
    HAL_DMA_DeInit(hspi->hdmarx);
    /* USER CODE BEGIN SPI2_MspDeInit 1 */

    /* USER CODE END SPI2_MspDeInit 1 */
  }

}

/**
  * @brief UART MSP Initialization
  * This function configures the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(huart->Instance==USART6)
  {
    /* USER CODE BEGIN USART6_MspInit 0 */

    /* USER CODE END USART6_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
    PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN USART6_MspInit 1 */

    /* USER CODE END USART6_MspInit 1 */

  }

}

/**
  * @brief UART MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART6)
  {
    /* USER CODE BEGIN USART6_MspDeInit 0 */

    /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_7);

    /* USER CODE BEGIN USART6_MspDeInit 1 */

    /* USER CODE END USART6_MspDeInit 1 */
  }

}

static uint32_t FMC_Initialized = 0;

static void HAL_FMC_MspInit(void){
  /* USER CODE BEGIN FMC_MspInit 0 */

  /* USER CODE END FMC_MspInit 0 */
  GPIO_InitTypeDef GPIO_InitStruct ={0};
  if (FMC_Initialized) {
    return;
  }
  FMC_Initialized = 1;

  /* Peripheral clock enable */
  __HAL_RCC_FMC_CLK_ENABLE();

  /** FMC GPIO Configuration
  PE1   ------> FMC_NBL1
  PE0   ------> FMC_NBL0
  PG15   ------> FMC_SDNCAS
  PD0   ------> FMC_D2
  PD1   ------> FMC_D3
  PF0   ------> FMC_A0
  PF1   ------> FMC_A1
  PF2   ------> FMC_A2
  PF3   ------> FMC_A3
  PG8   ------> FMC_SDCLK
  PF4   ------> FMC_A4
  PH5   ------> FMC_SDNWE
  PH3   ------> FMC_SDNE0
  PF5   ------> FMC_A5
  PD15   ------> FMC_D1
  PD10   ------> FMC_D15
  PC3   ------> FMC_SDCKE0
  PD14   ------> FMC_D0
  PD9   ------> FMC_D14
  PD8   ------> FMC_D13
  PF12   ------> FMC_A6
  PG1   ------> FMC_A11
  PF15   ------> FMC_A9
  PF13   ------> FMC_A7
  PG0   ------> FMC_A10
  PE8   ------> FMC_D5
  PG5   ------> FMC_BA1
  PG4   ------> FMC_BA0
  PF14   ------> FMC_A8
  PF11   ------> FMC_SDNRAS
  PE9   ------> FMC_D6
  PE11   ------> FMC_D8
  PE14   ------> FMC_D11
  PE7   ------> FMC_D4
  PE10   ------> FMC_D7
  PE12   ------> FMC_D9
  PE15   ------> FMC_D12
  PE13   ------> FMC_D10
  */
  GPIO_InitStruct.Pin = FMC_NBL1_Pin|FMC_NBL0_Pin|FMC_D5_Pin|FMC_D6_Pin
                          |FMC_D8_Pin|FMC_D11_Pin|FMC_D4_Pin|FMC_D7_Pin
                          |FMC_D9_Pin|FMC_D12_Pin|FMC_D10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FMC_SDNCAS_Pin|FMC_SDCLK_Pin|FMC_A11_Pin|FMC_A10_Pin
                          |FMC_BA1_Pin|FMC_BA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FMC_D2_Pin|FMC_D3_Pin|FMC_D1_Pin|FMC_D15_Pin
                          |FMC_D0_Pin|FMC_D14_Pin|FMC_D13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FMC_A0_Pin|FMC_A1_Pin|FMC_A2_Pin|FMC_A3_Pin
                          |FMC_A4_Pin|FMC_A5_Pin|FMC_A6_Pin|FMC_A9_Pin
                          |FMC_A7_Pin|FMC_A8_Pin|FMC_SDNRAS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FMC_SDNME_Pin|FMC_SDNE0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FMC_SDCKE0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(FMC_SDCKE0_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN FMC_MspInit 1 */

  /* USER CODE END FMC_MspInit 1 */
}

void HAL_SDRAM_MspInit(SDRAM_HandleTypeDef* hsdram){
  /* USER CODE BEGIN SDRAM_MspInit 0 */

  /* USER CODE END SDRAM_MspInit 0 */
  HAL_FMC_MspInit();
  /* USER CODE BEGIN SDRAM_MspInit 1 */

  /* USER CODE END SDRAM_MspInit 1 */
}

static uint32_t FMC_DeInitialized = 0;

static void HAL_FMC_MspDeInit(void){
  /* USER CODE BEGIN FMC_MspDeInit 0 */

  /* USER CODE END FMC_MspDeInit 0 */
  if (FMC_DeInitialized) {
    return;
  }
  FMC_DeInitialized = 1;
  /* Peripheral clock enable */
  __HAL_RCC_FMC_CLK_DISABLE();

  /** FMC GPIO Configuration
  PE1   ------> FMC_NBL1
  PE0   ------> FMC_NBL0
  PG15   ------> FMC_SDNCAS
  PD0   ------> FMC_D2
  PD1   ------> FMC_D3
  PF0   ------> FMC_A0
  PF1   ------> FMC_A1
  PF2   ------> FMC_A2
  PF3   ------> FMC_A3
  PG8   ------> FMC_SDCLK
  PF4   ------> FMC_A4
  PH5   ------> FMC_SDNWE
  PH3   ------> FMC_SDNE0
  PF5   ------> FMC_A5
  PD15   ------> FMC_D1
  PD10   ------> FMC_D15
  PC3   ------> FMC_SDCKE0
  PD14   ------> FMC_D0
  PD9   ------> FMC_D14
  PD8   ------> FMC_D13
  PF12   ------> FMC_A6
  PG1   ------> FMC_A11
  PF15   ------> FMC_A9
  PF13   ------> FMC_A7
  PG0   ------> FMC_A10
  PE8   ------> FMC_D5
  PG5   ------> FMC_BA1
  PG4   ------> FMC_BA0
  PF14   ------> FMC_A8
  PF11   ------> FMC_SDNRAS
  PE9   ------> FMC_D6
  PE11   ------> FMC_D8
  PE14   ------> FMC_D11
  PE7   ------> FMC_D4
  PE10   ------> FMC_D7
  PE12   ------> FMC_D9
  PE15   ------> FMC_D12
  PE13   ------> FMC_D10
  */
  HAL_GPIO_DeInit(GPIOE, FMC_NBL1_Pin|FMC_NBL0_Pin|FMC_D5_Pin|FMC_D6_Pin
                          |FMC_D8_Pin|FMC_D11_Pin|FMC_D4_Pin|FMC_D7_Pin
                          |FMC_D9_Pin|FMC_D12_Pin|FMC_D10_Pin);

  HAL_GPIO_DeInit(GPIOG, FMC_SDNCAS_Pin|FMC_SDCLK_Pin|FMC_A11_Pin|FMC_A10_Pin
                          |FMC_BA1_Pin|FMC_BA0_Pin);

  HAL_GPIO_DeInit(GPIOD, FMC_D2_Pin|FMC_D3_Pin|FMC_D1_Pin|FMC_D15_Pin
                          |FMC_D0_Pin|FMC_D14_Pin|FMC_D13_Pin);

  HAL_GPIO_DeInit(GPIOF, FMC_A0_Pin|FMC_A1_Pin|FMC_A2_Pin|FMC_A3_Pin
                          |FMC_A4_Pin|FMC_A5_Pin|FMC_A6_Pin|FMC_A9_Pin
                          |FMC_A7_Pin|FMC_A8_Pin|FMC_SDNRAS_Pin);

  HAL_GPIO_DeInit(GPIOH, FMC_SDNME_Pin|FMC_SDNE0_Pin);

  HAL_GPIO_DeInit(FMC_SDCKE0_GPIO_Port, FMC_SDCKE0_Pin);

  /* USER CODE BEGIN FMC_MspDeInit 1 */

  /* USER CODE END FMC_MspDeInit 1 */
}

void HAL_SDRAM_MspDeInit(SDRAM_HandleTypeDef* hsdram){
  /* USER CODE BEGIN SDRAM_MspDeInit 0 */

  /* USER CODE END SDRAM_MspDeInit 0 */
  HAL_FMC_MspDeInit();
  /* USER CODE BEGIN SDRAM_MspDeInit 1 */

  /* USER CODE END SDRAM_MspDeInit 1 */
}

extern DMA_HandleTypeDef hdma_sai2_a;

static uint32_t SAI2_client =0;

void HAL_SAI_MspInit(SAI_HandleTypeDef* hsai)
{

  GPIO_InitTypeDef GPIO_InitStruct;
/* SAI2 */
    if(hsai->Instance==SAI2_Block_A)
    {
    /* Peripheral clock enable */
    if (SAI2_client == 0)
    {
       __HAL_RCC_SAI2_CLK_ENABLE();
    }
    SAI2_client ++;

    /**SAI2_A_Block_A GPIO Configuration
    PI4     ------> SAI2_MCLK_A
    PI5     ------> SAI2_SCK_A
    PI7     ------> SAI2_FS_A
    PI6     ------> SAI2_SD_A
    */
    GPIO_InitStruct.Pin = SAI2_MCLKA_Pin|SAI2_SCKA_Pin|SAI2_FSA_Pin|SAI2_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

      /* Peripheral DMA init*/

    hdma_sai2_a.Instance = DMA2_Stream4;
    hdma_sai2_a.Init.Channel = DMA_CHANNEL_3;
    hdma_sai2_a.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_sai2_a.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sai2_a.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sai2_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sai2_a.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_sai2_a.Init.Mode = DMA_CIRCULAR;
    hdma_sai2_a.Init.Priority = DMA_PRIORITY_LOW;
    hdma_sai2_a.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_sai2_a) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one stream to perform all the requested DMAs. */
    __HAL_LINKDMA(hsai,hdmarx,hdma_sai2_a);

    __HAL_LINKDMA(hsai,hdmatx,hdma_sai2_a);

    }
    if(hsai->Instance==SAI2_Block_B)
    {
      /* Peripheral clock enable */
      if (SAI2_client == 0)
      {
       __HAL_RCC_SAI2_CLK_ENABLE();
      }
    SAI2_client ++;

    /**SAI2_B_Block_B GPIO Configuration
    PG10     ------> SAI2_SD_B
    */
    GPIO_InitStruct.Pin = SAI2_SDB_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
    HAL_GPIO_Init(SAI2_SDB_GPIO_Port, &GPIO_InitStruct);

    }
}

void HAL_SAI_MspDeInit(SAI_HandleTypeDef* hsai)
{
/* SAI2 */
    if(hsai->Instance==SAI2_Block_A)
    {
    SAI2_client --;
    if (SAI2_client == 0)
      {
      /* Peripheral clock disable */
       __HAL_RCC_SAI2_CLK_DISABLE();
      }

    /**SAI2_A_Block_A GPIO Configuration
    PI4     ------> SAI2_MCLK_A
    PI5     ------> SAI2_SCK_A
    PI7     ------> SAI2_FS_A
    PI6     ------> SAI2_SD_A
    */
    HAL_GPIO_DeInit(GPIOI, SAI2_MCLKA_Pin|SAI2_SCKA_Pin|SAI2_FSA_Pin|SAI2_SDA_Pin);

    /* SAI2 DMA Deinit */
    HAL_DMA_DeInit(hsai->hdmarx);
    HAL_DMA_DeInit(hsai->hdmatx);
    }
    if(hsai->Instance==SAI2_Block_B)
    {
    SAI2_client --;
      if (SAI2_client == 0)
      {
      /* Peripheral clock disable */
      __HAL_RCC_SAI2_CLK_DISABLE();
      }

    /**SAI2_B_Block_B GPIO Configuration
    PG10     ------> SAI2_SD_B
    */
    HAL_GPIO_DeInit(SAI2_SDB_GPIO_Port, SAI2_SDB_Pin);

    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
