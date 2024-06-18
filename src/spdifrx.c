/**
  ******************************************************************************
  * File Name          : SPDIFRX.c
  * Description        : This file provides code for the configuration
  *                      of the SPDIFRX instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "spdifrx.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPDIFRX_HandleTypeDef hspdif;
DMA_HandleTypeDef hdma_spdif_rx_dt;
DMA_HandleTypeDef hdma_spdif_rx_cs;

/* SPDIFRX init function */
void MX_SPDIFRX_Init(void)
{

  hspdif.Instance = SPDIFRX;
  hspdif.Init.InputSelection = SPDIFRX_INPUT_IN0;
  hspdif.Init.Retries = SPDIFRX_MAXRETRIES_15;
  hspdif.Init.WaitForActivity = SPDIFRX_WAITFORACTIVITY_ON;
  hspdif.Init.ChannelSelection = SPDIFRX_CHANNEL_A;
  hspdif.Init.DataFormat = SPDIFRX_DATAFORMAT_MSB;
  hspdif.Init.StereoMode = SPDIFRX_STEREOMODE_ENABLE;
  hspdif.Init.PreambleTypeMask = SPDIFRX_PREAMBLETYPEMASK_ON;
  hspdif.Init.ChannelStatusMask = SPDIFRX_CHANNELSTATUS_ON;
  hspdif.Init.ValidityBitMask = SPDIFRX_VALIDITYMASK_ON;
  hspdif.Init.ParityErrorMask = SPDIFRX_PARITYERRORMASK_ON;
  if (HAL_SPDIFRX_Init(&hspdif) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_SPDIFRX_MspInit(SPDIFRX_HandleTypeDef* spdifrxHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spdifrxHandle->Instance==SPDIFRX)
  {
  /* USER CODE BEGIN SPDIFRX_MspInit 0 */

  /* USER CODE END SPDIFRX_MspInit 0 */
    /* SPDIFRX clock enable */
    __HAL_RCC_SPDIFRX_CLK_ENABLE();
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**SPDIFRX GPIO Configuration    
    PC4     ------> SPDIFRX_IN2
    PC5     ------> SPDIFRX_IN3
    PD8     ------> SPDIFRX_IN1
    PD7     ------> SPDIFRX_IN0 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* SPDIFRX DMA Init */
    /* SPDIF_RX_DT Init */
    hdma_spdif_rx_dt.Instance = DMA1_Stream1;
    hdma_spdif_rx_dt.Init.Channel = DMA_CHANNEL_0;
    hdma_spdif_rx_dt.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spdif_rx_dt.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spdif_rx_dt.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spdif_rx_dt.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_spdif_rx_dt.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_spdif_rx_dt.Init.Mode = DMA_CIRCULAR;
    hdma_spdif_rx_dt.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_spdif_rx_dt.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spdif_rx_dt) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spdifrxHandle,hdmaDrRx,hdma_spdif_rx_dt);

    /* SPDIF_RX_CS Init */
    hdma_spdif_rx_cs.Instance = DMA1_Stream6;
    hdma_spdif_rx_cs.Init.Channel = DMA_CHANNEL_0;
    hdma_spdif_rx_cs.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spdif_rx_cs.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spdif_rx_cs.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spdif_rx_cs.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_spdif_rx_cs.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_spdif_rx_cs.Init.Mode = DMA_CIRCULAR;
    hdma_spdif_rx_cs.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spdif_rx_cs.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spdif_rx_cs) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spdifrxHandle,hdmaCsRx,hdma_spdif_rx_cs);

    /* SPDIFRX interrupt Init */
    HAL_NVIC_SetPriority(SPDIF_RX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPDIF_RX_IRQn);
  /* USER CODE BEGIN SPDIFRX_MspInit 1 */

  /* USER CODE END SPDIFRX_MspInit 1 */
  }
}

void HAL_SPDIFRX_MspDeInit(SPDIFRX_HandleTypeDef* spdifrxHandle)
{

  if(spdifrxHandle->Instance==SPDIFRX)
  {
  /* USER CODE BEGIN SPDIFRX_MspDeInit 0 */

  /* USER CODE END SPDIFRX_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPDIFRX_CLK_DISABLE();
  
    /**SPDIFRX GPIO Configuration    
    PC4     ------> SPDIFRX_IN2
    PC5     ------> SPDIFRX_IN3
    PD8     ------> SPDIFRX_IN1
    PD7     ------> SPDIFRX_IN0 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4|GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_7);

    /* SPDIFRX DMA DeInit */
    HAL_DMA_DeInit(spdifrxHandle->hdmaDrRx);
    HAL_DMA_DeInit(spdifrxHandle->hdmaCsRx);

    /* SPDIFRX interrupt Deinit */
    HAL_NVIC_DisableIRQ(SPDIF_RX_IRQn);
  /* USER CODE BEGIN SPDIFRX_MspDeInit 1 */

  /* USER CODE END SPDIFRX_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
