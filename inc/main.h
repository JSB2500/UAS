/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ESP32_A_NSS_Pin GPIO_PIN_2
#define ESP32_A_NSS_GPIO_Port GPIOE
#define ESP32_A_HSK_Pin GPIO_PIN_3
#define ESP32_A_HSK_GPIO_Port GPIOE
#define ESP32_B_EN_Pin GPIO_PIN_4
#define ESP32_B_EN_GPIO_Port GPIOE
#define ESP32_B_NSS_Pin GPIO_PIN_5
#define ESP32_B_NSS_GPIO_Port GPIOE
#define ESP32_B_HSK_Pin GPIO_PIN_6
#define ESP32_B_HSK_GPIO_Port GPIOE
#define NUserButton_Pin GPIO_PIN_13
#define NUserButton_GPIO_Port GPIOC
#define NUserButton_EXTI_IRQn EXTI15_10_IRQn
#define FLASH_NSS_Pin GPIO_PIN_15
#define FLASH_NSS_GPIO_Port GPIOC
#define MCU_CLK_IN_Pin GPIO_PIN_0
#define MCU_CLK_IN_GPIO_Port GPIOH
#define LCD_NRST_Pin GPIO_PIN_0
#define LCD_NRST_GPIO_Port GPIOC
#define LCD_NSS_Pin GPIO_PIN_1
#define LCD_NSS_GPIO_Port GPIOC
#define LCD_NC_D_Pin GPIO_PIN_2
#define LCD_NC_D_GPIO_Port GPIOC
#define LCD_NBACKLIGHT_Pin GPIO_PIN_3
#define LCD_NBACKLIGHT_GPIO_Port GPIOC
#define SAI_B_DATATOSTM_Pin GPIO_PIN_0
#define SAI_B_DATATOSTM_GPIO_Port GPIOA
#define LED_Red_Pin GPIO_PIN_1
#define LED_Red_GPIO_Port GPIOA
#define LED_Green_Pin GPIO_PIN_2
#define LED_Green_GPIO_Port GPIOA
#define LED_Blue_Pin GPIO_PIN_3
#define LED_Blue_GPIO_Port GPIOA
#define LCD_SCK_Pin GPIO_PIN_5
#define LCD_SCK_GPIO_Port GPIOA
#define LCD_MISO_Pin GPIO_PIN_6
#define LCD_MISO_GPIO_Port GPIOA
#define LCD_MOSI_Pin GPIO_PIN_7
#define LCD_MOSI_GPIO_Port GPIOA
#define SAI_RxDMA_Pin GPIO_PIN_0
#define SAI_RxDMA_GPIO_Port GPIOB
#define SAI_TxDMA_Pin GPIO_PIN_1
#define SAI_TxDMA_GPIO_Port GPIOB
#define LEDCHAIN_DATA_Pin GPIO_PIN_9
#define LEDCHAIN_DATA_GPIO_Port GPIOE
#define TestPoint_Pin GPIO_PIN_11
#define TestPoint_GPIO_Port GPIOB
#define TS_NSS_Pin GPIO_PIN_12
#define TS_NSS_GPIO_Port GPIOB
#define TS_SCK_Pin GPIO_PIN_13
#define TS_SCK_GPIO_Port GPIOB
#define TS_MISO_Pin GPIO_PIN_14
#define TS_MISO_GPIO_Port GPIOB
#define TS_MOSI_Pin GPIO_PIN_15
#define TS_MOSI_GPIO_Port GPIOB
#define SAI_A_DATAFROMSTM_Pin GPIO_PIN_11
#define SAI_A_DATAFROMSTM_GPIO_Port GPIOD
#define SAI_A_FS_Pin GPIO_PIN_12
#define SAI_A_FS_GPIO_Port GPIOD
#define SAI_A_BCK_Pin GPIO_PIN_13
#define SAI_A_BCK_GPIO_Port GPIOD
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define GENSPI_SCK_Pin GPIO_PIN_10
#define GENSPI_SCK_GPIO_Port GPIOC
#define GENSPI_MISO_Pin GPIO_PIN_11
#define GENSPI_MISO_GPIO_Port GPIOC
#define GENSPI_MOSI_Pin GPIO_PIN_12
#define GENSPI_MOSI_GPIO_Port GPIOC
#define TAS_PDN_Pin GPIO_PIN_0
#define TAS_PDN_GPIO_Port GPIOD
#define TAS_NRST_Pin GPIO_PIN_1
#define TAS_NRST_GPIO_Port GPIOD
#define MUX_S0_Pin GPIO_PIN_2
#define MUX_S0_GPIO_Port GPIOD
#define MUX_S1_Pin GPIO_PIN_3
#define MUX_S1_GPIO_Port GPIOD
#define DAC_FLT_Pin GPIO_PIN_4
#define DAC_FLT_GPIO_Port GPIOD
#define DAC_DEMP_Pin GPIO_PIN_5
#define DAC_DEMP_GPIO_Port GPIOD
#define DAC_XSMT_Pin GPIO_PIN_6
#define DAC_XSMT_GPIO_Port GPIOD
#define SI_INT_Pin GPIO_PIN_5
#define SI_INT_GPIO_Port GPIOB
#define SI_INT_EXTI_IRQn EXTI9_5_IRQn
#define SI_NRST_Pin GPIO_PIN_6
#define SI_NRST_GPIO_Port GPIOB
#define SAI_A_MCLK_Pin GPIO_PIN_0
#define SAI_A_MCLK_GPIO_Port GPIOE
#define ESP32_A_EN_Pin GPIO_PIN_1
#define ESP32_A_EN_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

#define ILI9341_CSX_GPIO_Port LCD_NSS_GPIO_Port
#define ILI9341_CSX_Pin LCD_NSS_Pin
#define ILI9341_D_CX_GPIO_Port LCD_NC_D_GPIO_Port
#define ILI9341_D_CX_Pin LCD_NC_D_Pin
#define ILI9341_RESX_GPIO_Port LCD_NRST_GPIO_Port
#define ILI9341_RESX_Pin LCD_NRST_Pin
#define XPT2046_CSX_GPIO_Port TS_NSS_GPIO_Port
#define XPT2046_CSX_Pin TS_NSS_Pin
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
