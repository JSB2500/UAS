#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "cmsis_os.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_sai.h"
#include "stm32f7xx_hal_dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2c.h"
#include "spi.h"
#include "sai.h"
#include "tim.h"
#include "main.h"
#include "PJB_STM32_ESP32_FM_DAB_ADC_DAC_PA.h"
#include "JSB_SPDIFRX.h"
#include "Si468x/Si468x.h"
#include "Si468x/Si468x_FM.h"
#include "Si468x/Si468x_DAB.h"
#include "PJB_SST25V.h"
#include "AR1010.h"
#include "JSB_IR_RC5.h"
#include "JSB_General.h"
#include "JSB_UI_Elements.h"
#include "main.h"
#include "go.h"

///////////////////////////////////////////////////////////////////////////////
// UI:

#define Page_TitleColour (ILI9341_COLOR_DARKERGREY)
#define Page_ButtonColor (ILI9341_COLOR_PURPLE)
#define Page_SliderForegroundColor (ILI9341_COLOR_BRIGHTPURPLE)
#define Page_SliderBackgroundColor (ILI9341_COLOR_PURPLE)

static JSB_UI_Page_t *pCurrentPage = NULL;
static uint8_t PageChanged = 0;
static uint8_t BluetoothOutChanged = 0;

// HomePage:
static JSB_UI_Page_t *UI_pHomePage;
static JSB_UI_Control_t *pHomePage_TitleButton;
static JSB_UI_Control_t *pHomePage_InputButton;
static JSB_UI_Control_t *pHomePage_ProcessButton;
static JSB_UI_Control_t *pHomePage_PowerAmpButton;
static JSB_UI_Control_t *pHomePage_BluetoothOutButton;
static JSB_UI_Control_t *pHomePage_WiFiButton;
static JSB_UI_Control_t *pHomePage_InfoButton;
static JSB_UI_Control_t *pHomePage_HomeButton;

// InputPage:
static JSB_UI_Page_t *UI_pInputPage;
static JSB_UI_Control_t *pInputPage_InputButton_Humax;
static JSB_UI_Control_t *pInputPage_InputButton_DVD_HDD;
static JSB_UI_Control_t *pInputPage_InputButton_GCA;
static JSB_UI_Control_t *pInputPage_InputButton_Computer;
static JSB_UI_Control_t *pInputPage_InputButton_DAB;
static JSB_UI_Control_t *pInputPage_InputButton_Si468x_FM;
static JSB_UI_Control_t *pInputPage_InputButton_AR1010_FM;
static JSB_UI_Control_t *pInputPage_InputButton_AuxIn;
static JSB_UI_Control_t *pInputPage_HomeButton;

// Process page:
static JSB_UI_Page_t *UI_pProcessPage;
static JSB_UI_Control_t *pProcessPage_TitleButton;
static JSB_UI_Control_t *pProcessPage_GainControl;
static JSB_UI_Control_t *pProcessPage_DefaultGainControl;
static JSB_UI_Control_t *pProcessPage_MuteButton;
static JSB_UI_Control_t *pProcessPage_HomeButton;

// Power amplifier page:
static JSB_UI_Page_t *UI_pPowerAmplifierPage;
static JSB_UI_Control_t *pPowerAmplifierPage_TitleButton;
static JSB_UI_Control_t *pPowerAmplifierPage_VolumeControl;
static JSB_UI_Control_t *pPowerAmplifierPage_DefaultVolumeButton;
static JSB_UI_Control_t *pPowerAmplifierPage_MuteButton;
static JSB_UI_Control_t *pPowerAmplifierPage_HomeButton;

// Bluetooth out page:
static JSB_UI_Page_t *UI_pBluetoothOutPage;
static JSB_UI_Control_t *pBluetoothOutPage_TitleButton;
static JSB_UI_Control_t *pBluetoothOutPage_OffButton;
static JSB_UI_Control_t *pBluetoothOutPage_OnButton;
static JSB_UI_Control_t *pBluetoothOutPage_HomeButton;

// WiFi page:
static JSB_UI_Page_t *UI_pWiFiPage;
static JSB_UI_Control_t *pWiFiPage_TitleButton;
static JSB_UI_Control_t *pWiFiPage_OffButton;
static JSB_UI_Control_t *pWiFiPage_OnButton;
static JSB_UI_Control_t *pWiFiPage_HomeButton;

// Info page:
static JSB_UI_Page_t *UI_pInfoPage;
static JSB_UI_Control_t *pInfoPage_TitleButton;
static JSB_UI_Control_t *pInfoPage_RadioInfoButton;
static JSB_UI_Control_t *pInfoPage_HomeButton;
static JSB_UI_Control_t *pInfoPage_DebugInfoButton;

// Radio info page:
static JSB_UI_Page_t *UI_pRadioInfoPage;
static JSB_UI_Control_t *pRadioInfoPage_TitleButton;
static JSB_UI_Control_t *pRadioInfoPage_BackButton;

// Debug info page:
static JSB_UI_Page_t *UI_pDebugInfoPage;
static JSB_UI_Control_t *pDebugInfoPage_TitleButton;
static JSB_UI_Control_t *pDebugInfoPage_BackButton;

// WiFi page:
static JSB_UI_Page_t *UI_pWiFiPage;
static JSB_UI_Control_t *pWiFiPage_TitleButton;
static JSB_UI_Control_t *pWiFiPage_OffButton;
static JSB_UI_Control_t *pWiFiPage_OnButton;
static JSB_UI_Control_t *pWiFiPage_HomeButton;

///////////////////////////////////////////////////////////////////////////////
// RTOS:

extern osThreadId ProcessAudioTaskHandle;
extern osMessageQId ProcessAudioTaskParameterQueueHandle;
extern osMutexId SPI3_MutexHandle;
extern osSemaphoreId GenSPITransferCompleteSemaphoreHandle;

typedef enum
{
  patnSPITransferComplete = 1
} ProcessAudioTaskNotification_t;

void GenSPI_Begin()
{
  xSemaphoreTake(SPI3_MutexHandle, portMAX_DELAY);
}

void GenSPI_End()
{
  xSemaphoreGive(SPI3_MutexHandle);
}

///////////////////////////////////////////////////////////////////////////////
// Input channels:

typedef enum
{
  icNone,
  icSPDIF0,
  icSPDIF1,
  icSPDIF2,
  icSPDIF3,
  icSi468x_DAB,
  icSi468x_FM,
  icAR1010_FM,
  icAuxIn,
  icMax = icAuxIn
} InputChannel_t;

static InputChannel_t InputChannel = icNone;
static uint8_t InputChannelChanged = 0;
static uint8_t IR_UseChannelButtonsToControlRadio = 0;

static void SetInputChannel(InputChannel_t Value)
{
  if (InputChannel == Value)
    return;

  IR_UseChannelButtonsToControlRadio = 0;
  InputChannel = Value;
  InputChannelChanged = 1;
}

static uint8_t GetSPDIFInputChannel(InputChannel_t Value)
{
  return Value - icSPDIF0;
}

static uint8_t InputChannelIsSPDIF()
{
  switch(InputChannel)
  {
    case icSPDIF0:
    case icSPDIF1:
    case icSPDIF2:
    case icSPDIF3:
      return 1;
    default:
      return 0;
  }
}

static uint8_t InputChannelIsI2S()
{
  switch(InputChannel)
  {
    case icSi468x_DAB:
    case icSi468x_FM:
    case icAR1010_FM:
    case icAuxIn:
      return 1;
    default:
      return 0;
  }
}

static uint8_t InputChannelIsRadio()
{
  switch(InputChannel)
  {
    case icSi468x_DAB:
    case icSi468x_FM:
    case icAR1010_FM:
      return 1;
    default:
      return 0;
  }
}

///////////////////////////////////////////////////////////////////////////////

static uint8_t PowerOn = 0;

#define NumInputChannels (2)
#define I2SBuffers_NumSamples_WhenBluetoothIsNotEnabled (32)
#define I2SBuffers_NumSamples_WhenBluetoothIsEnabled (320)

#define __32ByteAligned __attribute__ ((aligned(32)))
#define ARRAYSIZE_MAX(A, B) (((A) > (B)) ? (A) : (B)) /* Crude. Assumes types match. */
__32ByteAligned AudioSample24_t I2SInputBuffer[ARRAYSIZE_MAX(I2SBuffers_NumSamples_WhenBluetoothIsNotEnabled, I2SBuffers_NumSamples_WhenBluetoothIsEnabled)];
__32ByteAligned AudioSample24_t I2SOutputBuffer[ARRAYSIZE_MAX(ARRAYSIZE_MAX(I2SBuffers_NumSamples_WhenBluetoothIsNotEnabled, I2SBuffers_NumSamples_WhenBluetoothIsEnabled), SPDIFRX_Buffer_NumSamples)];

uint32_t I2SBuffers_NumSamples = 0;

static uint8_t InternalMuteActive = 0;

#define Process_Gain_Default (1.0f)
static float Process_Gain = Process_Gain_Default;
static uint8_t Process_UserMuted = 0;

#define PA_MasterVolume_dB_Default (0.0f)
#define PA_MasterVolume_dB_Min (-80.0f)
#define PA_MasterVolume_dB_Max (20.0f)
//
static float PA_MasterVolume_dB = PA_MasterVolume_dB_Default;
static uint8_t PA_UserMuted = 0;

uint32_t OutputSampleRate = 0;

///////////////////////////////////////////////////////////////////////////////
// SPDIFRX callbacks:

void SPDIFRX_Locked_Callback(uint8_t Value)
{
  HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, Value);
}

uint8_t InternalMute(uint8_t Value);

void SPDIFRX_MuteOutput_Callback(uint8_t Value)
{
  InternalMute(Value);
}

static void SetOutputSampleRate_44100(uint32_t i_NumSamples);
static void SetOutputSampleRate_48000(uint32_t i_NumSamples);

void SPDIFRX_SetOutputSampleRate_Callback(uint32_t Value)
{
  if (Value == SAI_AUDIO_FREQUENCY_44K)
    SetOutputSampleRate_44100(SPDIFRX_Buffer_NumSamples);
  else if (Value == SAI_AUDIO_FREQUENCY_48K)
    SetOutputSampleRate_48000(SPDIFRX_Buffer_NumSamples);
  else
    OutputSampleRate = 0;
}

void ClearOutputBuffer();

void SPDIFRX_ClearOutputBuffer()
{
  ClearOutputBuffer();
}

///////////////////////////////////////////////////////////////////////////////
// Misc audio functions:

void I2SInput_Stop()
{
  ClearOutputBuffer();

  HAL_SAI_DMAStop(&hsai_BlockA2);
  HAL_SAI_DMAStop(&hsai_BlockB2);
}

uint32_t GetInputSampleRate()
{
  if (SPDIFRX_IsStarted())
    return SPDIFRX_GetInputSampleRate();

  return OutputSampleRate; // Input sample rate is the same as the output one.
}

///////////////////////////////////////////////////////////////////////////////
// ESP32:
//
// ESP32 firmware: "C:\JSB\Cloud\Spaceless\PJB_Radio_ESP32\05_BluetoothSource"

#define ESP32_TimeoutTimeInMilliseconds 1000 /* ESP32 boot time */

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi3)
  {
    BaseType_t HigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(GenSPITransferCompleteSemaphoreHandle, &HigherPriorityTaskWoken);
    portYIELD_FROM_ISR(HigherPriorityTaskWoken);
  }
}

static void *ESP32_pSamplesMemory = NULL;
static AudioSample16_t *ESP32_pSamples = NULL;
static uint32_t ESP32_Samples_NumSamples = 0;
static float ESP32_SourceSampleStepSize = 0.0f;

typedef struct
{
  GUID_t Format;
} ESP32Command_BeNothing_t;

typedef struct
{
  GUID_t Format;
  uint32_t SampleRate;
  uint16_t NumChannels;
  uint16_t NumBitsPerSampleChannel;
  uint32_t NumSamplesPerBlock;
} ESP32Command_BeBluetoothSource_t;

uint8_t ESP32_SendCommmandToBeNothing()
{
  ESP32Command_BeNothing_t Command;
  uint8_t ESP32_Ready;
  uint32_t NumTicks_Start;
  HAL_StatusTypeDef HAL_Result;

  NumTicks_Start = HAL_GetTick();
  do
  {
    ESP32_Ready = HAL_GPIO_ReadPin(ESP32_A_HSK_GPIO_Port, ESP32_A_HSK_Pin);
    if (HAL_GetTick() - NumTicks_Start > ESP32_TimeoutTimeInMilliseconds)
      return 0;
  } while (!ESP32_Ready);

  StringToGUID("{837C7FAA-9ADE-4E53-87B6-C5F42CEF40E9}", &Command.Format); // ESP32_BeNothing_CommandFormat0

  xSemaphoreTake(SPI3_MutexHandle, portMAX_DELAY);
  HAL_GPIO_WritePin(ESP32_A_NSS_GPIO_Port, ESP32_A_NSS_Pin, 0);
  HAL_Result = HAL_SPI_Transmit(&hspi3, (uint8_t *)&Command, sizeof(Command), ESP32_TimeoutTimeInMilliseconds);
  HAL_GPIO_WritePin(ESP32_A_NSS_GPIO_Port, ESP32_A_NSS_Pin, 1);
  xSemaphoreGive(SPI3_MutexHandle);
  if (HAL_Result != HAL_OK)
    return 0;

  return 1;
}

uint8_t ESP32_SendCommmandToBeBluetoothSource()
{
  ESP32Command_BeBluetoothSource_t Command;
  uint8_t ESP32_Ready;
  uint32_t NumTicks_Start;
  HAL_StatusTypeDef HAL_Result;

  NumTicks_Start = HAL_GetTick();
  do
  {
    ESP32_Ready = HAL_GPIO_ReadPin(ESP32_A_HSK_GPIO_Port, ESP32_A_HSK_Pin);
    if (HAL_GetTick() - NumTicks_Start > ESP32_TimeoutTimeInMilliseconds)
      return 0;
  } while (!ESP32_Ready);

  StringToGUID("{6B44D75A-86B2-4A89-8E04-D1ABCF16E78F}", &Command.Format); // ESP32_BeBluetoothSource_CommandFormat0
  Command.SampleRate = SAI_AUDIO_FREQUENCY_44K; // Always operate at a sample rate of 44.1 KHz.
  Command.NumChannels = 2;
  Command.NumBitsPerSampleChannel = 16;
  Command.NumSamplesPerBlock = ESP32_Samples_NumSamples;

  xSemaphoreTake(SPI3_MutexHandle, portMAX_DELAY);
  HAL_GPIO_WritePin(ESP32_A_NSS_GPIO_Port, ESP32_A_NSS_Pin, 0);
  HAL_Result = HAL_SPI_Transmit(&hspi3, (uint8_t *)&Command, sizeof(Command), ESP32_TimeoutTimeInMilliseconds);
  HAL_GPIO_WritePin(ESP32_A_NSS_GPIO_Port, ESP32_A_NSS_Pin, 1);
  xSemaphoreGive(SPI3_MutexHandle);
  if (HAL_Result != HAL_OK)
    return 0;

  return 1;
}

uint8_t ESP32_SendSamplesFromAudioOutputBufferToESP32(uint8_t SecondHalf)
{
  int32_t AudioOutputBufferStartIndex, SampleIndex;
  AudioSample16_t ESPSample;
  AudioSample24_t AudioOutputBufferSample;
  GUID_t Format;
  uint8_t ESP32_Ready;
  HAL_StatusTypeDef HAL_Result;

  ESP32_Ready = HAL_GPIO_ReadPin(ESP32_A_HSK_GPIO_Port, ESP32_A_HSK_Pin);
  if (!ESP32_Ready)
    return 0; // Don't wait.

  StringToGUID("{4839C500-E431-42D3-9508-83009C1E31D4}", &Format); // ESP32_BeBluetoothSource_DataFormat0

  if (SecondHalf)
    AudioOutputBufferStartIndex = I2SBuffers_NumSamples / 2;
  else
    AudioOutputBufferStartIndex = 0;

  for (SampleIndex = 0; SampleIndex < ESP32_Samples_NumSamples; ++SampleIndex)
  {
    float I2SOutputBufferPosition = AudioOutputBufferStartIndex + SampleIndex * ESP32_SourceSampleStepSize;

    double I2SOutputBufferPosition_Integral;
    double I2SOutputBufferPosition_Fractional;
    I2SOutputBufferPosition_Fractional = modf(I2SOutputBufferPosition, &I2SOutputBufferPosition_Integral);

    int32_t SourceSampleIndex0 = Wrap_int((int)(I2SOutputBufferPosition_Integral) - 1, I2SBuffers_NumSamples);
    int32_t SourceSampleIndex1 = Wrap_int(SourceSampleIndex0 + 1, I2SBuffers_NumSamples);
    int32_t SourceSampleIndex2 = Wrap_int(SourceSampleIndex1 + 1, I2SBuffers_NumSamples);
    int32_t SourceSampleIndex3 = Wrap_int(SourceSampleIndex2 + 1, I2SBuffers_NumSamples);

    AudioSample24_t SourceSample0 = I2SOutputBuffer[SourceSampleIndex0];
    AudioSample24_t SourceSample1 = I2SOutputBuffer[SourceSampleIndex1];
    AudioSample24_t SourceSample2 = I2SOutputBuffer[SourceSampleIndex2];
    AudioSample24_t SourceSample3 = I2SOutputBuffer[SourceSampleIndex3];

    AudioOutputBufferSample.Left = CubicInterpolate(SourceSample0.Left, SourceSample1.Left, SourceSample2.Left, SourceSample3.Left, I2SOutputBufferPosition_Fractional);
    AudioOutputBufferSample.Right = CubicInterpolate(SourceSample0.Right, SourceSample1.Right, SourceSample2.Right, SourceSample3.Right, I2SOutputBufferPosition_Fractional);

    ESPSample.Left = AudioOutputBufferSample.Left >> 8;
    ESPSample.Right = AudioOutputBufferSample.Right >> 8;

    ESP32_pSamples[SampleIndex] = ESPSample;
  }

  xSemaphoreTake(GenSPITransferCompleteSemaphoreHandle, 0); //!!!RTOS: Clear semaphore before use. This appears necessary.
  xSemaphoreTake(SPI3_MutexHandle, portMAX_DELAY);
  HAL_GPIO_WritePin(ESP32_A_NSS_GPIO_Port, ESP32_A_NSS_Pin, 0);
  HAL_SPI_Transmit(&hspi3, (uint8_t *)&Format, sizeof(Format), HAL_MAX_DELAY);
  uint32_t NumBytes = ESP32_Samples_NumSamples * sizeof(AudioSample16_t);
  SCB_CleanDCache_by_Addr((uint32_t *)ESP32_pSamples, NumBytes);
  HAL_Result = HAL_SPI_Transmit_DMA(&hspi3, (uint8_t *)ESP32_pSamples, NumBytes); // One DMA item is 8 bits.
  if (HAL_Result != HAL_OK)
  {
    xSemaphoreGive(SPI3_MutexHandle);
    HAL_GPIO_WritePin(ESP32_A_NSS_GPIO_Port, ESP32_A_NSS_Pin, 1);
    return 0;
  }
  xSemaphoreTake(GenSPITransferCompleteSemaphoreHandle, portMAX_DELAY); // Wait for transmit to complete.
  HAL_GPIO_WritePin(ESP32_A_NSS_GPIO_Port, ESP32_A_NSS_Pin, 1);
  xSemaphoreGive(SPI3_MutexHandle);

  return 1;
}

///////////////////////////////////////////////////////////////////////////////
// Bluetooth:

static uint8_t BluetoothOut_On = 0;
static uint8_t BluetoothOut_Started = 0;

uint8_t BluetoothOut_Start()
{
  IfNotNullFreeAndNull((void **)&ESP32_pSamplesMemory);
  ESP32_pSamples = NULL;

  if (!OutputSampleRate)
    return 0;

  ESP32_SourceSampleStepSize = (float)OutputSampleRate / (float)SAI_AUDIO_FREQUENCY_44K;
  ESP32_Samples_NumSamples = (int)round((I2SBuffers_NumSamples / 2) / ESP32_SourceSampleStepSize);

  // Align the samples buffer to a 32 byte boundary. This is needed for cache invalidation purposes.
  ESP32_pSamplesMemory = calloc(1, ESP32_Samples_NumSamples * sizeof(AudioSample16_t) + 31);
  ESP32_pSamples = (AudioSample16_t *)(((uint32_t)ESP32_pSamplesMemory + 31) & ~31);

  return ESP32_SendCommmandToBeBluetoothSource();
}

void BluetoothOut_Stop()
{
  ESP32_SendCommmandToBeNothing();

  IfNotNullFreeAndNull((void **)&ESP32_pSamplesMemory);
  ESP32_pSamples = NULL;
}

///////////////////////////////////////////////////////////////////////////////
// WiFi:

static uint8_t WiFi_On = 0;

///////////////////////////////////////////////////////////////////////////////
// Audio:

float I2S_24bitToNormalizedFloat(int32_t Value)
{
  if (Value & 0x00800000)
    Value |= 0xFF000000;

  return Value / 16777216.0f;
}

int32_t I2S_NormalizedFloatTo24bit(float Value)
{
  return round(16777216.0f * Value);
}

void AudioBuffer_TransferFromInputBufferToOutputBuffer(AudioBufferHalf_t AudioBufferHalf)
{
  uint16_t FirstSampleIndex, SampleIndex;
  float Audio_Left, Audio_Right;
  AudioSample24_t *pInputSample, *pOutputSample;

  FirstSampleIndex = 0;
  switch (AudioBufferHalf)
  {
    case abhFirstHalf: FirstSampleIndex = 0; break;
    case abhSecondHalf: FirstSampleIndex = I2SBuffers_NumSamples / 2; break;
    default: Error_Handler();
  }

  for(SampleIndex = FirstSampleIndex; SampleIndex < FirstSampleIndex + (I2SBuffers_NumSamples / 2); ++SampleIndex)
  {
    pInputSample = &I2SInputBuffer[SampleIndex];
    pOutputSample = &I2SOutputBuffer[SampleIndex];

    Audio_Left = I2S_24bitToNormalizedFloat(pInputSample->Left);
    Audio_Right = I2S_24bitToNormalizedFloat(pInputSample->Right);

    Audio_Left *= Process_Gain;
    Audio_Right *= Process_Gain;

    pOutputSample-> Left = I2S_NormalizedFloatTo24bit(Audio_Left);
    pOutputSample-> Right = I2S_NormalizedFloatTo24bit(Audio_Right);
  }
}

void ProcessAudio(AudioBufferHalf_t AudioBufferHalf)
// The (RTOS) time from the end of the ISR to here is 2.4 us.
// The (RTOS) time from calling xQueueSendFromISR() to here is 4.0 us.
{
  if (InputChannelIsI2S())
    AudioBuffer_TransferFromInputBufferToOutputBuffer(AudioBufferHalf);
  else if (SPDIFRX_IsStarted())
    SPDIFRX_CopySamplesToGivenBuffer(I2SOutputBuffer, AudioBufferHalf); // This MUST be called even if the output is muted (to stay in sync).

  if (InternalMuteActive || Process_UserMuted)
    ClearOutputBuffer(); // Could clear just the relevant half of the buffer.

  if (BluetoothOut_Started)
  {
    // Send first half of buffer to ESP32:
    if (!SPDIFRX_IsStarted() || SPDIFRX_IsReceiving()) // Disable during SPDIFRX sample rate measurement as otherwise the measurement result is too small e.g. 43kHz instead of 44.1kHz.
      ESP32_SendSamplesFromAudioOutputBufferToESP32(AudioBufferHalf == abhSecondHalf);
  }
}

typedef enum
{
  dibpNone,
  dibpEndOfFirstHalf,
  dibpEndOfSecondHalf
} DmaInterruptBufferPosition_t;

static DmaInterruptBufferPosition_t SAI_BlockA2_PreviousCallbackPosition = dibpNone;
static DmaInterruptBufferPosition_t SAI_BlockB2_PreviousCallbackPosition = dibpNone;

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai == &hsai_BlockB2)
  {
    SAI_BlockB2_PreviousCallbackPosition = dibpEndOfFirstHalf;

    if (SAI_BlockA2_PreviousCallbackPosition == dibpEndOfFirstHalf)
    {
      AudioBufferHalf_t AudioBufferHalf = abhFirstHalf;
      BaseType_t HigherProrityTaskWoken;
      xQueueSendFromISR(ProcessAudioTaskParameterQueueHandle, &AudioBufferHalf, &HigherProrityTaskWoken);
      portYIELD_FROM_ISR(HigherProrityTaskWoken);
    }
  }
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai == &hsai_BlockB2)
  {
    SAI_BlockB2_PreviousCallbackPosition = dibpEndOfSecondHalf;

    if (SAI_BlockA2_PreviousCallbackPosition == dibpEndOfSecondHalf)
    {
      AudioBufferHalf_t AudioBufferHalf = abhSecondHalf;
      BaseType_t HigherProrityTaskWoken;
      xQueueSendFromISR(ProcessAudioTaskParameterQueueHandle, &AudioBufferHalf, &HigherProrityTaskWoken);
      portYIELD_FROM_ISR(HigherProrityTaskWoken);
    }
  }
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai == &hsai_BlockA2)
  {
    SAI_BlockA2_PreviousCallbackPosition = dibpEndOfFirstHalf;

    if (!InputChannelIsI2S() || (SAI_BlockB2_PreviousCallbackPosition == dibpEndOfFirstHalf))
    {
      AudioBufferHalf_t AudioBufferHalf = abhFirstHalf;
      BaseType_t HigherProrityTaskWoken;
      xQueueSendFromISR(ProcessAudioTaskParameterQueueHandle, &AudioBufferHalf, &HigherProrityTaskWoken);
      portYIELD_FROM_ISR(HigherProrityTaskWoken);
    }
  }
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai == &hsai_BlockA2)
  {
    SAI_BlockA2_PreviousCallbackPosition = dibpEndOfSecondHalf;

    if (!InputChannelIsI2S() || (SAI_BlockB2_PreviousCallbackPosition == dibpEndOfSecondHalf))
    {
      AudioBufferHalf_t AudioBufferHalf = abhSecondHalf;
      BaseType_t HigherProrityTaskWoken;
      xQueueSendFromISR(ProcessAudioTaskParameterQueueHandle, &AudioBufferHalf, &HigherProrityTaskWoken);
      portYIELD_FROM_ISR(HigherProrityTaskWoken);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// Si4684:

typedef enum
{
  sttNone,
  sttCurrent,
  sttPrevious,
  sttNext

} Radio_ServiceToTune_t;

static Radio_ServiceToTune_t Radio_ServiceToTune = sttNone;
#define DAB_ServiceNameMaxLength 128
char DAB_ServiceName[DAB_ServiceNameMaxLength] = "";
static uint16_t DAB_NumServices = 0;
static uint16_t DAB_CurrentServiceID = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == SI_INT_Pin)
    si468x_interrupt();
}

uint8_t DAB_IsLikedService(char *pServiceName)
{
  if (strcmp(pServiceName, "BBC Radio 1") == 0)
    return 1;
  if (strcmp(pServiceName, "BBC Radio 2") == 0)
    return 1;
  if (strcmp(pServiceName, "BBC Radio 3") == 0)
    return 1;
  if (strcmp(pServiceName, "BBC Radio 4") == 0)
    return 1;
  if (strcmp(pServiceName, "Classic FM") == 0)
    return 1;
  if (strcmp(pServiceName, "BBC Sheffield") == 0)
    return 1;
  if (strcmp(pServiceName, "Hallam FM") == 0)
    return 1;

  return 0;
}

uint8_t DAB_TuneService(Radio_ServiceToTune_t ServiceToTune, uint8_t GetLikedServiceOnly)
{
  uint32_t InitialServiceID = DAB_CurrentServiceID;

  if (!DAB_NumServices)
    return 0;

  uint8_t Found = 0;

  if (ServiceToTune == sttCurrent)
  {
    JSB_si468x_load_service_name_from_flash(DAB_CurrentServiceID, DAB_ServiceName, DAB_ServiceNameMaxLength);
    if (!GetLikedServiceOnly || DAB_IsLikedService(DAB_ServiceName))
      Found = 1;

    if (!Found)
      ServiceToTune = sttNext;
  }

  if (!Found)
  {
    do
    {
      switch (ServiceToTune)
      {
        case sttPrevious:
          --DAB_CurrentServiceID;
          if (DAB_CurrentServiceID == 0)
            DAB_CurrentServiceID = DAB_NumServices - 1;
          break;

        case sttNext:
          ++DAB_CurrentServiceID;
          if (DAB_CurrentServiceID == DAB_NumServices)
            DAB_CurrentServiceID = 0;
          break;

        default:
          Error_Handler();
      }

      if (DAB_CurrentServiceID == InitialServiceID)
        break;

      JSB_si468x_load_service_name_from_flash(DAB_CurrentServiceID, DAB_ServiceName, DAB_ServiceNameMaxLength);

      if (!GetLikedServiceOnly || DAB_IsLikedService(DAB_ServiceName))
      {
        Found = 1;
        break;
      }
    } while(1);
  }

  if (Found)
  {
    DAB_DigRad_Status DAB_Status;

    si468x_DAB_tune_service(DAB_CurrentServiceID);
    si468x_DAB_get_digrad_status(&DAB_Status);
    return DAB_Status.VALID ? 1 : 0;
  }

  return 0;
}

void Si4684_Initialize(enum Si468x_MODE Mode)
{
  GUID_t DAB_FlashFormat0, DAB_FlashFormatID;
  char S[128];
  uint8_t WasMuted;

  WasMuted = InternalMute(1); // Mute because the I2S lines from the Si4684 float during its initialization, and tend to pick up spurious signals.

  si468x_init(Mode);

  if (Mode == Si468x_MODE_DAB)
  {
    // Do a DAB scan if the flash data is either not written or the wrong format (version):
    StringToGUID("{8f74f7d6-eba6-4a72-bd43-16f123bbc3e6}", &DAB_FlashFormat0);
    SST25_ReadBlock(0, (uint8_t *)&DAB_FlashFormatID, 16);
    if (IsUserButtonPressed() || !GUID_IsEqual(&DAB_FlashFormatID, &DAB_FlashFormat0))
    {
      ILI9341_Clear(0);
      sprintf(S, "DAB scanning...");
      ILI9341_DrawTextAtXY(S, 0, 1 * ILI9341_GetFontYSpacing(), tpLeft);
      if (ILI9341_UsingBackBuffer())
        ILI9341_CopyBackBufferToDisplay();
      PageChanged = 1;

      SST25_EraseSector_4K(0);
      SST25_WriteBlock(0, (uint8_t *)&DAB_FlashFormat0, 16);

      si468x_DAB_band_scan();
    }

    SST25_ReadBlock(4096, (uint8_t *) &DAB_NumServices, 2);
  }

  InternalMute(WasMuted);
}

void Si4684_RequireMode(enum Si468x_MODE Mode)
{
  if (Mode == JSB_si468x_get_current_mode())
    return;

  Si4684_Initialize(Mode);
}

///////////////////////////////////////////////////////////////////////////////
// Audio:

void ClearOutputBuffer()
{
  AudioSample24_t ZeroSample={0,0};
  for(uint32_t SampleIndex = 0; SampleIndex < I2SBuffers_NumSamples; ++SampleIndex)
    I2SOutputBuffer[SampleIndex] = ZeroSample;
}

void UpdateMuteAndVolume()
{
  DAC_Mute(0); // Don't mute this.

  if (InternalMuteActive) // Internal mute trumps user settings.
    PA_Mute();
  else
  {
    if (PA_UserMuted)
      PA_Mute();
    else
      PA_SetMasterVolume(PA_MasterVolume_dB);
  }
}

uint8_t InternalMute(uint8_t Value)
// Used to silence the output if bad stuff would otherwise sound.
// Returns original value.
{
  uint8_t Result = InternalMuteActive;
  InternalMuteActive = Value;
  UpdateMuteAndVolume();
  return Result;
}

static void SetOutputSampleRate(uint32_t i_SampleRate, uint32_t i_NumSamples, uint32_t i_N, uint32_t i_P, uint32_t i_Q, uint32_t i_DivQ)
// This must be called when switching between SPDIFRX and I2S passthrough because different sized buffers are used.
// Calling this causes the I2S master signals to stop for a short period. This causes a slight click in the DAC output (e.g. when switching from no input to SPDIFRX) even with the DAC muted.
{
  uint8_t WasMuted;

  WasMuted = InternalMute(1); // Try to prevent click from DAC output when stopping DMAs. Not completely successful.
  HandleHALResult(HAL_SAI_DMAStop(&hsai_BlockB2), "HAL_SAI_DMAStop");
  HandleHALResult(HAL_SAI_DMAStop(&hsai_BlockA2), "HAL_SAI_DMAStop");
  InternalMute(WasMuted);

  ClearOutputBuffer();

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  EnsureFixPresent_Fix_JSB_20191115_HAL_RCCEx_GetPeriphCLKConfig__PLLDivisionFactors();
  HAL_RCCEx_GetPeriphCLKConfig(&PeriphClkInitStruct);

  PeriphClkInitStruct.PLLI2S.PLLI2SN = i_N;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = i_P;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = i_Q;
  PeriphClkInitStruct.PLLI2SDivQ = i_DivQ;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) // NB: This disables the PLL whilst changing its parameters.
    Error_Handler();

  hsai_BlockA2.Init.AudioFrequency = i_SampleRate;
  OutputSampleRate = i_SampleRate;

  I2SBuffers_NumSamples = i_NumSamples;

  HandleHALResult(HAL_SAI_InitProtocol(&hsai_BlockA2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, NumInputChannels), "HAL_SAI_InitProtocol");

  // => The order that the DMAs are started matters!
  //    => If the input DMA (BlockB2) is started first, the input DMA lags the output one by 20 us (Debug optimization).
  //       => Adding a delay between calling HAL_SAI_Receive_DMA(hsai_BlockB2) and HAL_SAI_Transmit_DMA(hsai_BlockA2) *does not* affect this. Presumably the slave starts with the master (BlockA2).
  //    => If the output DMA (BlockA2) is started first, the input DMA lags the output one by 41 us (Debug optimization).
  //       => Adding a delay between calling HAL_SAI_Transmit_DMA(hsai_BlockA2) and HAL_SAI_Receive_DMA(hsai_BlockB2) *does* affect this.
  // => I2S startup:
  //    => The I2S BitClock starts when __HAL_SAI_ENABLE() is called in HAL_SAI_Transmit_DMA(&hsai_BlockA2, ...).
  //    => The I2S L_n/R signal starts when "hsai->Instance->CR1 |= SAI_xCR1_DMAEN" is executed in HAL_SAI_Transmit_DMA(&hsai_BlockA2, ...).
  //    => Starting the L_n/R signal causes a slight click in the DAC output (but not the PA output).
  WasMuted = InternalMute(1); // Try to prevent click from DAC output when starting DMAs. Not completely successful.
  if (InputChannelIsI2S())
    HandleHALResult(HAL_SAI_Receive_DMA(&hsai_BlockB2, (uint8_t *)I2SInputBuffer, NumInputChannels * I2SBuffers_NumSamples), "HAL_SAI_Receive_DMA");
  HandleHALResult(HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t *)I2SOutputBuffer, NumInputChannels * I2SBuffers_NumSamples), "HAL_SAI_Transmit_DMA");
  InternalMute(WasMuted);

  PA_ClearErrorStatus(); // Clear the likely MCLK error. !!! It seems that the PA can crash etc on MCLK change, giving no audio output. It may not be a good idea to hide this by clearing the error. !!!
}

static void SetOutputSampleRate_44100(uint32_t i_NumSamples)
{
  SetOutputSampleRate(SAI_AUDIO_FREQUENCY_44K, i_NumSamples, 429, RCC_PLLI2SP_DIV4, 2, 19);
}

static void SetOutputSampleRate_48000(uint32_t i_NumSamples)
{
  SetOutputSampleRate(SAI_AUDIO_FREQUENCY_48K, i_NumSamples, 344, RCC_PLLI2SP_DIV4, 14, 1);
}

///////////////////////////////////////////////////////////////////////////////

void SetCurrentPage(JSB_UI_Page_t *Value)
{
  pCurrentPage = Value;
  PageChanged = 1;
}

void SetPowerOn(uint8_t Value)
{
  PowerOn = Value;
  SetCurrentPage(UI_pHomePage);
}

void Process_SetGain(float Value)
{
  Value = Clamp_float(Value, 0.0f, 1.0f);

  Process_Gain = Value;
  SPDIFRX_TransferGain = Value;
  PageChanged = 1;
}

void Process_SetMute(uint8_t Value)
{
  Process_UserMuted = Value;
  PageChanged = 1;
}

void PA_SetUserMute(uint8_t Value)
{
  PA_UserMuted = Value;
  UpdateMuteAndVolume();
  PageChanged = 1;
}

void PA_SetVolume(float Value)
// Unmutes if muted.
{
  Value = Clamp_float(Value, PA_MasterVolume_dB_Min, PA_MasterVolume_dB_Max);

  Process_SetMute(0);
  PA_UserMuted = 0;
  PA_MasterVolume_dB = Value;
  UpdateMuteAndVolume();
  PageChanged = 1;
}

void BluetoothOut_SetOffOn(uint8_t Value)
{
  if (Value != BluetoothOut_On)
  {
    BluetoothOut_On = Value;
    ESP32_A_Enable(Value);
    BluetoothOutChanged = 1;
    PageChanged = 1;
  }
}

void WiFi_SetOffOn(uint8_t Value)
{
  if (Value != WiFi_On)
  {
    WiFi_On = Value;
    ESP32_B_Enable(Value);
    PageChanged = 1;
  }
}

void SetUserMute(uint8_t Value)
// Sets both PA mute and all mute. For use by remote control, which doesn't distinguish between PA mute and all mute. MAY CAUSE LOGICAL PROBLEMS!!!
{
  PA_SetUserMute(Value);
  Process_SetMute(Value);
  PageChanged = 1;
}

///////////////////////////////////////////////////////////////////////////////
// Control utility functions:

void DrawRectAroundControl(JSB_UI_Control_t *pControl)
{
  JSB_UI_Rect_t Rect;

  Rect = pControl->Rect;
  Rect.Left -= 2;
  Rect.Top -= 2;
  Rect.Width += 4;
  Rect.Height += 4;

  ILI9341_DrawRectangle(Rect.Left, Rect.Top, Rect. Width, Rect.Height, ILI9341_COLOR_WHITE);
}

///////////////////////////////////////////////////////////////////////////////
// HomePage:

void HomePage_TitleBarPressed(JSB_UI_Control_t *pControl)
{
  JSB_UI_Page_PressUp();
  SetPowerOn(0);
}

void HomePage_InputButtonPressed(JSB_UI_Control_t *pControl)
{
  JSB_UI_Page_PressUp();
  SetCurrentPage(UI_pInputPage);
}

void HomePage_ProcessButtonPressed(JSB_UI_Control_t *pControl)
{
  JSB_UI_Page_PressUp();
  SetCurrentPage(UI_pProcessPage);
}

void HomePage_PowerAmpButtonPressed(JSB_UI_Control_t *pControl)
{
  JSB_UI_Page_PressUp();
  SetCurrentPage(UI_pPowerAmplifierPage);
}

void HomePage_BluetoothOutButtonPressed(JSB_UI_Control_t *pControl)
{
  JSB_UI_Page_PressUp();
  SetCurrentPage(UI_pBluetoothOutPage);
}

void HomePage_WiFiButtonPressed(JSB_UI_Control_t *pControl)
{
  JSB_UI_Page_PressUp();
  SetCurrentPage(UI_pWiFiPage);
}

void HomePage_InfoButtonPressed(JSB_UI_Control_t *pControl)
{
  JSB_UI_Page_PressUp();
  SetCurrentPage(UI_pInfoPage);
}

void CreateHomePage()
{
  uint32_t Y;
  uint32_t YInc = 40;
  uint32_t ButtonWidth = 160;

  UI_pHomePage = JSB_UI_CreatePage();

  pHomePage_TitleButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(0, 0, 240, 32), "Phil's Uber Audio System", 0, Page_TitleColour, NULL, 1, &HomePage_TitleBarPressed, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pHomePage, pHomePage_TitleButton);
  //
  Y = 60;
  pHomePage_InputButton = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(120, Y, ButtonWidth, 32), "Input", 0, Page_ButtonColor, NULL, 1, &HomePage_InputButtonPressed, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pHomePage, pHomePage_InputButton);
  //
  Y += YInc;
  pHomePage_ProcessButton = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(120, Y, ButtonWidth, 32), "Process", 0, Page_ButtonColor, NULL, 1, &HomePage_ProcessButtonPressed, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pHomePage, pHomePage_ProcessButton);
  //
  Y += YInc;
  pHomePage_PowerAmpButton = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(120, Y, ButtonWidth, 32), "Power amp", 0, Page_ButtonColor, NULL, 1, &HomePage_PowerAmpButtonPressed, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pHomePage, pHomePage_PowerAmpButton);
  //
  Y += YInc;
  pHomePage_BluetoothOutButton = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(120, Y, ButtonWidth, 32), "Bluetooth out", 0, Page_ButtonColor, NULL, 1, &HomePage_BluetoothOutButtonPressed, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pHomePage, pHomePage_BluetoothOutButton);
  //
  Y += YInc;
  pHomePage_WiFiButton = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(120, Y, ButtonWidth, 32), "WiFi", 0, Page_ButtonColor, NULL, 1, &HomePage_WiFiButtonPressed, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pHomePage, pHomePage_WiFiButton);
  //
  Y += YInc;
  pHomePage_InfoButton = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(120, Y, ButtonWidth, 32), "Info", 0, Page_ButtonColor, NULL, 1, &HomePage_InfoButtonPressed, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pHomePage, pHomePage_InfoButton);
  //
  pHomePage_HomeButton = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(120, 304, 100, 32), "Home", 0, Page_ButtonColor, NULL, 1, NULL, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pHomePage, pHomePage_HomeButton);
}

///////////////////////////////////////////////////////////////////////////////
// InputPage:

void InputPage_InputButtonPressed(JSB_UI_Control_t *pControl)
{
  if(pControl == pInputPage_InputButton_Humax)
    SetInputChannel(icSPDIF0);
  else if(pControl == pInputPage_InputButton_DVD_HDD)
    SetInputChannel(icSPDIF1);
  else if(pControl == pInputPage_InputButton_GCA)
    SetInputChannel(icSPDIF2);
  else if(pControl == pInputPage_InputButton_Computer)
    SetInputChannel(icSPDIF3);
  else if(pControl == pInputPage_InputButton_DAB)
    SetInputChannel(icSi468x_DAB);
  else if(pControl == pInputPage_InputButton_Si468x_FM)
    SetInputChannel(icSi468x_FM);
  else if(pControl == pInputPage_InputButton_AR1010_FM)
    SetInputChannel(icAR1010_FM);
  else if(pControl == pInputPage_InputButton_AuxIn)
    SetInputChannel(icAuxIn);
}

void InputPage_InputButtonCustomDraw(JSB_UI_Control_t *pControl)
{
  if (pControl == pInputPage_InputButton_Humax)
  {
    if (InputChannel != icSPDIF0)
      return;
  }
  else if(pControl == pInputPage_InputButton_DVD_HDD)
  {
    if (InputChannel != icSPDIF1)
      return;
  }
  else if(pControl == pInputPage_InputButton_GCA)
  {
    if (InputChannel != icSPDIF2)
      return;
  }
  else if(pControl == pInputPage_InputButton_Computer)
  {
    if (InputChannel != icSPDIF3)
      return;
  }
  else if(pControl == pInputPage_InputButton_DAB)
  {
    if (InputChannel != icSi468x_DAB)
      return;
  }
  else if(pControl == pInputPage_InputButton_Si468x_FM)
  {
    if (InputChannel != icSi468x_FM)
      return;
  }
  else if(pControl == pInputPage_InputButton_AR1010_FM)
  {
    if (InputChannel != icAR1010_FM)
      return;
  }
  else if(pControl == pInputPage_InputButton_AuxIn)
  {
    if (InputChannel != icAuxIn)
      return;
  }

  DrawRectAroundControl(pControl);
}

void InputPage_HomeButtonPressed(JSB_UI_Control_t *pControl)
{
  JSB_UI_Page_PressUp();
  SetCurrentPage(UI_pHomePage);
}

void CreateInputPage()
{
  uint32_t Y;
  uint32_t YInc = 40;
  uint32_t ButtonWidth = 236; // Leave room for selection rectangle.

  UI_pInputPage = JSB_UI_CreatePage();

  Y = 18;
  pInputPage_InputButton_Humax = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(120, Y, ButtonWidth, 32), "Humax", 0, Page_ButtonColor, NULL, 1, &InputPage_InputButtonPressed, NULL, InputPage_InputButtonCustomDraw);
  JSB_UI_AddControlToPage(UI_pInputPage, pInputPage_InputButton_Humax);
  //
  Y += YInc;
  pInputPage_InputButton_DVD_HDD = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(120, Y, ButtonWidth, 32), "DVD/HDD", 0, Page_ButtonColor, NULL, 1, &InputPage_InputButtonPressed, NULL, InputPage_InputButtonCustomDraw);
  JSB_UI_AddControlToPage(UI_pInputPage, pInputPage_InputButton_DVD_HDD);
  //
  Y += YInc;
  pInputPage_InputButton_GCA = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(120, Y, ButtonWidth, 32), "Google Chromecast Audio", 0, Page_ButtonColor, NULL, 1, &InputPage_InputButtonPressed, NULL, InputPage_InputButtonCustomDraw);
  JSB_UI_AddControlToPage(UI_pInputPage, pInputPage_InputButton_GCA);
  //
  Y += YInc;
  pInputPage_InputButton_Computer = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(120, Y, ButtonWidth, 32), "Computer", 0, Page_ButtonColor, NULL, 1, &InputPage_InputButtonPressed, NULL, InputPage_InputButtonCustomDraw);
  JSB_UI_AddControlToPage(UI_pInputPage, pInputPage_InputButton_Computer);
  //
  Y += YInc;
  pInputPage_InputButton_DAB = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(59, Y, 114, 32), "Si468x DAB", 0, Page_ButtonColor, NULL, 1, &InputPage_InputButtonPressed, NULL, InputPage_InputButtonCustomDraw);
  JSB_UI_AddControlToPage(UI_pInputPage, pInputPage_InputButton_DAB);
  pInputPage_InputButton_Si468x_FM = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(181, Y, 114, 32), "Si468x FM", 0, Page_ButtonColor, NULL, 1, &InputPage_InputButtonPressed, NULL, InputPage_InputButtonCustomDraw);
  JSB_UI_AddControlToPage(UI_pInputPage, pInputPage_InputButton_Si468x_FM);
  //
  Y += YInc;
  pInputPage_InputButton_AR1010_FM = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(120, Y, ButtonWidth, 32), "AR1010 FM", 0, Page_ButtonColor, NULL, 1, &InputPage_InputButtonPressed, NULL, InputPage_InputButtonCustomDraw);
  JSB_UI_AddControlToPage(UI_pInputPage, pInputPage_InputButton_AR1010_FM);
  //
  Y += YInc;
  pInputPage_InputButton_AuxIn = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(120, Y, ButtonWidth, 32), "Aux in", 0, Page_ButtonColor, NULL, 1, &InputPage_InputButtonPressed, NULL, InputPage_InputButtonCustomDraw);
  JSB_UI_AddControlToPage(UI_pInputPage, pInputPage_InputButton_AuxIn);
  //
  pInputPage_HomeButton = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(120, 304, 100, 32), "Home", 0, Page_ButtonColor, NULL, 1, &InputPage_HomeButtonPressed, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pInputPage, pInputPage_HomeButton);
}

///////////////////////////////////////////////////////////////////////////////
// ProcessPage:

void ProcessPage_GainControlChanged(JSB_UI_Control_t *pControl)
{
  Process_SetGain(Process_Gain);
}

void ProcessPage_DefaultGainControlPressed(JSB_UI_Control_t *pControl)
{
  Process_SetGain(Process_Gain_Default);
}

void ProcessPage_MuteButtonPressed(JSB_UI_Control_t *pControl)
{
  Process_SetMute(!Process_UserMuted);
}

void ProcessPage_MuteButtonCustomDraw(JSB_UI_Control_t *pControl)
{
  if (Process_UserMuted)
    DrawRectAroundControl(pControl);
}

void ProcessPage_HomeButtonPressed(JSB_UI_Control_t *pControl)
{
  JSB_UI_Page_PressUp();
  SetCurrentPage(UI_pHomePage);
}

void CreateProcessPage()
{
  uint32_t Y;
  uint32_t YSpacing = 8;
  uint32_t ButtonWidth = 208; // Leave room for finger to access ends of slider.

  UI_pProcessPage = JSB_UI_CreatePage();

  Y = 0;
  pProcessPage_TitleButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(0, Y, 240, 32), "Process", 0, Page_TitleColour, NULL, 1, NULL, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pProcessPage, pProcessPage_TitleButton);
  //
  Y += 32 + YSpacing;
  pProcessPage_GainControl = JSB_UI_CreateControl(ctHorizontalSlider, JSB_UI_Rect(120 - ButtonWidth / 2, Y, ButtonWidth, 80), "Gain", Page_SliderForegroundColor, Page_SliderBackgroundColor, &Process_Gain, 1, &ProcessPage_GainControlChanged, ProcessPage_GainControlChanged, NULL);
  JSB_UI_AddControlToPage(UI_pProcessPage, pProcessPage_GainControl);
  //
  Y += 80 + YSpacing;
  pProcessPage_DefaultGainControl = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(120 - ButtonWidth / 2, Y, ButtonWidth, 32), "Default gain", 0, Page_ButtonColor, NULL, 1, ProcessPage_DefaultGainControlPressed, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pProcessPage, pProcessPage_DefaultGainControl);
  //
  Y += 32 + YSpacing;
  pProcessPage_MuteButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(120 - ButtonWidth / 2, Y, ButtonWidth, 32), "Mute", 0, Page_ButtonColor, NULL, 1, ProcessPage_MuteButtonPressed, NULL, ProcessPage_MuteButtonCustomDraw);
  JSB_UI_AddControlToPage(UI_pProcessPage, pProcessPage_MuteButton);
  //
  pProcessPage_HomeButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(120 - 100 / 2, 288, 100, 32), "Home", 0, Page_ButtonColor, NULL, 1, &ProcessPage_HomeButtonPressed, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pProcessPage, pProcessPage_HomeButton);
}

///////////////////////////////////////////////////////////////////////////////
// PowerAmplifierPage:

void pPowerAmplifierPage_VolumeControlTweakText(JSB_UI_Control_t *pControl, char *Result, uint32_t MaxResultLength)
{
  snprintf(Result, MaxResultLength, "%s %0.1f dB", pControl->Text, *pControl->pSliderValue);
}

void PowerAmplifierPage_VolumeControlChanged(JSB_UI_Control_t *pControl)
{
  PA_SetVolume(PA_MasterVolume_dB);
}

void PowerAmplifierPage_DefaultVolumeButtonPressed(JSB_UI_Control_t *pControl)
{
  PA_MasterVolume_dB = PA_MasterVolume_dB_Default;
  PA_SetVolume(PA_MasterVolume_dB);
}

void PowerAmplifierPage_MuteButtonPressed(JSB_UI_Control_t *pControl)
{
  if (!PA_UserMuted)
    PA_SetUserMute(1);
  else
  {
    PA_SetVolume(PA_MasterVolume_dB);
  }
}

void PowerAmplifierPage_MuteButtonCustomDraw(JSB_UI_Control_t *pControl)
{
  if (PA_UserMuted)
    DrawRectAroundControl(pControl);
}

void PowerAmplifierPage_HomeButtonPressed(JSB_UI_Control_t *pControl)
{
  JSB_UI_Page_PressUp();
  SetCurrentPage(UI_pHomePage);
}

void CreatePowerAmplifierPage()
{
  uint32_t Y;
  uint32_t YSpacing = 8;
  uint32_t ButtonWidth = 208; // Leave room for finger to access ends of slider.

  UI_pPowerAmplifierPage = JSB_UI_CreatePage();

  Y = 0;
  pPowerAmplifierPage_TitleButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(0, Y, 240, 32), "Power amplifier", 0, Page_TitleColour, NULL, 1, NULL, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pPowerAmplifierPage, pPowerAmplifierPage_TitleButton);
  //
  Y += 32 + YSpacing;
  pPowerAmplifierPage_VolumeControl = JSB_UI_CreateControl(ctHorizontalSlider, JSB_UI_Rect(120 - ButtonWidth / 2, Y, ButtonWidth, 80), "Volume", Page_SliderForegroundColor, Page_SliderBackgroundColor, &PA_MasterVolume_dB, 1, &PowerAmplifierPage_VolumeControlChanged, PowerAmplifierPage_VolumeControlChanged, NULL);
    JSB_UI_AddControlToPage(UI_pPowerAmplifierPage, pPowerAmplifierPage_VolumeControl);
  pPowerAmplifierPage_VolumeControl->SliderMinValue = PA_MasterVolume_dB_Min;
  pPowerAmplifierPage_VolumeControl->SliderMaxValue = PA_MasterVolume_dB_Max;
  pPowerAmplifierPage_VolumeControl->pTweakTextHandler = pPowerAmplifierPage_VolumeControlTweakText;
  //
  Y += 80 + YSpacing;
  pPowerAmplifierPage_DefaultVolumeButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(120 - ButtonWidth / 2, Y, ButtonWidth, 32), "Default volume", 0, Page_ButtonColor, NULL, 1, PowerAmplifierPage_DefaultVolumeButtonPressed, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pPowerAmplifierPage, pPowerAmplifierPage_DefaultVolumeButton);
  //
  Y += 32 + YSpacing;
  pPowerAmplifierPage_MuteButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(120 - ButtonWidth / 2, Y, ButtonWidth, 32), "Mute", 0, Page_ButtonColor, NULL, 1, PowerAmplifierPage_MuteButtonPressed, NULL, PowerAmplifierPage_MuteButtonCustomDraw);
  JSB_UI_AddControlToPage(UI_pPowerAmplifierPage, pPowerAmplifierPage_MuteButton);
  //
  pPowerAmplifierPage_HomeButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(120 - 100 / 2, 288, 100, 32), "Home", 0, Page_ButtonColor, NULL, 1, &PowerAmplifierPage_HomeButtonPressed, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pPowerAmplifierPage, pPowerAmplifierPage_HomeButton);
}

///////////////////////////////////////////////////////////////////////////////

void BluetoothOutPage_OffButtonPressed(JSB_UI_Control_t *pControl)
{
  BluetoothOut_SetOffOn(0);
}

void BluetoothOutPage_OnButtonPressed(JSB_UI_Control_t *pControl)
{
  BluetoothOut_SetOffOn(1);
}

void BluetoothOutPage_OffOnButtonCustomDraw(JSB_UI_Control_t *pControl)
{
  if (pControl == pBluetoothOutPage_OffButton)
  {
    if (BluetoothOut_On)
      return;
  }
  else if(pControl == pBluetoothOutPage_OnButton)
  {
    if (!BluetoothOut_On)
      return;
  }

  DrawRectAroundControl(pControl);
}

void BluetoothOutPage_HomeButtonPressed(JSB_UI_Control_t *pControl)
{
  JSB_UI_Page_PressUp();
  SetCurrentPage(UI_pHomePage);
}

void CreateBluetoothOutPage()
{
  uint32_t Y;
  uint32_t YSpacing = 8;
  UI_pBluetoothOutPage = JSB_UI_CreatePage();

  Y = 0;
  pBluetoothOutPage_TitleButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(0, Y, 240, 32), "Bluetooth out", 0, Page_TitleColour, NULL, 1, NULL, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pBluetoothOutPage, pBluetoothOutPage_TitleButton);
  //
  Y += 32 + YSpacing;
  pBluetoothOutPage_OffButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(2, Y, 114, 32), "Off", 0, Page_ButtonColor, NULL, 1, &BluetoothOutPage_OffButtonPressed, NULL, BluetoothOutPage_OffOnButtonCustomDraw);
  JSB_UI_AddControlToPage(UI_pBluetoothOutPage, pBluetoothOutPage_OffButton);
  pBluetoothOutPage_OnButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(124, Y, 114, 32), "On", 0, Page_ButtonColor, NULL, 1, &BluetoothOutPage_OnButtonPressed, NULL, BluetoothOutPage_OffOnButtonCustomDraw);
  JSB_UI_AddControlToPage(UI_pBluetoothOutPage, pBluetoothOutPage_OnButton);
  //
  pBluetoothOutPage_HomeButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(120 - 100 / 2, 288, 100, 32), "Home", 0, Page_ButtonColor, NULL, 1, &BluetoothOutPage_HomeButtonPressed, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pBluetoothOutPage, pBluetoothOutPage_HomeButton);
}

///////////////////////////////////////////////////////////////////////////////

void WiFiPage_OffButtonPressed(JSB_UI_Control_t *pControl)
{
  WiFi_SetOffOn(0);
}

void WiFiPage_OnButtonPressed(JSB_UI_Control_t *pControl)
{
  WiFi_SetOffOn(1);
}

void WiFiPage_OffOnButtonCustomDraw(JSB_UI_Control_t *pControl)
{
  if (pControl == pWiFiPage_OffButton)
  {
    if (WiFi_On)
      return;
  }
  else if(pControl == pWiFiPage_OnButton)
  {
    if (!WiFi_On)
      return;
  }

  DrawRectAroundControl(pControl);
}

void WiFiPage_HomeButtonPressed(JSB_UI_Control_t *pControl)
{
  JSB_UI_Page_PressUp();
  SetCurrentPage(UI_pHomePage);
}

void CreateWiFiPage()
{
  uint32_t Y;
  uint32_t YSpacing = 8;
  UI_pWiFiPage = JSB_UI_CreatePage();

  Y = 0;
  pWiFiPage_TitleButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(0, Y, 240, 32), "WiFI", 0, Page_TitleColour, NULL, 1, NULL, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pWiFiPage, pWiFiPage_TitleButton);
  //
  Y += 32 + YSpacing;
  pWiFiPage_OffButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(2, Y, 114, 32), "Off", 0, Page_ButtonColor, NULL, 1, &WiFiPage_OffButtonPressed, NULL, WiFiPage_OffOnButtonCustomDraw);
  JSB_UI_AddControlToPage(UI_pWiFiPage, pWiFiPage_OffButton);
  pWiFiPage_OnButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(124, Y, 114, 32), "On", 0, Page_ButtonColor, NULL, 1, &WiFiPage_OnButtonPressed, NULL, WiFiPage_OffOnButtonCustomDraw);
  JSB_UI_AddControlToPage(UI_pWiFiPage, pWiFiPage_OnButton);
  //
  pWiFiPage_HomeButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(120 - 100 / 2, 288, 100, 32), "Home", 0, Page_ButtonColor, NULL, 1, &WiFiPage_HomeButtonPressed, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pWiFiPage, pWiFiPage_HomeButton);
}

///////////////////////////////////////////////////////////////////////////////

void InfoPage_RadioInfoButtonPressed(JSB_UI_Control_t *pControl)
{
  JSB_UI_Page_PressUp();
  SetCurrentPage(UI_pRadioInfoPage);
}

void InfoPage_HomeButtonPressed(JSB_UI_Control_t *pControl)
{
  JSB_UI_Page_PressUp();
  SetCurrentPage(UI_pHomePage);
}

void InfoPage_DebugInfoButtonPressed(JSB_UI_Control_t *pControl)
{
  JSB_UI_Page_PressUp();
  SetCurrentPage(UI_pDebugInfoPage);
}

void InfoPage_CustomDraw(JSB_UI_Page_t *pPage)
{
  uint8_t PA_ErrorStatus;
  char S[200];
  uint32_t LineIndex = 3;

  if (InputChannelIsSPDIF())
  {
    sprintf(S, "Input: SPDIF %d", GetSPDIFInputChannel(InputChannel));
    ILI9341_DrawTextAtXY(S, 0, LineIndex++ * ILI9341_GetFontYSpacing(), tpLeft);
  }
  else
  {
    switch (InputChannel)
    {
      case icNone:
        sprintf(S, "Input: None");
        break;
      case icSi468x_DAB:
        sprintf(S, "Input: Si4648 DAB radio");
        break;
      case icSi468x_FM:
        sprintf(S, "Input: Si4648 FM radio");
        break;
      case icAR1010_FM:
        sprintf(S, "Input: AR1010 FM radio");
        break;
      case icAuxIn:
        sprintf(S, "Input: Aux in");
        break;
      default:
        Error_Handler();
        break;
    }

    ILI9341_DrawTextAtXY(S, 0, LineIndex++ * ILI9341_GetFontYSpacing(), tpLeft);
  }

  if (InputChannel == icSi468x_DAB)
  {
    sprintf(S, "DAB service: %s", DAB_ServiceName);
    ILI9341_DrawTextAtXY(S, 0, LineIndex++ * ILI9341_GetFontYSpacing(), tpLeft);
  }

  if (InputChannelIsSPDIF())
  {
    uint8_t SPDIFRX_NumTransitionErrors, SPDIFRX_NumFrameErrors, SPDIFRX_NumSyncErrors;
    SPDIFRX_GetNumErrors(&SPDIFRX_NumTransitionErrors, &SPDIFRX_NumFrameErrors, &SPDIFRX_NumSyncErrors);

    sprintf(S, "Num SPDIF errors: %d", SPDIFRX_NumTransitionErrors + SPDIFRX_NumFrameErrors + SPDIFRX_NumSyncErrors);
    ILI9341_DrawTextAtXY(S, 0, LineIndex++ * ILI9341_GetFontYSpacing(), tpLeft);
  }

  sprintf(S, "Input sample rate: %u", (unsigned int)GetInputSampleRate());
  ILI9341_DrawTextAtXY(S, 0, LineIndex++ * ILI9341_GetFontYSpacing(), tpLeft);

  sprintf(S, "Process gain: %0.2f", Process_Gain);
  ILI9341_DrawTextAtXY(S, 0, LineIndex++ * ILI9341_GetFontYSpacing(), tpLeft);

  sprintf(S, "Process muted: %s", BooleanToNoYes(Process_UserMuted));
  ILI9341_DrawTextAtXY(S, 0, LineIndex++ * ILI9341_GetFontYSpacing(), tpLeft);

  sprintf(S, "Output sample rate: %u", (unsigned int)OutputSampleRate);
  ILI9341_DrawTextAtXY(S, 0, LineIndex++ * ILI9341_GetFontYSpacing(), tpLeft);

  sprintf(S, "PA master volume: %0.1f", PA_MasterVolume_dB);
  ILI9341_DrawTextAtXY(S, 0, LineIndex++ * ILI9341_GetFontYSpacing(), tpLeft);

  sprintf(S, "PA muted: %s", BooleanToNoYes(PA_UserMuted));
  ILI9341_DrawTextAtXY(S, 0, LineIndex++ * ILI9341_GetFontYSpacing(), tpLeft);

  PA_ErrorStatus = PA_GetErrorStatus();
  sprintf(S, "PA error status: %2X", PA_ErrorStatus);
  ILI9341_DrawTextAtXY(S, 0, LineIndex++ * ILI9341_GetFontYSpacing(), tpLeft);

  sprintf(S, "Bluetooth out: %s", BooleanToOffOn(BluetoothOut_On));
  ILI9341_DrawTextAtXY(S, 0, LineIndex++ * ILI9341_GetFontYSpacing(), tpLeft);
}

void CreateInfoPage()
{
  UI_pInfoPage = JSB_UI_CreatePage();
  UI_pInfoPage->ContinuousRefresh = 1;

  UI_pInfoPage->pCustomDrawHandler = &InfoPage_CustomDraw;

  pInfoPage_TitleButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(0, 0, 240, 32), "Info", 0, Page_TitleColour, NULL, 1, NULL, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pInfoPage, pInfoPage_TitleButton);
  //
  pInfoPage_RadioInfoButton = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(30, 304, 60, 32), "Radio", 0, Page_ButtonColor, NULL, 1, &InfoPage_RadioInfoButtonPressed, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pInfoPage, pInfoPage_RadioInfoButton);
  //
  pInfoPage_HomeButton = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(120, 304, 100, 32), "Home", 0, Page_ButtonColor, NULL, 1, &InfoPage_HomeButtonPressed, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pInfoPage, pInfoPage_HomeButton);
  //
//  pInfoPage_DebugInfoButton = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(210, 304, 60, 32), "Debug", 0, Page_ButtonColor, NULL, 1, &InfoPage_DebugInfoButtonPressed, NULL, NULL);
//  JSB_UI_AddControlToPage(UI_pInfoPage, pInfoPage_DebugInfoButton);
}

///////////////////////////////////////////////////////////////////////////////

void RadioInfoPage_BackButtonPressed(JSB_UI_Control_t *pControl)
{
  JSB_UI_Page_PressUp();
  SetCurrentPage(UI_pInfoPage);
}

void RadioInfoPage_CustomDraw(JSB_UI_Page_t *pPage)
{
  char S[200];
  uint32_t LineIndex = 3;
  JSB_FM_RSQ_Status_t FM_RSQ_Status;
  uint8_t StereoSeparation;
  DAB_DigRad_Status DAB_Status;

  switch (InputChannel)
  {
    case icAR1010_FM:
      sprintf(S, "Radio: AR1010 FM");
      ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);

      sprintf(S, "Frequency: %0.1f MHz", JSB_AR1010_get_frequency());
      ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);

      sprintf(S, "RSSI: %d", JSB_AR1010_get_RSSI());
      ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);

      sprintf(S, "Stereo: %s", BooleanToNoYes(JSB_AR1010_is_stereo()));
      ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);

      break;

    case icSi468x_FM:
      JSB_si468x_FM_get_rsq_status(&FM_RSQ_Status);
      StereoSeparation = JSB_si468x_FM_get_stereo_separation();

      sprintf(S, "Radio: Si4684 FM");
      ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);

      sprintf(S, "Frequency: %0.1f MHz", FM_RSQ_Status.Frequency);
      ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);

      sprintf(S, "RSSI: %d dBuV", FM_RSQ_Status.RSSI);
      ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);

      sprintf(S, "SNR: %d dB", FM_RSQ_Status.SNR);
      ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);

      sprintf(S, "Multi-path: %d", FM_RSQ_Status.MultiPath);
      ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);

      sprintf(S, "Stereo separation: %d", StereoSeparation);
      ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);

      break;

    case icSi468x_DAB:
      si468x_DAB_get_digrad_status(&DAB_Status);

      sprintf(S, "Radio: Si4684 DAB");
      ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);

      sprintf(S, "Service: %s", DAB_ServiceName);
      ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);

      sprintf(S, "RSSI: %d", (int8_t)DAB_Status.data[2]);
      ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);

      sprintf(S, "SNR: %d dB", (int8_t)DAB_Status.data[3]);
      ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);

      sprintf(S, "FIC quality: %d", (int8_t)DAB_Status.data[4]);
      ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);

      sprintf(S, "CNR: %d", (int8_t)DAB_Status.data[5]);
      ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);

      break;

    default:
      sprintf(S, "Radio: None");
      ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);
      break;
  }
}

void CreateRadioInfoPage()
{
  UI_pRadioInfoPage = JSB_UI_CreatePage();
  UI_pRadioInfoPage->ContinuousRefresh = 1;

  UI_pRadioInfoPage->pCustomDrawHandler = &RadioInfoPage_CustomDraw;

  pRadioInfoPage_TitleButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(0, 0, 240, 32), "Radio info", 0, Page_TitleColour, NULL, 1, NULL, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pRadioInfoPage, pRadioInfoPage_TitleButton);
  //
  pRadioInfoPage_BackButton = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(120, 304, 100, 32), "Back", 0, Page_ButtonColor, NULL, 1, &RadioInfoPage_BackButtonPressed, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pRadioInfoPage, pRadioInfoPage_BackButton);
}

///////////////////////////////////////////////////////////////////////////////

void DebugInfoPage_BackButtonPressed(JSB_UI_Control_t *pControl)
{
  JSB_UI_Page_PressUp();
  SetCurrentPage(UI_pInfoPage);
}

char S[200],RegisterValue_str[8];
uint32_t LineIndex = 3;

//static void DrawADCRegisters()
//{
//  sprintf(S, "ADC state");
//  ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);
//
//  uint8_t RegisterAddress=0;
//  for (uint8_t RegisterGroup=0; RegisterGroup<8; ++RegisterGroup)
//  {
//    S[0]='\0';
//    for (uint8_t RegisterOffset=0; RegisterOffset<10; ++RegisterOffset)
//    {
//      uint8_t RegisterValue=ADC_RegisterRead(RegisterAddress);
//      sprintf(RegisterValue_str,"%02x",RegisterValue);
//      if (RegisterOffset!=0)
//        strcat(S," ");
//      strcat(S,RegisterValue_str);
//      ++RegisterAddress;
//    }
//    ILI9341_DrawTextAtXY(S, 0, LineIndex++ *ILI9341_GetFontYSpacing(), tpLeft);
//  }
//}

void DebugInfoPage_CustomDraw(JSB_UI_Page_t *pPage)
{
  // DrawADCRegisters();
}

void CreateDebugInfoPage()
{
  UI_pDebugInfoPage = JSB_UI_CreatePage();
  UI_pDebugInfoPage->ContinuousRefresh = 1;

  UI_pDebugInfoPage->pCustomDrawHandler = &DebugInfoPage_CustomDraw;

  pDebugInfoPage_TitleButton = JSB_UI_CreateControl(ctButton, JSB_UI_Rect(0, 0, 240, 32), "Debug info", 0, Page_TitleColour, NULL, 1, NULL, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pDebugInfoPage, pDebugInfoPage_TitleButton);
  //
  pDebugInfoPage_BackButton = JSB_UI_CreateControl(ctButton, JSB_UI_RectFromCentrePoint(120, 304, 100, 32), "Back", 0, Page_ButtonColor, NULL, 1, &DebugInfoPage_BackButtonPressed, NULL, NULL);
  JSB_UI_AddControlToPage(UI_pDebugInfoPage, pDebugInfoPage_BackButton);
}

///////////////////////////////////////////////////////////////////////////////
// IR:

void ProcessIR()
{
  uint8_t IR_Address, IR_Command, IR_Toggle, IR_Error;
  static uint8_t IR_PreviousToggle = -1;

  if (JSB_IR_RC5_GetIfAvailable(&IR_Address, &IR_Command, &IR_Toggle, &IR_Error))
  {
    if (!IR_Error)
    {
      if (IR_Address == 16)
      {
        if (IR_Command == 17) // Volume down.
          PA_SetVolume(Clamp_float(PA_MasterVolume_dB - 1.0f, PA_MasterVolume_dB_Min, PA_MasterVolume_dB_Max));
        else if (IR_Command == 16) // Volume up.
          PA_SetVolume(Clamp_float(PA_MasterVolume_dB + 1.0f, PA_MasterVolume_dB_Min, PA_MasterVolume_dB_Max));
      }

      if (IR_Address == 20)
      {
        if (IR_Command == 50) // Left.
          Process_SetGain(Process_Gain - 0.01f);
        else if (IR_Command == 52) // Right.
          Process_SetGain(Process_Gain + 0.01f);
        else if (IR_Command == 33) // LeftBar.
          Process_SetGain(0.1f);
        else if (IR_Command == 32) // RightBar.
          Process_SetGain(Process_Gain_Default);
      }

      if (IR_Toggle != IR_PreviousToggle)
      {
        if ((IR_Address == 16) && (IR_Command == 13)) // 'Mute'
        {
          SetUserMute(!PA_UserMuted);
        }

        if ((IR_Address == 16) && (IR_Command == 12)) // 'Power'
        {
          SetPowerOn(!PowerOn);
        }

        if (IR_Address == 17)
        {
          if (IR_UseChannelButtonsToControlRadio)
          {
            if (IR_Command == 33) // Channel down.
            {
              Radio_ServiceToTune = sttPrevious;
            }
            else if (IR_Command == 32) // Channel up.
            {
              Radio_ServiceToTune = sttNext;
            }
          }
          else
          {
            if (IR_Command == 33) // Channel down.
            {
              if (InputChannel > icNone)
                SetInputChannel(InputChannel - 1);
            }
            else if (IR_Command == 32) // Channel up.
            {
              if (InputChannel < icMax)
                SetInputChannel(InputChannel + 1);
            }
          }
        }

        if (IR_Command == 63)
        {
          if (IR_Address == 4) // '0'
            SetInputChannel(icNone);
          else if (IR_Address == 17) // '1'
            SetInputChannel(icSPDIF0);
          else if (IR_Address == 12) // '2'
            SetInputChannel(icSPDIF1);
          else if (IR_Address == 20) // '3'
            SetInputChannel(icSPDIF2);
          else if (IR_Address == 5) // '4'
            SetInputChannel(icSPDIF3);
          else if (IR_Address == 0) // '6'
            SetInputChannel(icSi468x_DAB);
          else if (IR_Address == 22) // '7'
            SetInputChannel(icSi468x_FM);
          else if (IR_Address == 18) // '8'
            SetInputChannel(icAuxIn);
        }

        if ((IR_Address == 20) && (IR_Command == 53)) // Play
          SetUserMute(0);

        if ((IR_Address == 20) && (IR_Command == 48)) // Pause
          SetUserMute(!PA_UserMuted);

        if ((IR_Address == 20) && (IR_Command == 54)) // Stop
          SetUserMute(1);

        if ((IR_Address == 20) && (IR_Command == 59)) // 'BACK'
          SetCurrentPage(UI_pHomePage);

        if ((IR_Address == 20) && (IR_Command == 28)) // 'OPT+'
          PA_SetVolume(0.0f);

        if ((IR_Address == 17) && (IR_Command == 15)) // 'i' (Info)
          SetCurrentPage(UI_pInfoPage);

        if ((IR_Address == 16) && (IR_Command == 63)) // Speaker?
          BluetoothOut_SetOffOn(!BluetoothOut_On);

        if ((IR_Address == 17) && (IR_Command == 46)) // 'TV/RADIO'
        {
          if (InputChannelIsRadio())
          {
            IR_UseChannelButtonsToControlRadio = 1;
            Radio_ServiceToTune = sttCurrent; // Re-tune radio. Use in case the previous tune didn't work out e.g. DAB was silent.
          }
        }

        if ((IR_Address == 16) && (IR_Command == 38)) // 'SLEEP'
          SetPowerOn(!PowerOn);

        if ((IR_Address == 16) && (IR_Command == 59)) // 'SOURCE'
        {
          IR_UseChannelButtonsToControlRadio = 0;
        }
      }

      IR_PreviousToggle = IR_Toggle;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////

void Go()
{
  int16_t Touch_RawX, Touch_RawY, Touch_RawZ;
  int16_t Touch_ScreenX, Touch_ScreenY;
  uint8_t SomethingPressed;
  uint8_t SPDIFRX_Started = 0;
  uint8_t SPDIFRX_StopRequired = 0;
  uint8_t I2SInput_Started = 0;
  uint8_t I2SInput_StopRequired = 0;
  uint8_t BluetoothOut_StopRequired = 0;
  uint8_t UserButtonPressed = 0;

  PowerOn = 1;

  pGenSPI_Begin = &GenSPI_Begin;
  pGenSPI_End = &GenSPI_End;

  JSB_UI_InitializeDisplay();
  LCD_BacklightOn(1);
  //
  JSB_IR_RC5_Initialize(&htim3);
  //
  ADC_Initialize();
  ADC_SetGain(10.0f); // Match AR1010 output level (set to maximum) to Si4684 FM output level.
  //
  DAC_Initialize();
  //
  Si4684_Initialize(Si468x_MODE_None);
  Radio_ServiceToTune = sttNone;
  //
  AR1010_init();
  AR1010_auto_tune(101.7, 1); // JSB: Was 96.4 (for Guildford). For Sheffield: 88.6 for Radio Sheffield. 92.1 for Radio 3. 101.7 for Classic FM, 97.4 for Hallam FM.
  AR1010_set_volume(18);
  //
  PA_Initialize(0);
  // PA_Initialize(1); // Headphones.

  InternalMute(0);

  CreateHomePage();
  CreateInputPage();
  CreateProcessPage();
  CreatePowerAmplifierPage();
  CreateInfoPage();
  CreateRadioInfoPage();
  CreateDebugInfoPage();
  CreateBluetoothOutPage();
  CreateWiFiPage();

  SetCurrentPage(UI_pHomePage);

  SomethingPressed = 0;

  while(1)
  {
    if (Radio_ServiceToTune != sttNone)
    {
      if (I2SInput_Started) // Don't tune if no I2S else distortion occurs (on DAB, at least).
      {
        switch (InputChannel)
        {
          case icSi468x_FM:
            si468x_FM_tune(92.1); // Hardwired!
            //!!! JSB_si468x_set_mute(0); // Un-mute after station is tuned.
            break;

          case icSi468x_DAB:
            DAB_TuneService(Radio_ServiceToTune, 1);
            //!!! if (DAB_TuneService(Radio_ServiceToTune, 1))
              //!!! JSB_si468x_set_mute(0); // Un-mute after station is tuned.
            break;

          default:
            break;
        }
      }

      Radio_ServiceToTune = sttNone;
    }

    if (InputChannelChanged || BluetoothOutChanged)
    {
      InputChannelChanged = 0;
      BluetoothOutChanged = 0;

      SPDIFRX_StopRequired = 1;
      I2SInput_StopRequired = 1;
      BluetoothOut_StopRequired = 1;
      Radio_ServiceToTune = sttCurrent; // Re-tune DAB after I2S clock changes to prevent distortion.

      PageChanged = 1;

      switch(InputChannel)
      {
        case icAR1010_FM:
          ADC_SelectSource(ADC_Source_AR1010);
          break;

        case icAuxIn:
          ADC_SelectSource(ADC_Source_BlueJack);
          break;

        default:
          break;
      }

      SetLED(LED_Blue, InputChannel == icAuxIn); // Just for fun.
    }

    if (SPDIFRX_Started)
    {
      if (SPDIFRX_HasReceiveErrorOccurred())
      {
        SPDIFRX_StopRequired = 1;
        BluetoothOut_StopRequired = 1; // In case the sample rate has changed.
      }

      if (SPDIFRX_StopRequired)
      {
        SPDIFRX_Stop();
        OutputSampleRate = 0; // Force DAC to be reconfigured so that DMAs are also reconfigured when coming from SPDIF.
        SPDIFRX_Started = 0;
      }
    }

    if (I2SInput_Started)
    {
      if (I2SInput_StopRequired)
      {
        I2SInput_Stop();
        OutputSampleRate = 0; // Force DAC to be reconfigured so that DMAs are also reconfigured when coming from SPDIF.
        I2SInput_Started = 0;
      }
    }

    if (BluetoothOut_Started)
    {
      if (BluetoothOut_StopRequired)
      {
        BluetoothOut_Stop();
        BluetoothOut_Started = 0;
      }
    }

    SPDIFRX_StopRequired = 0;
    I2SInput_StopRequired = 0;
    BluetoothOut_StopRequired = 0;

    if (InputChannelIsSPDIF())
    {
      if (!SPDIFRX_Started)
      {
        I2SBuffers_NumSamples = SPDIFRX_Buffer_NumSamples;
        SPDIFRX_Started = SPDIFRX_Start(hsai_BlockA2.Init.AudioFrequency, GetSPDIFInputChannel(InputChannel), 0, 0);
        PA_ClearErrorStatus();
      }
    }
    else if (InputChannelIsI2S())
    {
      if (!I2SInput_Started)
      {
        if (BluetoothOut_On)
          SetOutputSampleRate_48000(I2SBuffers_NumSamples_WhenBluetoothIsEnabled); // Currently, the ESP32 requires large sample packets so increase sample buffer size.
        else
          SetOutputSampleRate_48000(I2SBuffers_NumSamples_WhenBluetoothIsNotEnabled);
        ClearOutputBuffer();
        InternalMute(0); // 18/6/2024: In case SPDIF left InternalMuteActive on e.g. if input data stopped.
        I2SInput_Started = 1;

        switch(InputChannel)
        {
          case icSi468x_FM:
            Si4684_RequireMode(Si468x_MODE_FM);
            //!!! JSB_si468x_set_mute(1); // Mute until station tuned / re-tuned to prevent distortion / sound blip.
            I2S_SelectSource(I2S_Source_Si4684);
            break;

          case icSi468x_DAB:
            Si4684_RequireMode(Si468x_MODE_DAB);
            //!!! JSB_si468x_set_mute(1); // Mute until station tuned / re-tuned to prevent distortion / sound blip.
            I2S_SelectSource(I2S_Source_Si4684);
            break;

          case icAR1010_FM:
          case icAuxIn:
            I2S_SelectSource(I2S_Source_ADC);
            break;

          default:
            break;
        }
      }
    }

    if (BluetoothOut_On)
    {
      if (!BluetoothOut_Started)
      {
        if (BluetoothOut_Start())
          BluetoothOut_Started = 1;
      }
    }

    if (PageChanged || (pCurrentPage->ContinuousRefresh))
    {
      PageChanged = 0;

      JSB_UI_DrawPage(pCurrentPage);
    }

    LCD_BacklightOn(PowerOn); // Do this after updating the page in case it was changed whilst the display was off.

    if (XPT2046_Sample(&Touch_RawX, &Touch_RawY, &Touch_RawZ))
    {
      if (!PowerOn)
      {
        if (!SomethingPressed)
        {
          SetPowerOn(1);
          SomethingPressed = 1;
        }
      }
      else
      {
        XPT2046_ConvertRawToScreen(Touch_RawX, Touch_RawY, &Touch_ScreenX, &Touch_ScreenY);

        if (!SomethingPressed)
          SomethingPressed = JSB_UI_Page_PressDown(pCurrentPage, Touch_ScreenX, Touch_ScreenY);
        JSB_UI_Page_PressMove(pCurrentPage, Touch_ScreenX, Touch_ScreenY);
      }
    }
    else
    {
      if (SomethingPressed)
      {
        JSB_UI_Page_PressUp();
        SomethingPressed = 0;
      }
    }

    ProcessIR();

    if (IsUserButtonPressed())
    {
      if (!UserButtonPressed)
      {
        UserButtonPressed = 1;
        if (InputChannelIsRadio())
          Radio_ServiceToTune = sttNext;
      }
    }
    else
    {
      UserButtonPressed = 0;
    }

    osDelay(1);
  }
}

///////////////////////////////////////////////////////////////////////////////
