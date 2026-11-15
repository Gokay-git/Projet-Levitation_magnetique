/* main.h de remplacement : definit le minimum de HAL STM32 pour compiler le firmware sur PC */
#ifndef __MAIN_H
#define __MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK = 0, HAL_ERROR } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef struct { volatile uint32_t BSRR; } GPIO_TypeDef;
extern GPIO_TypeDef gA, gB, gC;
#define GPIOA (&gA)
#define GPIOB (&gB)
#define GPIOC (&gC)
#define B1_Pin 0x2000u
#define B1_GPIO_Port GPIOC
#define LD2_Pin 0x0020u
#define LD2_GPIO_Port GPIOA
#define Y1_B_Pin 0x0040u
#define Y1_B_GPIO_Port GPIOA
#define Y1_A_Pin 0x0080u
#define Y1_A_GPIO_Port GPIOA
#define X2_A_Pin 0x0400u
#define X2_A_GPIO_Port GPIOB
#define TIMING_Pin 0x0080u
#define TIMING_GPIO_Port GPIOC
#define X2_B_Pin 0x0008u
#define X2_B_GPIO_Port GPIOB
#define X1_B_Pin 0x0010u
#define X1_B_GPIO_Port GPIOB
#define X1_A_Pin 0x0020u
#define X1_A_GPIO_Port GPIOB
#define Y2_A_Pin 0x0040u
#define Y2_A_GPIO_Port GPIOB
#define Y2_B_Pin 0x0100u
#define Y2_B_GPIO_Port GPIOB
typedef struct { volatile uint32_t PSC, CR1, RCR, ARR, CCR[4]; } TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
#define TIM_CHANNEL_1 0u
#define TIM_CHANNEL_2 4u
#define TIM_CHANNEL_3 8u
#define TIM_CHANNEL_4 12u
#define TIM_CR1_CMS (3u << 5)
#define __HAL_TIM_GET_AUTORELOAD(h) ((h)->Instance->ARR)
#define __HAL_TIM_SET_COMPARE(h, ch, v) ((h)->Instance->CCR[(ch) / 4u] = (v))
typedef struct { int dummy; } ADC_TypeDef;
extern ADC_TypeDef adc1_inst;
#define ADC1 (&adc1_inst)
typedef struct { uint32_t NbrOfConversion; } ADC_InitTypeDef;
typedef struct { ADC_TypeDef *Instance; ADC_InitTypeDef Init; } ADC_HandleTypeDef;
typedef struct { int d; } DMA_HandleTypeDef;
#define DMA_IT_HT 0
#define __HAL_DMA_DISABLE_IT(h, it) ((void)(h))
typedef struct { int d; } UART_HandleTypeDef;
#define ADC_SINGLE_ENDED 0u
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h, uint32_t m);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *h, uint32_t *buf, uint32_t n);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t ch);
uint32_t HAL_RCC_GetPCLK2Freq(void);
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *p, uint16_t n, uint32_t t);
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState s);
void Error_Handler(void);
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
#endif
