/* CI stand-in for the CubeMX-generated main.h (same pin labels). */
#ifndef __MAIN_H
#define __MAIN_H
#include "stm32f4xx_hal.h"
void Error_Handler(void);
#define B1_Pin        GPIO_PIN_13
#define B1_GPIO_Port  GPIOC
#define LD2_Pin       GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#endif
