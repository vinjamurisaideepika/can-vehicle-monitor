/**
 * CI stand-in for stm32f4xx_hal_msp.c and stm32f4xx_it.c (CubeMX generates
 * both). The MSP functions route pins to peripherals and enable clocks;
 * the IRQ handlers forward interrupts into the HAL.
 */
#include "main.h"

extern CAN_HandleTypeDef  hcan1;
extern TIM_HandleTypeDef  htim6;
extern UART_HandleTypeDef huart2;

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    GPIO_InitTypeDef g = {0};
    if (h->Instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        g.Pin       = GPIO_PIN_2 | GPIO_PIN_3; /* PA2 TX, PA3 RX -> ST-LINK VCP */
        g.Mode      = GPIO_MODE_AF_PP;
        g.Pull      = GPIO_NOPULL;
        g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        g.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &g);
        HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *h)
{
    GPIO_InitTypeDef g = {0};
    if (h->Instance == CAN1) {
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        g.Pin       = GPIO_PIN_11 | GPIO_PIN_12; /* PA11 CAN1_RX, PA12 CAN1_TX */
        g.Mode      = GPIO_MODE_AF_PP;
        g.Pull      = GPIO_NOPULL;
        g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        g.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOA, &g);
        HAL_NVIC_SetPriority(CAN1_TX_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
        HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
        HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *h)
{
    if (h->Instance == TIM6) {
        __HAL_RCC_TIM6_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }
}

/* ------------------------------------------------------ IRQ handlers */
void NMI_Handler(void)        { while (1) { } }
void HardFault_Handler(void)  { while (1) { } }
void MemManage_Handler(void)  { while (1) { } }
void BusFault_Handler(void)   { while (1) { } }
void UsageFault_Handler(void) { while (1) { } }
void SVC_Handler(void)        { }
void DebugMon_Handler(void)   { }
void PendSV_Handler(void)     { }
void SysTick_Handler(void)    { HAL_IncTick(); }

void USART2_IRQHandler(void)    { HAL_UART_IRQHandler(&huart2); }
void TIM6_DAC_IRQHandler(void)  { HAL_TIM_IRQHandler(&htim6); }
void EXTI15_10_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(B1_Pin); }
void CAN1_TX_IRQHandler(void)   { HAL_CAN_IRQHandler(&hcan1); }
void CAN1_RX0_IRQHandler(void)  { HAL_CAN_IRQHandler(&hcan1); }
void CAN1_SCE_IRQHandler(void)  { HAL_CAN_IRQHandler(&hcan1); }
