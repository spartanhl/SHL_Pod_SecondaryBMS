/***********************************************
* @file it.c
* @brief ISR source file
* @author Oliver Moore
* @version 1.4
* @date 03-02-2022
***********************************************/

#include "it.h"
#include "main.h"

extern UART_HandleTypeDef huart2;
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim6;

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/
void NMI_Handler(void) {
	while(1) {}
}

void HardFault_Handler(void) {
	while(1) {}
}

void MemManage_Handler(void) {
	while(1) {}
}

void BusFault_Handler(void) {
	while(1) {}
}

void UsageFault_Handler(void) {
	while(1) {}
}

void SVC_Handler(void) {

}

void DebugMon_Handler(void) {

}

void PendSV_Handler(void) {

}

void SysTick_Handler(void) {
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/
void USART2_IRQHandler(void) {
	HAL_UART_IRQHandler(&huart2);
}

void CAN1_TX_IRQHandler(void) {
	HAL_CAN_IRQHandler(&hcan1);
}

void CAN1_RX0_IRQHandler(void) {
	HAL_CAN_IRQHandler(&hcan1);
}

void CAN1_RX1_IRQHandler(void) {
	HAL_CAN_IRQHandler(&hcan1);
}

void CAN1_SCE_IRQHandler(void) {
	HAL_CAN_IRQHandler(&hcan1);
}

void TIM6_DAC_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim6);
}
