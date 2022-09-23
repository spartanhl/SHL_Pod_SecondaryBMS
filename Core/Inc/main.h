/***********************************************
* @file main.h
* @brief Spartan Hyperloop TinyBMS Testing
* @author Oliver Moore
* @version 1.7
* @date 09-17-2022
***********************************************/

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include "stm32f7xx_hal.h"
#include "TinyBMS.h"

/***************** External Variables ******************/
extern UART_HandleTypeDef huart2;
extern CAN_HandleTypeDef hcan1;
extern uint32_t TinybmsStdID_Request;
extern uint32_t TinybmsStdID_Response;
extern uint8_t rx_buffer[500];

/*************** Function Prototypes ***************/
void UART_Test_CMD(void);
void CAN_Test_CMD(void);
void TinyBMS_MonitorCharging_UART(void);
void TinyBMS_MonitorDischarging_UART(void);
void TinyBMS_MonitorCharging_CAN(void);
void TinyBMS_MonitorDischarging_CAN(void);
uint8_t TinyBMS_Init_UART(void);
uint8_t TinyBMS_Init_CAN(void);
void ElCon_SendMsg(void);
void SystemClock_Config_HSI(uint8_t clock_freq);
void GPIO_Init(void);
void UART_Init(void);
void TIM_Init(void);
void CAN_Init(uint8_t can_bitrate);
void CAN_Filter_Config(void);
void CAN_Begin(void);
void CAN1_Tx(uint8_t device, uint8_t* message, uint8_t len);
void LED_Manage_Output(uint8_t led_num);
void Error_Handler(void);

/*************** Macros ***************/
#define SYS_CLOCK_FREQ_50MHZ 	50
#define SYS_CLOCK_FREQ_84MHZ 	84
#define SYS_CLOCK_FREQ_120MHZ 	120

#define FALSE 					0
#define TRUE 					1

#define NO						0
#define YES						1

#define NEITHER					0
#define ONLYEVENTS				1
#define ONLYSTATS				2
#define BOTH					3

#define TINYBMS					0
#define ELCONCHARGER2			2

#define MENU_UART_CMD_TEST				1
#define MENU_CAN_CMD_TEST				2
#define MENU_MONITOR_CHARGE_UART		3
#define MENU_MONITOR_DISCHARGE_UART		4
#define MENU_MONITOR_CHARGE_CAN			5
#define MENU_MONITOR_DISCHARGE_CAN		6
#define MENU_QUIT						7

/* CAN Speed Macros based on SYSCLK freq */
#define CANBITRATE_1MBIT_50MHZ			0
#define CANBITRATE_500KBIT_50MHZ 		1
#define CANBITRATE_250KBIT_50MHZ 		2
#define CANBITRATE_125KBIT_50MHZ 		3

/* Secondary Battery Pack Macros */
#define NUMCELLS_SECONDARY 				7 		//Secondary Pack has 7 Cells in Series


/************  GPIO Macros ************/
/* HSE Bypass */
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH

/* Nucleo-144 onboard LED1,2,3 */
#define LED1_Pin GPIO_PIN_0
#define LED2_Pin GPIO_PIN_7
#define LED3_Pin GPIO_PIN_14
#define LED_GPIO_Port GPIOB

/* Nucleo-144 onboard user button */
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC

/* Serial Wire Debug (SWD) */
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SW0_Pin GPIO_PIN_3
#define SW0_GPIO_Port GPIOB

/* USART2: PA3 PD5 for TinyBMS communication */
#define USART2_TX_Pin GPIO_PIN_5
#define USART2_RX_Pin GPIO_PIN_6
#define USART2_GPIO_Port GPIOD

#define TinyBMS_TX_Pin USART2_TX_Pin
#define TinyBMS_RX_Pin USART2_RX_Pin
#define TinyBMS_GPIO_Port USART2_GPIO_Port

/* USART3: PD8 PD9 for ST-LINK Debug (printf ITM) */
#define USART3_TX_Pin GPIO_PIN_8
#define USART3_RX_Pin GPIO_PIN_9
#define USART3_GPIO_Port GPIOD

#define STLK_TX_Pin USART3_TX_Pin
#define STLK_RX_Pin USART3_RX_Pin
#define STLK_GPIO_Port USART3_GPIO_Port

/* CAN1 */
#define CAN1_RX_PIN	GPIO_PIN_0
#define CAN1_TX_PIN	GPIO_PIN_1
#define CAN1_GPIO_Port GPIOD
