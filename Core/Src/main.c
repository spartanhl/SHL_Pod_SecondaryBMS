/***********************************************
* @file main.c
* @brief Spartan Hyperloop TinyBMS Testing
* @author Oliver Moore
* @version 1.4
* @date 03-02-2022
***********************************************/
#include "main.h"

CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
TIM_HandleTypeDef htim6;
CAN_RxHeaderTypeDef RxHeader;

extern uint32_t TinybmsStdID_Request;
extern uint32_t TinybmsStdID_Response;

/******* Globals *******/
uint8_t led_num = 0;
uint8_t bms_opmode = 0;

uint16_t maxCellVoltage = 0, minCellVoltage = 0;
uint32_t initialSOC = 0;
float initialPackVoltage = 0, initialPackCurrent = 0;

int main(void) {
	/* Resets all peripherals, initializes the flash interface and Systick. */
	HAL_Init();

	/* Configure SYSCLK to 50MHZ */
	SystemClock_Config_HSI(SYS_CLOCK_FREQ_50MHZ);

	/* Initialize all configured peripherals */
	GPIO_Init();
	UART_Init();
	TIM_Init();
	CAN_Init(CANBITRATE_500KBIT_50MHZ);
	CAN_Filter_Config();
	CAN_Begin();

	//UART_Test_API();
	//CAN_Test_API();

	/* Application State Machine */
	//Todo:
  	while(1) {
		switch(bms_opmode) {
		case MONITOR_CHARGING:
			//TinyBMS Init
			if(TinyBMS_Init() != CMD_SUCCESS) {
				printf("TinyBMS Init failed.\n");
				Error_Handler();
			}

			//Start the Timer (Interrupt mode - Non-Blocking)
			//Timer is used to send a message to the charger every 1 second
			HAL_TIM_Base_Start_IT(&htim6);

			//Begin and Monitor Charging
			TinyBMS_MonitorCharging();

			//Stop the Timer
			HAL_TIM_Base_Stop_IT(&htim6);
			break;

		case MONITOR_OPERATION:
			//TinyBMS Init
			if(TinyBMS_Init() != CMD_SUCCESS) {
				printf("TinyBMS Init failed.\n");
				Error_Handler();
			}

			TinyBMS_MonitorOperation();
			break;

		default:
			Error_Handler();
		}
  	}
}

void UART_Test_API(void) {
	/*
	int8_t option = 0;
	uint8_t rl = 0, pl = 0;
	uint16_t addr = 0x00;
	//uint16_t addr[100];
	uint16_t data[100];
	*/

	/*** Uncomment to test specific TinyBMS API -- Hangs in while loop unless success is returned ****/

	//while(TinyBMS_UART_ACK(&huart2) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ReadRegBlock(&huart2, rl, addr) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ReadRegIndividual(&huart2, pl, addr[]) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_WriteRegBlock(&huart2, pl, addr, data[]) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_WriteRegIndividual(&huart2, pl, addr[], data[]) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ReadRegBlockMODBUS(&huart2, addr, rl) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_WriteRegBlockMODBUS(&huart2, addr, rl, pl, data[]) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ResetClearEventsStatistics(&huart2, option) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ReadNewestEvents(&huart2) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ReadAllEvents(&huart2) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ReadBatteryPackVoltage(&huart2) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ReadBatteryPackCurrent(&huart2) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ReadBatteryPackMaxCellVoltage(&huart2) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ReadBatteryPackMinCellVoltage(&huart2) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ReadOnlineStatus(&huart2) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ReadLifetimeCounter(&huart2) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ReadEstimatedSOCValue(&huart2) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ReadDeviceTemperatures(&huart2) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ReadBatteryPackCellVoltages(&huart2) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ReadSettingsValues(&huart2, option, rl) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ReadVersion(&huart2) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ReadVersionExtended(&huart2) != CMD_SUCCESS) {}
	//while(TinyBMS_UART_ReadCalcSpeedDistanceLeftEstTimeLeft(&huart2) != CMD_SUCCESS) {} */
}

void CAN_Test_API(void) {
	/*
	int8_t option = 0;
	uint8_t rl = 0, pl = 0;
	uint16_t addr = 0x00;
	//uint16_t addr[100];
	uint16_t data[100];
	uint8_t nodeID = 0x01;
	*/

	/*** Uncomment to test specific TinyBMS API -- Hangs in while() unless success is returned ****/

	//while(TinyBMS_CAN_ResetClearEventsStatistics(&hcan1, option) != CMD_SUCCESS) {}
	//while(TinyBMS_CAN_ReadRegBlock(&hcan1, rl, addr) != CMD_SUCCESS) {}
	//while(TinyBMS_CAN_WriteRegBlock(&hcan1, rl, addr, data[]) != CMD_SUCCESS) {}
	//while(TinyBMS_CAN_ReadNewestEvents(&hcan1) != CMD_SUCCESS) {}
	//while(TinyBMS_CAN_ReadAllEvents(&hcan1) != CMD_SUCCESS) {}
	//while(TinyBMS_CAN_ReadBatteryPackVoltage(&hcan1) != CMD_SUCCESS) {}
	//while(TinyBMS_CAN_ReadBatteryPackCurrent(&hcan1) != CMD_SUCCESS) {}
	//while(TinyBMS_CAN_ReadBatteryPackMaxCellVoltage(&hcan1) != CMD_SUCCESS) {}
	//while(TinyBMS_CAN_ReadBatteryPackMinCellVoltage(&hcan1) != CMD_SUCCESS) {}
	//while(TinyBMS_CAN_ReadOnlineStatus(&hcan1) != CMD_SUCCESS) {}
	//while(TinyBMS_CAN_ReadLifetimeCounter(&hcan1) != CMD_SUCCESS) {}
	//while(TinyBMS_CAN_ReadEstimatedSOCValue(&hcan1) != CMD_SUCCESS) {}
	//while(TinyBMS_CAN_ReadDeviceTemperatures(&hcan1) != CMD_SUCCESS) {}
	//while(TinyBMS_CAN_ReadBatteryPackCellVoltages(&hcan1) != CMD_SUCCESS) {}
	//while(TinyBMS_CAN_ReadSettingsValues(&hcan1, option, rl) != CMD_SUCCESS) {}
	//while(TinyBMS_CAN_ReadVersion(&hcan1) != CMD_SUCCESS) {}
	//while(TinyBMS_CAN_ReadCalcSpeedDistanceLeftEstTimeLeft(&hcan1) != CMD_SUCCESS) {}
	//while(TinyBMS_CAN_ReadNodeID(&hcan1) != CMD_SUCCESS) {}
	//while(TinyBMS_CAN_WriteNodeID(&hcan1, nodeID) != CMD_SUCCESS) {}
}

void TinyBMS_MonitorCharging(void) {

	//Todo:
	//Mostly Placeholder - Modify API to return their respective data
	uint16_t cellv[7] = {};
	uint16_t numDetectedCells = 0;

	while(TinyBMS_CAN_ReadOnlineStatus(&hcan1) == TINYBMS_STATUS_CHARGING) {

		//Verify that all cells are being detected
		numDetectedCells = TinyBMS_CAN_ReadRegBlock(&hcan1, 1, NUMBER_OF_DETECTED_CELLS);
		if(numDetectedCells != NUMCELLS_SECONDARY) {
			printf("Some cells are not being detected!\n");
		}

		//Get voltage of all cells and compare with max/min voltage thresholds
		TinyBMS_CAN_ReadBatteryPackCellVoltages(&hcan1);
		for(uint8_t i = 0; i < NUMCELLS_SECONDARY; i++) {
			if(cellv[i] < minCellVoltage)  {
				printf("Cell %u is below the minimum voltage threshold!\n", i+1);
			}
			if(cellv[i] > maxCellVoltage) {
				printf("Cell %u is above the maximum voltage threshold!\n", i+1);
			}
		}

		//Check if cells need balancing or are in progress of balancing
		//Regs 51 & 52: BALANCING_DECISION_BITS & REAL_BALANCING_BITS
		TinyBMS_CAN_ReadRegBlock(&hcan1, 2, BALANCING_DECISION_BITS);

		//Check Newest Events
		TinyBMS_CAN_ReadNewestEvents(&hcan1);

		//Check Online Status
		TinyBMS_CAN_ReadOnlineStatus(&hcan1);

		//Check Temperatures
		TinyBMS_CAN_ReadDeviceTemperatures(&hcan1);

		//Get State of Charge
		TinyBMS_CAN_ReadEstimatedSOCValue(&hcan1);

		//Get Pack Voltage
		TinyBMS_CAN_ReadBatteryPackVoltage(&hcan1);

		//Get Pack Current
		TinyBMS_CAN_ReadBatteryPackCurrent(&hcan1);
	}
}

void TinyBMS_MonitorOperation(void) {
	//Todo:
	//Similar to MonitorCharging
}

void ElCon_SendMsg(void) {
	//Triggered from HAL_TIM_PeriodElapsedCallback()
	//Every 1 second, send 8-bytes of data with voltage and current requested to ExtID 0x1806E5F4
	//Todo:
	uint8_t msg[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t len = 8;
	CAN1_Tx(ELCONCHARGER2, msg, len);
}

uint8_t TinyBMS_Init(void) {
	uint8_t retval = CMD_FAILURE;
	//Todo:
	//Mostly Placeholder - Modify API to return their respective data

	//Read CAN NodeID and update to it if required
	TinyBMS_CAN_ReadNodeID(&hcan1);

	//Reset BMS
	TinyBMS_CAN_ResetClearEventsStatistics(&hcan1, TINYBMS_RESET_BMS);

	//Clear Events & Statistics
	TinyBMS_CAN_ResetClearEventsStatistics(&hcan1, TINYBMS_CLEAR_EVENTS);
	TinyBMS_CAN_ResetClearEventsStatistics(&hcan1, TINYBMS_CLEAR_STATS);

	//Confirm BMS Reset by reading Lifetime Counter
	TinyBMS_CAN_ReadLifetimeCounter(&hcan1);

	//Read Version
	TinyBMS_CAN_ReadVersion(&hcan1);

	//Get Min/Max Cell Voltage Thresholds
	minCellVoltage = TinyBMS_CAN_ReadBatteryPackMinCellVoltage(&hcan1);
	maxCellVoltage = TinyBMS_CAN_ReadBatteryPackMaxCellVoltage(&hcan1);

	//Check for any active events
	TinyBMS_CAN_ReadAllEvents(&hcan1);

	//Verify Pack Voltage and Current
	initialPackVoltage = TinyBMS_CAN_ReadBatteryPackVoltage(&hcan1);
	initialPackCurrent = TinyBMS_CAN_ReadBatteryPackCurrent(&hcan1);

	//Get initial State of Charge
	initialSOC = TinyBMS_CAN_ReadEstimatedSOCValue(&hcan1);

	//Check Temperatures
	TinyBMS_CAN_ReadDeviceTemperatures(&hcan1);

	//Verify Online Status is TINYBMS_STATUS_IDLE
	if(TinyBMS_CAN_ReadOnlineStatus(&hcan1) == TINYBMS_STATUS_IDLE) {
		//do nothing
	} else if(TinyBMS_CAN_ReadOnlineStatus(&hcan1) == TINYBMS_STATUS_FAULT) {
		//Check for any active events
		TinyBMS_CAN_ReadAllEvents(&hcan1);
		retval = CMD_FAILURE;
		return retval;
	} else {
		Error_Handler();
	}

	//Settings Registers: 300-301, 303-304, 306-308, 312-320, 328, 330-343
	//					  (30 total settings) (344-399 reserved)
	// rl max is 100 (0x64) registers, but this exceeds the actual total
	TinyBMS_CAN_ReadSettingsValues(&hcan1, TINYBMS_SETTINGS_CURRENT, 30);

	retval = CMD_SUCCESS;
	return retval;
}

void SystemClock_Config_HSI(uint8_t clock_freq) {
	RCC_OscInitTypeDef osc_init = {0};
	RCC_ClkInitTypeDef clk_init = {0};
	uint8_t flash_latency = 0;

	//Using HSI to derive PLL
	osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	osc_init.HSIState = RCC_HSI_ON;
	osc_init.PLL.PLLState = RCC_PLL_ON;
	osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSI;

	switch(clock_freq) {
	case SYS_CLOCK_FREQ_50MHZ: {
		osc_init.PLL.PLLM = 16;
		osc_init.PLL.PLLN = 100;
		osc_init.PLL.PLLP = RCC_PLLP_DIV2;
		osc_init.PLL.PLLQ = 2;

		clk_init.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | \
							  RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2);
		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV2;
		flash_latency = 1;
		break;
	}
	case SYS_CLOCK_FREQ_84MHZ: {
		osc_init.PLL.PLLM = 16;
		osc_init.PLL.PLLN = 168;
		osc_init.PLL.PLLP = RCC_PLLP_DIV2;
		osc_init.PLL.PLLQ = 2;

		clk_init.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | \
							  RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2);
		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV2;
		flash_latency = 2;
		break;
	}
	case SYS_CLOCK_FREQ_120MHZ: {
		osc_init.PLL.PLLM = 16;
		osc_init.PLL.PLLN = 240;
		osc_init.PLL.PLLP = RCC_PLLP_DIV2;
		osc_init.PLL.PLLQ = 2;

		clk_init.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | \
							  RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2);
		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV2;
		flash_latency = 3;
		break;
	}
	default:
		return;
	}

	if(HAL_RCC_OscConfig(&osc_init) != HAL_OK) {
		Error_Handler();
	}

	if(HAL_RCC_ClockConfig(&clk_init, flash_latency) != HAL_OK) {
		Error_Handler();
	}

	//Configure the SYSTICK timer interrupt frequency for every 1ms
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
	//Configure SYSTICK
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	//SYSTICK IRQn interrupt configuration
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	//__HAL_RCC_GPIOH_CLK_ENABLE();

	/* Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, (LED1_Pin | LED2_Pin | LED3_Pin), GPIO_PIN_RESET);

	/* Configure GPIO pin : USER_Btn_Pin */
	GPIO_InitStruct.Pin = USER_Btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	/* Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
	GPIO_InitStruct.Pin = (LED1_Pin | LED2_Pin | LED3_Pin);
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void UART_Init(void) {
	//USART2: PD5 PD6 for TinyBMS communication
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if(HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}

	//USART3: PD8 PD9 for ST-LINK debugging (printf ITM)
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if(HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
}

void TIM_Init(void) {
	//TIM6 - Basic Timer
	//Every 1 Second or 1Hz freq
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 4999;
	htim6.Init.Period = 10000-1;
	if(HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
}

void CAN_Init(uint8_t can_bitrate) {
	/*	 		bxCAN (Basic Extended Controller Area Network) p.1295 of RM0385
	 *  . STM32F746xx has CAN1 and CAN2 peripherals
	 *    - CAN1 has direct access to 512B SRAM while CAN2 does not
	 *  . Supports "Time Triggered Communication" for safety-critical applications
	 *  . Supports CAN 2.0A (standard 11-bit ID) and 2.0B (extended 29-bit ID)
	 *  	. TinyBMS supports CAN2.0A (11-bit ID)
	 *  	. TinyBMS CAN bitrate of 500kbit/s (cannot be changed by user)
	 *  . 3 Tx Mailboxes, 2 Rx FIFOs
	 *  . 28 Filter banks shared between CAN1 and CAN2 for dual CAN
	 *  . Max Bitrate of bxCAN is 1Mbit/s
	 * 	* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	hcan1.Instance = CAN1;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.AutoBusOff = ENABLE;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;

	/* Settings related to CAN bit timings (http://www.bittiming.can-wiki.info/) */
	switch(can_bitrate) {
	/*
	case CANBITRATE_1MBIT_50MHZ:
		* ** 1Mbit/s (max bitrate) @ 50MHz SYSCLK ** *
		//prescaler = 5, num_TQ = 10, Seg1 = 8, Seg2 = 1, Sample point at 90.0, register CAN_BTR = 0x00070004
		hcan1.Init.Prescaler = 5;
		hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
		hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
		hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
		break;
	*/
	/********* TinyBMS only supports 500kbit/s CAN speed *********/
	case CANBITRATE_500KBIT_50MHZ:
		/* ** 500kbit/s @ 50MHz SYSCLK ** */
		//prescaler = 5, num_TQ = 10, Seg1 = 8, Seg2 = 1, Sample point at 90.0, register CAN_BTR = 0x00070009
		hcan1.Init.Prescaler = 10;
		hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
		hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
		hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
		break;
	/*
	case CANBITRATE_250KBIT_50MHZ:
		* ** 250kbit/s @ 50MHz SYSCLK ** *
		//prescaler = 5, num_TQ = 10, Seg1 = 8, Seg2 = 1, Sample point at 90.0, register CAN_BTR = 0x00070013
		hcan1.Init.Prescaler = 20;
		hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
		hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
		hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
		break;
	*/
	/*
	case CANBITRATE_125KBIT_50MHZ:
		* ** 125kbit/s @ 50MHz SYSCLK ** *
		//prescaler = 25, num_TQ = 16, Seg1 = 13, Seg2 = 2, Sample point at 87.5, register CAN_BTR = 0x001c0018
		hcan1.Init.Prescaler = 25;
		hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
		hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
		hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
		break;
	*/
	default:
		Error_Handler();
	}
	if(HAL_CAN_Init(&hcan1) != HAL_OK) {
		Error_Handler();
	}
}

void CAN_Filter_Config(void) {
	/*
	 *  __TinyBMS Related Info:__
	 *  CAN2.0A (11-bit CAN Identifier only)
	 * 	TinyBMS Default Node ID: 0x01 -> StdID: 0x201	     |-CAN StdID-|
	 * 														      |NodeID|
	 *  Request   StdID: 01000(Node ID Default=0x01..0x3F) = 01000 000001 = 010 0000 0001 = 0x201
	 *  Response  StdID: 01001(Node ID Default=0x01..0x3F) = 01001 000001 = 010 0100 0001 = 0x241
	 *	Request  ID Range: 0x201-0x23F
	 *	Response ID Range: 0x241-0x27F
	 *
	 *	__ElCon Charger Related Info:__
	 *  ElCon Secondary Pack Charger Node ID: 0x1806E5F4
	 *
	 *
	 *	Filter Bank 0:	FB0_R1 (32-bit)    ID Reg / ID Reg 1
	 *  				FB0_R2 (32-bit)  Mask Reg / ID Reg 2
	 *
	 *  Note: Mask Mode is useful for rules pertaining to matching specific bits of an ID.
	 *  Note: List/ID Mode is useful for matching one or two exact ID's
	 *
	 * 														Mask Mode:
	 * 			  31 30 29 28 27 26 25 24 | 23 22 21 | 20 19 18 17 16 | 15 14 13 12 11 10 9 8 | 7 6 5 4 3 | 2 | 1 | 0
	 * 	  ID Reg  x  x  x  x  x  x  x  x	x  x  x    x  x  x  x  x    x  x  x  x  x  x  x x   x x x x x   x   x   x
	 * 	Mask Reg  x  x  x  x  x  x  x  x    x  x  x    x  x  x  x  x    x  x  x  x  x  x  x x   x x x x x   x   x   x
	 * 			  |------STID[10:3]-------|-STID[2:0]|---EXID[17:13]--|-------EXID[12:5]------|-EXID[4:0]-|IDE|RTD|-0-|
	 * 			  <-------------------FilterIDHigh-------------------> <-----------------FilterIDLow------------------>
	 * 			  <-----------------FilterMaskIDHigh-----------------> <---------------FilterMaskIDLow---------------->
	 *			  <----x----> <----x---->	<-----x-----> <----x---->   <----x----> <----x--->  <--x--> <------x------>
	 *    		  <----x----> <----x---->	<-----x-----> <----x---->   <----x----> <----x--->  <--x--> <------x------>
	 *
	 *
	 *													Identifier List Mode:
	 * 			  31 30 29 28 27 26 25 24 | 23 22 21 | 20 19 18 17 16 | 15 14 13 12 11 10 9 8 | 7 6 5 4 3 | 2 | 1 | 0
	 * 	ID Reg 1  0  1  0  0  0  0  0  0	0  0  1    0  0  0  0  0    0  0  0  0  0  0  0 0   0 0 0 0 0   0   0   0
	 * 	ID Reg 2  0  1  0  0  1  0  0  0    0  0  1    0  0  0  0  0    0  0  0  0  0  0  0 0   0 0 0 0 0   0   0   0
	 * 			  |------STID[10:3]-------|-STID[2:0]|---EXID[17:13]--|-------EXID[12:5]------|-EXID[4:0]-|IDE|RTD|-0-|
	 * 			  <-------------------FilterIDHigh-------------------> <-----------------FilterIDLow------------------>
	 * 			  <-----------------FilterMaskIDHigh-----------------> <---------------FilterMaskIDLow---------------->
	 *			  <----4----> <----0---->	<-----2-----> <----0---->   <----0----> <----0--->  <--0--> <------0------>
	 *    		  <----4----> <----8---->	<-----2-----> <----0---->   <----0----> <----0--->  <--0--> <------0------>
	 *
	 * Note: Mask Mode can also be used to check:
	 * RTR = 0 (Data Frame)				IDE = 0 (11-bit STID)
	 * RTR = 1 (Remote Frame)			IDE = 1 (29-bit EXID)
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	CAN_FilterTypeDef can1_filter_init = {0};

	//TinyBMS Default Node ID: 0x01 (hard-coded)
	//ID List Mode: Allows TinyBMS Request/Response messages from bus
	can1_filter_init.FilterActivation = ENABLE;
	can1_filter_init.FilterBank = 0;
	can1_filter_init.FilterFIFOAssignment = CAN_RX_FIFO0;
	can1_filter_init.FilterIdHigh = 0x4020; 	//IDLIST "Request to TinyBMS"
	can1_filter_init.FilterIdLow = 0x0000;
	can1_filter_init.FilterMaskIdHigh = 0x4820; //IDLIST "Response from TinyBMS"
	can1_filter_init.FilterMaskIdLow = 0x0000;
	can1_filter_init.FilterMode = CAN_FILTERMODE_IDLIST; //ID List Mode
	can1_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;
	if(HAL_CAN_ConfigFilter(&hcan1, &can1_filter_init) != HAL_OK) {
		Error_Handler();
	}
}

void CAN_Begin(void) {
	//Activate Notifications (Interrupts) by setting CAN_IER bits
	if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
		Error_Handler();
	}

	//Start CAN
	if(HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}
}

void CAN1_Tx(uint8_t device, uint8_t* message, uint8_t len) {
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;

	if(device == TINYBMS) {
		TxHeader.DLC = len;				//Data Length Code (in Bytes)
		if(message[0] == CAN_TBMS_WRITE_CAN_NODEID) 	//Standard ID (Write new nodeID.. Request StdID: 0x200 + user_input)
			TxHeader.StdId = (TINYBMS_CAN_REQUEST_BASE_STDID + message[1]);
		else if(message[0] == CAN_TBMS_READ_CAN_NODEID) //Standard ID (Read current nodeID.. Request StdID: 0x200)
			TxHeader.StdId = TINYBMS_CAN_REQUEST_BASE_STDID;
		else 											//Standard ID (Otherwise.. Request StdID: 0x201-0x23F)
			TxHeader.StdId = TinybmsStdID_Request;
		TxHeader.IDE = CAN_ID_STD; 		//Standard or Extended ID type
		TxHeader.RTR = CAN_RTR_DATA;	//Remote Transmission Request
		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, message, &TxMailbox) != HAL_OK) {
			Error_Handler();
		}
	} else if(device == ELCONCHARGER2) {
		//Todo:
		TxHeader.DLC = 8;				//Data Length Code (in Bytes)
		TxHeader.ExtId = 0x1806E5F4;	//Extended ID
		TxHeader.IDE = CAN_ID_EXT; 		//Standard or Extended ID type
		TxHeader.RTR = CAN_RTR_DATA;	//Remote Transmission Request
		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, message, &TxMailbox) != HAL_OK) {
			Error_Handler();
		}
	} else {
		Error_Handler();
	}
}

void LED_Manage_Output(uint8_t led_no) {

	switch(led_no) {
	case 1:
		HAL_GPIO_WritePin(LED_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(LED_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(LED_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		break;
	case 4:
		HAL_GPIO_WritePin(LED_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
		break;
	default:
		break;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART2) {
		printf("HAL_UART_TxCpltCallback USART2\r\n");
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART2) {
		printf("HAL_UART_RxCpltCallback USART2\r\n");
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART2) {
		printf("HAL_UART_ErrorCallback USART2\r\n");
	}
}

void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART2) {
		printf("HAL_UART_AbortReceiveCpltCallback USART2\r\n");
	}
}

void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART2) {
		printf("HAL_UART_AbortTransmitCpltCallback USART2\r\n");
	}
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART2) {
		printf("HAL_UART_AbortCpltCallback USART2\r\n");
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
	if(hcan->Instance == CAN1) {
		char msg[50];
		printf("HAL_CAN_TxMailbox0CompleteCallback CAN1\r\n");
		sprintf(msg,"Message Transmitted:M0\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	}
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
	if(hcan->Instance == CAN1) {
		char msg[50];
		printf("HAL_CAN_TxMailbox1CompleteCallback CAN1\r\n");
		sprintf(msg,"Message Transmitted:M1\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	}
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
	if(hcan->Instance == CAN1) {
		char msg[50];
		printf("HAL_CAN_TxMailbox2CompleteCallback CAN1\r\n");
		sprintf(msg,"Message Transmitted:M2\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if(hcan->Instance == CAN1) {
		//Deactivate Notifications before getting Rx Message
		if(HAL_CAN_DeactivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		printf("HAL_CAN_RxFifo0MsgPendingCallback CAN1\r\n");
	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	if(hcan->Instance == CAN1) {
		char msg[50];
		printf("HAL_CAN_ErrorCallback CAN1\r\n");
		sprintf(msg, "CAN Error Detected\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if(htim->Instance == TIM6) {
		//Every 1 second during Charging, send message to ElCon charger
		ElCon_SendMsg();
	}
}

void Error_Handler(void) {
	while(1);
}

