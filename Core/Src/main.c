/***********************************************
* @file main.c
* @brief Spartan Hyperloop TinyBMS Testing
* @author Oliver Moore
* @version 1.6
* @date 09-13-2022
***********************************************/
#include "main.h"

CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
TIM_HandleTypeDef htim6;
CAN_RxHeaderTypeDef RxHeader;

/******* Globals *******/
uint8_t rx_buffer[500];

/****************** TinyBMS CAN Identifiers ******************/
//Initialize TinyBMS Request/Response StdId's to Default
uint32_t TinybmsStdID_Request = TINYBMS_CAN_REQUEST_DEFAULT_STDID;
uint32_t TinybmsStdID_Response = TINYBMS_CAN_RESPONSE_DEFAULT_STDID;

int main(void) {
	uint8_t app_done = 0;

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

    printf("*** Spartan Hyperloop 2018-22 ***\r\n");
	printf("Energus TinyBMS API Test App\r\n");
	printf("Secondary Battery Pack BMS\r\n");

    printf("Menu\r\n");
    printf("1: UART API Test\r\n");
    printf("2: CAN API Test\r\n");
    printf("3: Monitor Charging (UART)\r\n");
    printf("4: Monitor Discharging (UART)\r\n");
    printf("5: Monitor Charging (CAN)\r\n");
    printf("6: Monitor Discharging (CAN)\r\n");
    printf("7: Quit\r\n");

	//Application Menu
	while(!app_done) {
		int userInput = 0;
		scanf("%d", &userInput);

		switch(userInput) {
		//UART API Test
		case MENU_UART_API_TEST:
			printf("UART API Test..\r\n");
			UART_Test_API();
			app_done = 1;
			break;
		//CAN API Test
		case MENU_CAN_API_TEST:
			printf("CAN API Test..\r\n");
			CAN_Test_API();
			app_done = 1;
			break;
		//Monitor Charging (UART)
		case MENU_MONITOR_CHARGE_UART:
			printf("Monitor Charging (UART)..\r\n");
			TinyBMS_MonitorCharging_UART();
			app_done = 1;
			break;
		//Monitor Discharging (UART)
		case MENU_MONITOR_DISCHARGE_UART:
			printf("Monitor Discharging (UART)..\r\n");
			TinyBMS_MonitorDischarging_UART();
			app_done = 1;
			break;
		//Monitor Charging (CAN)
		case MENU_MONITOR_CHARGE_CAN:
			printf("Monitor Charging (CAN)..\r\n");
			TinyBMS_MonitorDischarging_CAN();
			app_done = 1;
			break;
		//Monitor Discharging (CAN)
		case MENU_MONITOR_DISCHARGE_CAN:
			printf("Monitor Discharging (CAN)..\r\n");
			TinyBMS_MonitorDischarging_CAN();
			app_done = 1;
			break;
		//Quit
		case MENU_QUIT:
			printf("Exiting..\r\n");
			app_done = 1;
			break;
		//Invalid
		default:
			printf("Invalid input.\r\n");
			app_done = 0;
			break;
		}
	}
	return 0;
}

void UART_Test_API(void) {
	int8_t option = 0;
	//rl: registers to read/write | pl: payload length in bytes
	uint8_t rl = 0, pl = 0;
	uint16_t addr = 0x00;

	/*** Hangs in while loop unless success ***/
	//1.1.1 ACK
	while(TinyBMS_UART_ACK(&huart2) != CMD_SUCCESS) {}

	//1.1.2 ReadRegBlock
	rl = 16;
	addr = CELL1_VOLTAGE;
	while(TinyBMS_UART_ReadRegBlock(&huart2, rl, addr) != CMD_SUCCESS) {}

	//1.1.3 ReadRegIndividual
	pl = 10;
	uint16_t addrs1[5] = { BMS_LIFETIME_COUNTER, ESTIMATED_TIME_LEFT,
						   BATTERY_PACK_VOLTAGE, BATTERY_PACK_CURRENT,
						   BMS_ONLINE_STATUS };
	while(TinyBMS_UART_ReadRegIndividual(&huart2, pl, addrs1) != CMD_SUCCESS) {}

	//1.1.4 WriteRegBlock
	pl = 8;
	addr = OVERVOLTAGE_CUTOFF;
	uint16_t data1[4] = {4200, 3000, 50, 25};
	/* Writing to 4 registers (2bytes each) starting with: OVERVOLTAGE_CUTOFF,
	 * UNDERVOLTAGE_CUTOFF, DISCHARGE_OVERCURRENT_CUTOFF, CHARGE_OVERCURRENT_CUTOFF */
	while(TinyBMS_UART_WriteRegBlock(&huart2, pl, addr, data1) != CMD_SUCCESS) {}

	//1.1.5 WriteRegIndividual
	pl = 8;
	uint16_t addrs2[4] = { FULLYCHARGED_VOLTAGE, FULLYDISCHARGED_VOLTAGE,
							  BATTERY_CAPACITY, NUMBER_OF_SERIES_CELLS };
	uint16_t data2[4] = {4200, 3000, 5000, 7};
	while(TinyBMS_UART_WriteRegIndividual(&huart2, pl, addrs2, data2) != CMD_SUCCESS) {}

	//1.1.6 ReadRegBlockMODBUS
	addr = CELL1_VOLTAGE;
	rl = 16;
	while(TinyBMS_UART_ReadRegBlockMODBUS(&huart2, addr, rl) != CMD_SUCCESS) {}

	//1.1.7 WriteRegBlockMODBUS
	addr = OVERVOLTAGE_CUTOFF;
	rl = 4;
	pl = 8;
	uint16_t data3[4] = {4200, 3000, 50, 25};
	/* Writing to 4 registers (2bytes each) starting with: OVERVOLTAGE_CUTOFF,
	 * UNDERVOLTAGE_CUTOFF, DISCHARGE_OVERCURRENT_CUTOFF, CHARGE_OVERCURRENT_CUTOFF */
	while(TinyBMS_UART_WriteRegBlockMODBUS(&huart2, addr, rl, pl, data3) != CMD_SUCCESS) {}

	//1.1.8 ResetClearEventsStatistics
	option = TINYBMS_CLEAR_EVENTS;
	while(TinyBMS_UART_ResetClearEventsStatistics(&huart2, option) != CMD_SUCCESS) {}
	option = TINYBMS_CLEAR_STATS;
	while(TinyBMS_UART_ResetClearEventsStatistics(&huart2, option) != CMD_SUCCESS) {}
	option = TINYBMS_RESET_BMS;
	while(TinyBMS_UART_ResetClearEventsStatistics(&huart2, option) != CMD_SUCCESS) {}

	//1.1.9 ReadNewestEvents
	while(TinyBMS_UART_ReadNewestEvents(&huart2) != CMD_SUCCESS) {}

	//1.1.10 ReadAllEvents
	while(TinyBMS_UART_ReadAllEvents(&huart2) != CMD_SUCCESS) {}

	//1.1.11 ReadBatteryPackVoltage
	float packVoltage = TinyBMS_UART_ReadBatteryPackVoltage(&huart2);
	printf("Secondary Battery Pack Voltage: %f (V)\r\n", packVoltage);

	//1.1.12 ReadBatteryPackCurrent
	float packCurrent = TinyBMS_UART_ReadBatteryPackCurrent(&huart2);
	printf("Secondary Battery Pack Current: %f (A)\r\n", packCurrent);

	//1.1.13 ReadBatteryPackMaxCellVoltage
	uint16_t maxCellVoltage = TinyBMS_UART_ReadBatteryPackMaxCellVoltage(&huart2);
	printf("Secondary Battery Pack Maximum Cell Voltage: %u (mV)\r\n", maxCellVoltage);

	//1.1.14 ReadBatteryPackMinCellVoltage
	uint16_t minCellVoltage = TinyBMS_UART_ReadBatteryPackMinCellVoltage(&huart2);
	printf("Secondary Battery Pack Minimum Cell Voltage: %u (mV)\r\n", minCellVoltage);

	//1.1.15 ReadOnlineStatus
	uint16_t onlineStatus = TinyBMS_UART_ReadOnlineStatus(&huart2);
	printf("TinyBMS Online Status: %u\r\n", onlineStatus);

	//1.1.16 ReadLifetimeCounter
	uint32_t lifetimeCounter = TinyBMS_UART_ReadLifetimeCounter(&huart2);
	printf("TinyBMS Lifetime Counter: %lu (s)\r\n", lifetimeCounter);

	//1.1.17 ReadEstimatedSOCValue
	uint32_t estSOC = TinyBMS_UART_ReadEstimatedSOCValue(&huart2);
	printf("Estimated State of Charge (SOC): %lu (0.000 001 %% resolution)\r\n", estSOC);

	//1.1.18 ReadDeviceTemperatures
	while(TinyBMS_UART_ReadDeviceTemperatures(&huart2) != CMD_SUCCESS) {}

	//1.1.19 ReadBatteryPackCellVoltages
	while(TinyBMS_UART_ReadBatteryPackCellVoltages(&huart2) != CMD_SUCCESS) {}

	//1.1.20 ReadSettingsValues
	rl = 100; //Registers to read: Max. 100 (0x64) registers
	option = TINYBMS_SETTINGS_MIN;
	while(TinyBMS_UART_ReadSettingsValues(&huart2, option, rl) != CMD_SUCCESS) {}
	option = TINYBMS_SETTINGS_MAX;
	while(TinyBMS_UART_ReadSettingsValues(&huart2, option, rl) != CMD_SUCCESS) {}
	option = TINYBMS_SETTINGS_DEFAULT;
	while(TinyBMS_UART_ReadSettingsValues(&huart2, option, rl) != CMD_SUCCESS) {}
	option = TINYBMS_SETTINGS_CURRENT;
	while(TinyBMS_UART_ReadSettingsValues(&huart2, option, rl) != CMD_SUCCESS) {}

	//1.1.21 ReadVersion
	while(TinyBMS_UART_ReadVersion(&huart2) != CMD_SUCCESS) {}

	//1.1.22 ReadVersionExtended
	while(TinyBMS_UART_ReadVersionExtended(&huart2) != CMD_SUCCESS) {}

	//1.1.23 ReadCalcSpeedDistanceLeftEstTimeLeft
	while(TinyBMS_UART_ReadCalcSpeedDistanceLeftEstTimeLeft(&huart2) != CMD_SUCCESS) {}
}

void CAN_Test_API(void) {
	int8_t option = 0;
	uint8_t rl = 0;
	uint16_t addr = 0x00;

	/*** Hangs in while loop unless success ***/
	//2.1.1 ResetClearEventsStatistics
	option = TINYBMS_CLEAR_EVENTS;
	while(TinyBMS_CAN_ResetClearEventsStatistics(&hcan1, option) != CMD_SUCCESS) {}
	option = TINYBMS_CLEAR_STATS;
	while(TinyBMS_CAN_ResetClearEventsStatistics(&hcan1, option) != CMD_SUCCESS) {}
	option = TINYBMS_RESET_BMS;
	while(TinyBMS_CAN_ResetClearEventsStatistics(&hcan1, option) != CMD_SUCCESS) {}

	//2.1.2 ReadRegBlock
	rl = 16;
	addr = CELL1_VOLTAGE;
	while(TinyBMS_CAN_ReadRegBlock(&hcan1, rl, addr) != CMD_SUCCESS) {}

	//2.1.3 WriteRegBlock
	rl = 4;
	addr = OVERVOLTAGE_CUTOFF;
	uint16_t data1[4] = {4200, 3000, 50, 25};
	/* Writing to 4 registers (2bytes each) starting with: OVERVOLTAGE_CUTOFF,
	* UNDERVOLTAGE_CUTOFF, DISCHARGE_OVERCURRENT_CUTOFF, CHARGE_OVERCURRENT_CUTOFF */
	while(TinyBMS_CAN_WriteRegBlock(&hcan1, rl, addr, data1) != CMD_SUCCESS) {}

	//2.1.4 ReadNewestEvents
	while(TinyBMS_CAN_ReadNewestEvents(&hcan1) != CMD_SUCCESS) {}

	//2.1.5 ReadAllEvents
	while(TinyBMS_CAN_ReadAllEvents(&hcan1) != CMD_SUCCESS) {}

	//2.1.6 ReadBatteryPackVoltage
	float packVoltage = TinyBMS_CAN_ReadBatteryPackVoltage(&hcan1);
	printf("Secondary Battery Pack Voltage: %f (V)\r\n", packVoltage);

	//2.1.7 ReadBatteryPackCurrent
	float packCurrent = TinyBMS_CAN_ReadBatteryPackCurrent(&hcan1);
	printf("Secondary Battery Pack Current: %f (A)\r\n", packCurrent);

	//2.1.8 ReadBatteryPackMaxCellVoltage
	uint16_t maxCellVoltage = TinyBMS_CAN_ReadBatteryPackMaxCellVoltage(&hcan1);
	printf("Secondary Battery Pack Maximum Cell Voltage: %u (mV)\r\n", maxCellVoltage);

	//2.1.9 ReadBatteryPackMinCellVoltage
	uint16_t minCellVoltage = TinyBMS_CAN_ReadBatteryPackMinCellVoltage(&hcan1);
	printf("Secondary Battery Pack Minimum Cell Voltage: %u (mV)\r\n", minCellVoltage);

	//2.1.10 ReadOnlineStatus
	uint16_t onlineStatus = TinyBMS_CAN_ReadOnlineStatus(&hcan1);
	printf("TinyBMS Online Status: %u\r\n", onlineStatus);

	//2.1.11 ReadLifetimeCounter
	uint32_t lifetimeCounter = TinyBMS_CAN_ReadLifetimeCounter(&hcan1);
	printf("TinyBMS Lifetime Counter: %lu (s)\r\n", lifetimeCounter);

	//2.1.12 ReadEstimatedSOCValue
	uint32_t estSOC = TinyBMS_CAN_ReadEstimatedSOCValue(&hcan1);
	printf("TinyBMS Estimated StateOfCharge: %lu (0.000 001 %% Resolution)\r\n", estSOC);

	//2.1.13 ReadDeviceTemperatures
	while(TinyBMS_CAN_ReadDeviceTemperatures(&hcan1) != CMD_SUCCESS) {}

	//2.1.14 ReadBatteryPackCellVoltages
	while(TinyBMS_CAN_ReadBatteryPackCellVoltages(&hcan1) != CMD_SUCCESS) {}

	//2.1.15 ReadSettingsValues
	rl = 100; //Registers to read: Max. 100 (0x64) registers
	option = TINYBMS_SETTINGS_MIN;
	while(TinyBMS_CAN_ReadSettingsValues(&hcan1, option, rl) != CMD_SUCCESS) {}
	option = TINYBMS_SETTINGS_MAX;
	while(TinyBMS_CAN_ReadSettingsValues(&hcan1, option, rl) != CMD_SUCCESS) {}
	option = TINYBMS_SETTINGS_DEFAULT;
	while(TinyBMS_CAN_ReadSettingsValues(&hcan1, option, rl) != CMD_SUCCESS) {}
	option = TINYBMS_SETTINGS_CURRENT;
	while(TinyBMS_CAN_ReadSettingsValues(&hcan1, option, rl) != CMD_SUCCESS) {}

	//2.1.16 ReadVersion
	while(TinyBMS_CAN_ReadVersion(&hcan1) != CMD_SUCCESS) {}

	//2.1.17 ReadCalcSpeedDistanceLeftEstTimeLeft
	while(TinyBMS_CAN_ReadCalcSpeedDistanceLeftEstTimeLeft(&hcan1) != CMD_SUCCESS) {}

	//2.1.18 ReadNodeID
	while(TinyBMS_CAN_ReadNodeID(&hcan1) != CMD_SUCCESS) {}

	//2.1.19 WriteNodeID
	uint8_t nodeID = 0x01;
	while(TinyBMS_CAN_WriteNodeID(&hcan1, nodeID) != CMD_SUCCESS) {}
}

void TinyBMS_MonitorCharging_UART(void) {
	//TinyBMS Init
	if(TinyBMS_Init_UART() != CMD_SUCCESS) {
		printf("TinyBMS Init failed.\r\n");
		Error_Handler();
	}

	//Start the Timer (Interrupt Mode / Non-Blocking)
	//Timer is used to send a message to the charger every 1 second
	//Todo:
	HAL_TIM_Base_Start_IT(&htim6);

	uint16_t cellv[7] = {};
	uint16_t minCellVoltage = TinyBMS_UART_ReadBatteryPackMinCellVoltage(&huart2);
	uint16_t maxCellVoltage = TinyBMS_UART_ReadBatteryPackMaxCellVoltage(&huart2);
	uint16_t numDetectedCells = TinyBMS_UART_ReadRegBlock(&huart2, 1, NUMBER_OF_DETECTED_CELLS);

	while(TinyBMS_UART_ReadOnlineStatus(&huart2) == TINYBMS_STATUS_CHARGING) {
		//Verify that all cells are being detected
		if(numDetectedCells != NUMCELLS_SECONDARY) {
			printf("Some cells are not being detected!\r\n");
		}

		//Get voltage of all cells and compare with max/min voltage thresholds
		TinyBMS_UART_ReadBatteryPackCellVoltages(&huart2);
		for(uint8_t i = 0; i < NUMCELLS_SECONDARY; i++) {
			cellv[i] = TinyBMS_UART_ReadRegBlock(&huart2, 1, CELL1_VOLTAGE+i);
			if(cellv[i] < minCellVoltage)  {
				printf("Cell %u is below the minimum voltage threshold!\r\n", i+1);
			}
			if(cellv[i] > maxCellVoltage) {
				printf("Cell %u is above the maximum voltage threshold!\r\n", i+1);
			}
		}

		//Check if cells need balancing or are in progress of balancing
		//Regs 51 & 52: BALANCING_DECISION_BITS & REAL_BALANCING_BITS
		TinyBMS_UART_ReadRegBlock(&huart2, 2, BALANCING_DECISION_BITS);

		//Check Newest Events
		TinyBMS_UART_ReadNewestEvents(&huart2);

		//Check Temperatures
		TinyBMS_UART_ReadDeviceTemperatures(&huart2);

		//Get State of Charge
		uint32_t estSOC = TinyBMS_UART_ReadEstimatedSOCValue(&huart2);
		printf("TinyBMS Estimated StateOfCharge: %lu (0.000 001 %% Resolution)\r\n", estSOC);

		//Get Pack Voltage
		float packVoltage = TinyBMS_UART_ReadBatteryPackVoltage(&huart2);
		printf("Secondary Battery Pack Voltage: %f (V)\r\n", packVoltage);

		//Get Pack Current
		float packCurrent = TinyBMS_UART_ReadBatteryPackCurrent(&huart2);
		printf("Secondary Battery Pack Current: %f (A)\r\n", packCurrent);
	}

	//Stop the Timer
	HAL_TIM_Base_Stop_IT(&htim6);
}

void TinyBMS_MonitorDischarging_UART(void) {
	//TinyBMS Init
	if(TinyBMS_Init_UART() != CMD_SUCCESS) {
		printf("TinyBMS Init failed.\r\n");
		Error_Handler();
	}

	//Todo:
	//Similar to MonitorCharging_UART
}

void TinyBMS_MonitorCharging_CAN(void) {
	//TinyBMS Init
	if(TinyBMS_Init_CAN() != CMD_SUCCESS) {
		printf("TinyBMS Init failed.\r\n");
		Error_Handler();
	}

	//Start the Timer (Interrupt Mode / Non-Blocking)
	//Timer is used to send a message to the charger every 1 second
	HAL_TIM_Base_Start_IT(&htim6);

	uint16_t cellv[7] = {};
	uint16_t minCellVoltage = TinyBMS_CAN_ReadBatteryPackMinCellVoltage(&hcan1);
	uint16_t maxCellVoltage = TinyBMS_CAN_ReadBatteryPackMaxCellVoltage(&hcan1);
	uint16_t numDetectedCells = TinyBMS_CAN_ReadRegBlock(&hcan1, 1, NUMBER_OF_DETECTED_CELLS);

	while(TinyBMS_CAN_ReadOnlineStatus(&hcan1) == TINYBMS_STATUS_CHARGING) {
		//Verify that all cells are being detected
		if(numDetectedCells != NUMCELLS_SECONDARY) {
			printf("Some cells are not being detected!\r\n");
		}

		//Get voltage of all cells and compare with max/min voltage thresholds
		TinyBMS_CAN_ReadBatteryPackCellVoltages(&hcan1);
		for(uint8_t i = 0; i < NUMCELLS_SECONDARY; i++) {
			cellv[i] = TinyBMS_CAN_ReadRegBlock(&hcan1, 1, CELL1_VOLTAGE+i);
			if(cellv[i] < minCellVoltage)  {
				printf("Cell %u is below the minimum voltage threshold!\r\n", i+1);
			}
			if(cellv[i] > maxCellVoltage) {
				printf("Cell %u is above the maximum voltage threshold!\r\n", i+1);
			}
		}

		//Check if cells need balancing or are in progress of balancing
		//Regs 51 & 52: BALANCING_DECISION_BITS & REAL_BALANCING_BITS
		TinyBMS_CAN_ReadRegBlock(&hcan1, 2, BALANCING_DECISION_BITS);

		//Check Newest Events
		TinyBMS_CAN_ReadNewestEvents(&hcan1);

		//Check Temperatures
		TinyBMS_CAN_ReadDeviceTemperatures(&hcan1);

		//Get State of Charge
		uint32_t estSOC = TinyBMS_CAN_ReadEstimatedSOCValue(&hcan1);
		printf("TinyBMS Estimated StateOfCharge: %lu (0.000 001 %% Resolution)\r\n", estSOC);

		//Get Pack Voltage
		float packVoltage = TinyBMS_CAN_ReadBatteryPackVoltage(&hcan1);
		printf("Secondary Battery Pack Voltage: %f (V)\r\n", packVoltage);

		//Get Pack Current
		float packCurrent = TinyBMS_CAN_ReadBatteryPackCurrent(&hcan1);
		printf("Secondary Battery Pack Current: %f (A)\r\n", packCurrent);
	}

	//Stop the Timer
	HAL_TIM_Base_Stop_IT(&htim6);
}

void TinyBMS_MonitorDischarging_CAN(void) {
	//TinyBMS Init
	if(TinyBMS_Init_CAN() != CMD_SUCCESS) {
		printf("TinyBMS Init failed.\r\n");
		Error_Handler();
	}

	//Todo:
	//Similar to MonitorCharging_CAN
}

void ElCon_SendMsg(void) {
	//Triggered from HAL_TIM_PeriodElapsedCallback()
	//Every 1 second, send 8-bytes of data with voltage and current requested to ExtID 0x1806E5F4
	//Todo:
	uint8_t msg[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t len = 8;
	CAN1_Tx(ELCONCHARGER2, msg, len);
}

uint8_t TinyBMS_Init_UART(void) {
	uint8_t retval = CMD_FAILURE;
	int validInput = FALSE;
	int userInput = 0;

	//Reset BMS?
	while(!validInput) {
		printf("Reset BMS?\r\n");
		printf("0: No   1: Yes\r\n");
		userInput = 0;
		scanf("%d", &userInput);

		switch(userInput) {
		case NO:
			validInput = TRUE;
			printf("Skipping reset..\r\n");
			break;
		case YES:
			validInput = TRUE;
			printf("Resetting TinyBMS..\r\n");
			TinyBMS_UART_ResetClearEventsStatistics(&huart2, TINYBMS_RESET_BMS);
			break;
		default:
			validInput = FALSE;
			printf("Invalid input.\r\n");
			break;
		}
	}

	//Check if BMS was Reset by reading Lifetime Counter
	if(TinyBMS_UART_ReadLifetimeCounter(&huart2) < 5) {
		printf("TinyBMS successfully Reset!\r\n");
	}

	//Clear Events & Statistics?
	while(!validInput) {
		printf("Clear Events and Statistics?\r\n)");
		printf("0: Neither          1: Only Events\r\n");
		printf("2: Only Statistics  3: Both\r\n");
		userInput = 0;
		scanf("%d", &userInput);

		switch(userInput) {
		case NEITHER:
			validInput = TRUE;
			printf("Keeping previous Events and Statistics..\r\n");
			break;
		case ONLYEVENTS:
			validInput = TRUE;
			printf("Clearing TinyBMS Events only..\r\n");
			TinyBMS_UART_ResetClearEventsStatistics(&huart2, TINYBMS_CLEAR_EVENTS);
			break;
		case ONLYSTATS:
			validInput = TRUE;
			printf("Clearing TinyBMS Statistics only..\r\n");
			TinyBMS_UART_ResetClearEventsStatistics(&huart2, TINYBMS_CLEAR_STATS);
			break;
		case BOTH:
			validInput = TRUE;
			printf("Clearing both TinyBMS Events and Statistics..\r\n");
			TinyBMS_UART_ResetClearEventsStatistics(&huart2, TINYBMS_CLEAR_EVENTS);
			TinyBMS_UART_ResetClearEventsStatistics(&huart2, TINYBMS_CLEAR_STATS);
			break;
		default:
			validInput = FALSE;
			printf("Invalid input.\r\n");
			break;
		}
	}

	//Read Version
	TinyBMS_UART_ReadVersion(&huart2);
	TinyBMS_UART_ReadVersionExtended(&huart2);

	//Get Min/Max Cell Voltage Thresholds
	uint16_t minCellVoltage = TinyBMS_UART_ReadBatteryPackMinCellVoltage(&huart2);
	printf("Battery Pack Minimum Cell Voltage: %u (mV)\r\n", minCellVoltage);
	uint16_t maxCellVoltage = TinyBMS_UART_ReadBatteryPackMaxCellVoltage(&huart2);
	printf("Battery Pack Maximum Cell Voltage: %u (mV)\r\n", maxCellVoltage);

	//Check for any active events
	TinyBMS_UART_ReadAllEvents(&huart2);

	//Verify Pack Voltage and Current
	float packVoltage = TinyBMS_UART_ReadBatteryPackVoltage(&huart2);
	printf("Secondary Battery Pack Voltage: %f (V)\r\n", packVoltage);
	float packCurrent = TinyBMS_UART_ReadBatteryPackCurrent(&huart2);
	printf("Secondary Battery Pack Current: %f (A)\r\n", packCurrent);

	//Get initial State of Charge
	uint32_t estSOC = TinyBMS_UART_ReadEstimatedSOCValue(&huart2);
	printf("TinyBMS Estimated StateOfCharge: %lu (0.000 001 %% Resolution)\r\n", estSOC);

	//Check Temperatures
	TinyBMS_UART_ReadDeviceTemperatures(&huart2);

	//Settings Registers: 300-301, 303-304, 306-308, 312-320, 328, 330-343
	//					  (30 total settings) (344-399 reserved)
	// rl max is 100 (0x64) registers, but this exceeds the actual total
	// Check manual for explanations and screenshots of Battery Insider settings
	uint16_t addr[28] = {
			FULLYCHARGED_VOLTAGE, 			//0
			FULLYDISCHARGED_VOLTAGE, 		//1
			EARLY_BALANCING_THRESHOLD, 		//2
			CHARGE_FINISHED_CURRENT, 		//3
			BATTERY_CAPACITY, 				//4
			NUMBER_OF_SERIES_CELLS, 		//5
			ALLOWED_DISBALANCE, 			//6
			PULSES_PER_UNIT,				//7
			DISTANCE_UNIT_NAME, 			//8
			OVERVOLTAGE_CUTOFF, 			//9
			UNDERVOLTAGE_CUTOFF, 			//10
			DISCHARGE_OVERCURRENT_CUTOFF, 	//11
			CHARGE_OVERCURRENT_CUTOFF,		//12
			OVERTEMP_CUTOFF,				//13
			LOWTEMP_CHARGER_CUTOFF,			//14
			//not setting STATE_OF_CHARGE_SETMANUAL
			CHARGER_TYPE,					//15
			LOAD_SWITCH_TYPE,				//16
			AUTOMATIC_RECOVERY,				//17
			CHARGER_SWITCH_TYPE,			//18
			IGNITION,						//19
			CHARGER_DETECTION,				//20
			SPEED_SENSOR_INPUT,				//21
			PRECHARGE_PIN,					//22
			PRECHARGE_DURATION,				//23
			TEMPERATURE_SENSOR_TYPE,		//24
			BMS_OPERATION_MODE,				//25
			//not setting SINGLE_PORT_SWITCH_TYPE
			BROADCAST_TIME,					//26
			PROTOCOL						//2
	};
	uint16_t data[28] = {
			4000, 							//0
			3000, 							//1
			3200, 							//2
			1000, 							//3
			5000, 							//4
			7, 								//5
			15, 							//6
			1,								//7
			UNIT_KILOMETER, 				//8
			4200, 							//9
			2900, 							//10
			60, 							//11
			30,								//12
			60,								//13
			1,								//14
			CHARGER_TYPE_CCCV,				//15
			LOAD_SWITCH_TYPE_FET,			//16
			5,								//17
			CHARGER_SWITCH_TYPE_CHARGEFET,	//18
			IGNITION_DISABLED,				//19
			CHARGER_DETECTION_INTERNAL,		//20
			SPEED_SENSOR_INPUT_DISABLED,	//21
			PRECHARGE_PIN_DISABLED,			//22
			PRECHARGE_DURATION_100MS,		//23
			TEMP_SENSOR_TYPE_DUAL10KNTC,	//24
			OPMODE_DUALPORT,				//25
			BROADCAST_TIME_DISABLED,		//26
			PROTOCOL_CAV3					//27
	};

	TinyBMS_UART_WriteRegIndividual(&huart2, 28, addr, data);

	//Read back the settings
	TinyBMS_UART_ReadSettingsValues(&huart2, TINYBMS_SETTINGS_CURRENT, 30);

	//Verify Online Status is TINYBMS_STATUS_IDLE before proceeding
	uint8_t isIdle = FALSE;
	while(!isIdle) {
		switch(TinyBMS_UART_ReadOnlineStatus(&huart2)) {
		case TINYBMS_STATUS_CHARGING:
			printf("TinyBMS is Charging..\r\n");
			break;
		case TINYBMS_STATUS_FULLYCHARGED:
			printf("TinyBMS is Fully Charged!\r\n");
			break;
		case TINYBMS_STATUS_DISCHARGING:
			printf("TinyBMS is Discharging..\r\n");
			break;
		case TINYBMS_STATUS_REGENERATION:
			printf("TinyBMS is Regenerating..\r\n");
			break;
		case TINYBMS_STATUS_IDLE:
			printf("TinyBMS is Idle..\r\n");
			isIdle = TRUE;
			retval = CMD_SUCCESS;
			break;
		case TINYBMS_STATUS_FAULT:
			printf("TinyBMS Fault detected..\r\n");
			//Check for any active events
			TinyBMS_UART_ReadAllEvents(&huart2);
			break;
		default:
			Error_Handler();
		}
	}

	return retval;
}

uint8_t TinyBMS_Init_CAN(void) {
	uint8_t retval = CMD_FAILURE;
	int validInput = FALSE;
	int userInput = 0;

	//Read CAN NodeID and update to it if required
	TinyBMS_CAN_ReadNodeID(&hcan1);

	//Reset BMS?
	while(!validInput) {
		printf("Reset BMS?\r\n");
		printf("0: No   1: Yes\r\n");
		userInput = 0;
		scanf("%d", &userInput);

		switch(userInput) {
		case NO:
			validInput = TRUE;
			printf("Skipping reset..\r\n");
			break;
		case YES:
			validInput = TRUE;
			printf("Resetting TinyBMS..\r\n");
			TinyBMS_CAN_ResetClearEventsStatistics(&hcan1, TINYBMS_RESET_BMS);
			break;
		default:
			validInput = FALSE;
			printf("Invalid input.\r\n");
			break;
		}
	}

	//Check if BMS was Reset by reading Lifetime Counter
	if(TinyBMS_CAN_ReadLifetimeCounter(&hcan1) < 5) {
		printf("TinyBMS successfully Reset!\r\n");
	}

	//Clear Events & Statistics?
	while(!validInput) {
		printf("Clear Events and Statistics?\r\n)");
		printf("0: Neither          1: Only Events\r\n");
		printf("2: Only Statistics  3: Both\r\n");
		userInput = 0;
		scanf("%d", &userInput);

		switch(userInput) {
		case NEITHER:
			validInput = TRUE;
			printf("Keeping previous Events and Statistics..\r\n");
			break;
		case ONLYEVENTS:
			validInput = TRUE;
			printf("Clearing TinyBMS Events only..\r\n");
			TinyBMS_CAN_ResetClearEventsStatistics(&hcan1, TINYBMS_CLEAR_EVENTS);
			break;
		case ONLYSTATS:
			validInput = TRUE;
			printf("Clearing TinyBMS Statistics only..\r\n");
			TinyBMS_CAN_ResetClearEventsStatistics(&hcan1, TINYBMS_CLEAR_STATS);
			break;
		case BOTH:
			validInput = TRUE;
			printf("Clearing both TinyBMS Events and Statistics..\r\n");
			TinyBMS_CAN_ResetClearEventsStatistics(&hcan1, TINYBMS_CLEAR_EVENTS);
			TinyBMS_CAN_ResetClearEventsStatistics(&hcan1, TINYBMS_CLEAR_STATS);
			break;
		default:
			validInput = FALSE;
			printf("Invalid input.\r\n");
			break;
		}
	}

	//Read Version
	TinyBMS_CAN_ReadVersion(&hcan1);

	//Get Min/Max Cell Voltage Thresholds
	uint16_t minCellVoltage = TinyBMS_CAN_ReadBatteryPackMinCellVoltage(&hcan1);
	printf("Battery Pack Minimum Cell Voltage: %u (mV)\r\n", minCellVoltage);
	uint16_t maxCellVoltage = TinyBMS_CAN_ReadBatteryPackMaxCellVoltage(&hcan1);
	printf("Battery Pack Maximum Cell Voltage: %u (mV)\r\n", maxCellVoltage);

	//Check for any active events
	TinyBMS_CAN_ReadAllEvents(&hcan1);

	//Verify Pack Voltage and Current
	float packVoltage = TinyBMS_CAN_ReadBatteryPackVoltage(&hcan1);
	printf("Secondary Battery Pack Voltage: %f (V)\r\n", packVoltage);
	float packCurrent = TinyBMS_CAN_ReadBatteryPackCurrent(&hcan1);
	printf("Secondary Battery Pack Current: %f (A)\r\n", packCurrent);

	//Get initial State of Charge
	uint32_t estSOC = TinyBMS_CAN_ReadEstimatedSOCValue(&hcan1);
	printf("TinyBMS Estimated StateOfCharge: %lu (0.000 001 %% Resolution)\r\n", estSOC);

	//Check Temperatures
	TinyBMS_CAN_ReadDeviceTemperatures(&hcan1);

	//Settings Registers: 300-301, 303-304, 306-308, 312-320, 328, 330-343
	//					  (30 total settings) (344-399 reserved)
	// rl max is 100 (0x64) registers, but this exceeds the actual total
	// Check manual for explanations and screenshots of Battery Insider settings
	uint16_t addr1[2] = {
			FULLYCHARGED_VOLTAGE, 			//0
			FULLYDISCHARGED_VOLTAGE 		//1
	};
	uint16_t addr2[2] = {
			EARLY_BALANCING_THRESHOLD, 		//2
			CHARGE_FINISHED_CURRENT			//3
	};
	uint16_t addr3[3] = {
			BATTERY_CAPACITY, 				//4
			NUMBER_OF_SERIES_CELLS, 		//5
			ALLOWED_DISBALANCE 				//6
	};
	uint16_t addr4[1] = {
			PULSES_PER_UNIT					//7
	};
	uint16_t addr5[7] = {
			DISTANCE_UNIT_NAME, 			//8
			OVERVOLTAGE_CUTOFF, 			//9
			UNDERVOLTAGE_CUTOFF, 			//10
			DISCHARGE_OVERCURRENT_CUTOFF, 	//11
			CHARGE_OVERCURRENT_CUTOFF,		//12
			OVERTEMP_CUTOFF,				//13
			LOWTEMP_CHARGER_CUTOFF			//14
	};
	//not setting STATE_OF_CHARGE_SETMANUAL
	uint16_t addr6[14] = {
			CHARGER_TYPE,					//15
			LOAD_SWITCH_TYPE,				//16
			AUTOMATIC_RECOVERY,				//17
			CHARGER_SWITCH_TYPE,			//18
			IGNITION,						//19
			CHARGER_DETECTION,				//20
			SPEED_SENSOR_INPUT,				//21
			PRECHARGE_PIN,					//22
			PRECHARGE_DURATION,				//23
			TEMPERATURE_SENSOR_TYPE,		//24
			//not setting SINGLE_PORT_SWITCH_TYPE
			BMS_OPERATION_MODE,				//25
			BROADCAST_TIME,					//26
			PROTOCOL						//27
	};

	uint16_t data1[2] = {
			4000, 							//0
			3000 							//1
	};
	uint16_t data2[2] = {
			3200, 							//2
			1000 							//3
	};
	uint16_t data3[3] = {
			5000, 							//4
			7, 								//5
			15 								//6
	};
	uint16_t data4[1] = {
			1								//7
	};
	uint16_t data5[7] = {
			UNIT_KILOMETER, 				//8
			4200, 							//9
			2900, 							//10
			60, 							//11
			30,								//12
			60,								//13
			1								//14
	};
	uint16_t data6[14] = {
			CHARGER_TYPE_CAN,				//15
			LOAD_SWITCH_TYPE_FET,			//16
			5,								//17
			CHARGER_SWITCH_TYPE_CHARGEFET,	//18
			IGNITION_DISABLED,				//19
			CHARGER_DETECTION_INTERNAL,		//20
			SPEED_SENSOR_INPUT_DISABLED,	//21
			PRECHARGE_PIN_DISABLED,			//22
			PRECHARGE_DURATION_100MS,		//23
			TEMP_SENSOR_TYPE_DUAL10KNTC,	//24
			OPMODE_DUALPORT,				//25
			BROADCAST_TIME_DISABLED,		//26
			PROTOCOL_CAV3					//27
	};
	TinyBMS_CAN_WriteRegBlock(&hcan1, 2, addr1[0], data1);
	TinyBMS_CAN_WriteRegBlock(&hcan1, 2, addr2[0], data2);
	TinyBMS_CAN_WriteRegBlock(&hcan1, 3, addr3[0], data3);
	TinyBMS_CAN_WriteRegBlock(&hcan1, 1, addr4[0], data4);
	TinyBMS_CAN_WriteRegBlock(&hcan1, 7, addr5[0], data5);
	TinyBMS_CAN_WriteRegBlock(&hcan1, 14, addr6[0], data6);

	//Read back the settings
	TinyBMS_CAN_ReadSettingsValues(&hcan1, TINYBMS_SETTINGS_CURRENT, 30);

	//Verify Online Status is TINYBMS_STATUS_IDLE before proceeding
	uint8_t isIdle = FALSE;
	while(!isIdle) {
		switch(TinyBMS_CAN_ReadOnlineStatus(&hcan1)) {
		case TINYBMS_STATUS_CHARGING:
			printf("TinyBMS is Charging..\r\n");
			break;
		case TINYBMS_STATUS_FULLYCHARGED:
			printf("TinyBMS is Fully Charged!\r\n");
			break;
		case TINYBMS_STATUS_DISCHARGING:
			printf("TinyBMS is Discharging..\r\n");
			break;
		case TINYBMS_STATUS_REGENERATION:
			printf("TinyBMS is Regenerating..\r\n");
			break;
		case TINYBMS_STATUS_IDLE:
			printf("TinyBMS is Idle..\r\n");
			isIdle = TRUE;
			retval = CMD_SUCCESS;
			break;
		case TINYBMS_STATUS_FAULT:
			printf("TinyBMS Fault detected..\r\n");
			//Check for any active events
			TinyBMS_CAN_ReadAllEvents(&hcan1);
			break;
		default:
			Error_Handler();
		}
	}

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
	case SYS_CLOCK_FREQ_50MHZ:
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
	case SYS_CLOCK_FREQ_84MHZ:
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
	case SYS_CLOCK_FREQ_120MHZ:
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
		HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	}
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
	if(hcan->Instance == CAN1) {
		char msg[50];
		printf("HAL_CAN_TxMailbox1CompleteCallback CAN1\r\n");
		sprintf(msg,"Message Transmitted:M1\r\n");
		HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	}
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
	if(hcan->Instance == CAN1) {
		char msg[50];
		printf("HAL_CAN_TxMailbox2CompleteCallback CAN1\r\n");
		sprintf(msg,"Message Transmitted:M2\r\n");
		HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
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
		HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM6) {
		//Every 1 second during Charging, send message to ElCon charger
		ElCon_SendMsg();
	}
}

void Error_Handler(void) {
	printf("Inside Error Handler\r\n");
	while(1);
}

