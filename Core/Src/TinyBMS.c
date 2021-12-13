/***********************************************
* @file TinyBMS.c
* @brief TinyBMS Library
* @author Oliver Moore
* @version 1.0
* @date 11-21-2021
***********************************************/

/*
 * UART API Testing :
 *
 *   #    	Testing   Complete    	Function
 *  1.1.1 		[x] 		[]		TinyBMS_ACK
 *  1.1.2 		[] 			[]		TinyBMS_ReadRegBlock
 *  1.1.3 		[] 			[]		TinyBMS_ReadRegIndividual
 *  1.1.4 		[] 			[]		TinyBMS_WriteRegBlock
 *  1.1.5 		[] 			[]		TinyBMS_WriteRegIndividual
 *  1.1.6 		[] 			[]		TinyBMS_ReadRegBlockMODBUS
 *  1.1.7 		[] 			[]		TinyBMS_WriteRegBlockMODBUS
 *
 *  1.1.8 		[x] 		[] 		TinyBMS_ResetClearEventsStatistics
 *  1.1.9 		[x] 		[]		TinyBMS_ReadNewestEvents
 * 1.1.10 		[x] 		[]		TinyBMS_ReadAllEvents
 * 1.1.11 		[x] 		[]		TinyBMS_ReadBatteryPackVoltage
 * 1.1.12 		[x] 		[]		TinyBMS_ReadBatteryPackCurrent
 * 1.1.13 		[] 			[]		TinyBMS_ReadBatteryPackMaxCellVoltage
 * 1.1.14 		[] 			[]		TinyBMS_ReadBatteryPackMinCellVoltage
 * 1.1.15 		[] 			[]		TinyBMS_ReadOnlineStatus
 * 1.1.16 		[] 			[]		TinyBMS_ReadLifetimeCounter
 * 1.1.17 		[] 			[]		TinyBMS_ReadEstimatedSOCValue
 * 1.1.18 		[x] 		[]		TinyBMS_ReadDeviceTemperatures
 * 1.1.19 		[] 			[]		TinyBMS_ReadBatteryPackCellVoltages
 * 1.1.20 		[] 			[]		TinyBMS_ReadSettingsValues
 * 1.1.21 		[] 			[]		TinyBMS_ReadVersion
 * 1.1.22 		[] 			[]		TinyBMS_ReadVersionExtended
 * 1.1.23 		[] 			[]		TinyBMS_ReadCalcSpeedDistanceLeftEstTimeLeft
 *
 */


/**************** Header Files ****************/
#include "TinyBMS.h"

/*************** Static Function Prototypes **************/
static uint8_t TinyBMS_ACK(UART_HandleTypeDef *huart2);
static uint8_t TinyBMS_ReadRegBlock(UART_HandleTypeDef *huart2, uint8_t rl, uint16_t addr);
static uint8_t TinyBMS_ReadRegIndividual(UART_HandleTypeDef *huart2, uint8_t pl, uint16_t* addr);
static uint8_t TinyBMS_WriteRegBlock(UART_HandleTypeDef *huart2, uint8_t pl, uint16_t addr, uint16_t* data);
static uint8_t TinyBMS_WriteRegIndividual(UART_HandleTypeDef *huart2, uint8_t pl, uint16_t* addr, uint16_t* data);
static uint8_t TinyBMS_ReadRegBlockMODBUS(UART_HandleTypeDef *huart2, uint16_t addr, uint8_t rl);
static uint8_t TinyBMS_WriteRegBlockMODBUS(UART_HandleTypeDef *huart2, uint16_t addr, uint8_t rl, uint8_t pl, uint16_t* data);

/* CRC Calculation Function Prototype */
static uint16_t CRC16(const uint8_t* data, uint16_t length);

/***************** Functions ******************/


/* ********************************
 *  TinyBMS UART Communication API
 * ******************************** */

/* Note1: UART configuration: baudrate 115200 bit/s, 8 data bits, 1 stop bit, no parity, no flow control. UART
 *        configuration is not allowed to be changed by the user. */

/* Note2: If Tiny BMS device is in sleep mode, the first command must be send twice. After received the first
 *        command BMS wakes up from sleep mode, but the response to the command will be sent when it
 *        receives the command a second time. Tiny BMS does not enter sleep mode again while communication is
 *        ongoing. */

/* Note3: The traditional UART commands are interpreted LSB first (Little Endian), while the MODBUS compatible
 *        commands are interpreted MSB first (Big Endian). CRC bytes are always in Little Endian.
 *        MODBUS R/W commands are also Block only, and not individual */

/* Note4: PL (Payload Length in bytes) is always 2*n registers, while RL (Register length) is just n registers. */


//1.1.1 TinyBMS Acknowledgment
/* ******************************************************************************
 * @fn					-  TinyBMS_ACK
 *
 * @brief				-  Deciphers the handshake (ACK or NACK) with the TinyBMS
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-  0x00 - CMD ERROR , 0x01 - CRC ERROR
 *
 */
static uint8_t TinyBMS_ACK(UART_HandleTypeDef *huart2) {

	printf("TinyBMS_ACK\n");
	uint8_t retval = 0xFF;

	uint8_t rx_buffer[50];
	uint8_t cmd = 0;
	uint16_t CRC_calc = 0, CRC_reply = 0;

	/* Response from BMS */
	HAL_UART_Receive_IT(huart2, (uint8_t *)rx_buffer, 2); //read bytes 1-2 to check ACK/NACK

	if(rx_buffer[0] == 0xAA) {

		//[NACK]
		if(rx_buffer[1] == 0x00) {
			printf("Response from BMS [NACK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 4); //read bytes 3-6 from NACK reply

			cmd = rx_buffer[2];
			printf("cmd: 0x%02X\n", cmd);

			uint8_t error = rx_buffer[3];

			if(error == 0x00) {
				printf("CMD ERROR\n");
			} else if(error == 0x01) {
				printf("CRC ERROR\n");
			} else {
				printf("Error: Byte 4 should be 0x00 or 0x01 but was 0x%02X\n", error);
			}

			CRC_reply = ((rx_buffer[5] << 8) | rx_buffer[4]);
			CRC_calc = CRC16(rx_buffer, 4); //Calc CRC for bytes 1-4 of NACK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");
				printf("ERROR Code: 0x%02X\n", error); //valid error code
				retval = error;
			} else {
				printf("CRC fail in BMS ERROR\n");
				retval = 0xFF; //failure
			}

		//[ACK]
		} else if(rx_buffer[1] == 0x01) {
			printf("Response from BMS [ACK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 3); //read bytes 3-5 from ACK reply

			cmd = rx_buffer[2];
			printf("cmd: 0x%02X\n", cmd);

			CRC_reply = ((rx_buffer[4] << 8) | rx_buffer[3]);
			CRC_calc = CRC16(rx_buffer, 3); //Calc CRC for bytes 1-3 of ACK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");
				retval = 0xAA; //success
			} else {
				printf("CRC fail in BMS ACK\n");
				retval = 0xFF; //failure
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x01 but was 0x%02X\n", rx_buffer[1]);
			retval = 0xFF; //failure
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = 0xFF; //failure
	}

	return retval;
}

//1.1.2 Read TinyBMS Registers Block
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadRegBlock
 *
 * @brief				-  Read from a register block
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 * @param[in]			-  rl - Number (length) of registers to read (uint8_t)
 * @param[in]			-  addr - First register's block address (LSB first) (uint16_t)
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-  Returns a pointer to the address of the first word of the data block (LSB first).
 * 						   	A memory block is a group of one or more contiguous bytes of memory allocated
 * 						  	by malloc(size_t size).
 */
static uint8_t TinyBMS_ReadRegBlock(UART_HandleTypeDef *huart2, uint8_t rl, uint16_t addr) {

	uint8_t retval = 0xFF;

	return retval;
}

//1.1.3 Read TinyBMS Individual Registers
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadRegIndividual
 *
 * @brief				-  Read from a single (or multiple) individual registers
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 * @param[in]			-  pl - Payload Length in bytes (byte length = 2*n registers . Every register
 * 							contains 2 bytes) (uint8_t)
 * @param[in]			-  addr - a pointer to the first address of data structure containing the register
 * 							addresses to be read from (LSB first) (uint16_t*)
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-  The order of data values returned will match the order of addresses given.
 * 										Individual Register Response from BMS [OK]:
 * 						 	Byte n*4   Byte n*4+1  Byte n*4+2  Byte n*4+3  Byte n*4+4  Byte n*4+5
 * 						 	ADDRn:LSB  ADDRn:MSB   DATAn:LSB   DATAn:MSB   CRC:LSB     CRC:MSB
 * 						 	      [UINT16]			     [UINT16]
 */
static uint8_t TinyBMS_ReadRegIndividual(UART_HandleTypeDef *huart2, uint8_t pl, uint16_t* addr) {

	uint8_t retval = 0xFF;
	return retval;

	//Placeholder code
    /*
    memset(bl_rx_buffer,0,200);

	for(uint32_t i = 0; i < strlen(payload); i++) {
		HAL_UART_Transmit_IT(&huart2, (uint8_t*)&tx_buffer[i], 200);
	}

	HAL_UART_Receive_IT(&huart1, (uint8_t *)rx_buffer, 1);*/
}

//1.1.4 Write TinyBMS Registers Block
/* ******************************************************************************
 * @fn					-  TinyBMS_WriteRegBlock
 *
 * @brief				-  Write to a register block
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 * @param[in]			-  pl - Payload Length in bytes (2*n registers) (uint8_t)
 * @param[in]			-  addr - First register's block address (LSB first) (uint16_t)
 * @param[in]			-  data - a pointer to the first address of data structure containing the register
 * 							values (LSB first) (uint16_t*)
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-  A memory block is a group of one or more contiguous bytes of memory allocated by malloc(size_t size).
 */
static uint8_t TinyBMS_WriteRegBlock(UART_HandleTypeDef *huart2, uint8_t pl, uint16_t addr, uint16_t* data) {
	uint8_t retval = 0xFF;
	return retval;
}

//1.1.5 Write TinyBMS Individual Registers
/* ******************************************************************************
 * @fn					-  TinyBMS_WriteRegIndividual
 *
 * @brief				-  Write to single (or multiple) individual registers
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 * @param[in]			-  pl - Payload Length in bytes (2*n registers) (uint8_t)
 * @param[in]			-  addr - a pointer to the first address of data structure containing the register
 * 							addresses to be read from (LSB first) (uint16_t*) *NOTE*
 * @param[in]			-  data - a pointer to the first address of data structure containing the register
 * 							values (LSB first) (uint16_t*) *NOTE*
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-  The order of data values given coincides with the order of addresses given.
 * 										Individual Register request to BMS:
 * 						 	Byte n*4   Byte n*4+1  Byte n*4+2  Byte n*4+3  Byte n*4+4  Byte n*4+5
 * 						 	ADDRn:LSB  ADDRn:MSB   DATAn:LSB   DATAn:MSB   CRC:LSB     CRC:MSB
 * 						 	      [UINT16]			     [UINT16]
 */
static uint8_t TinyBMS_WriteRegIndividual(UART_HandleTypeDef *huart2, uint8_t pl, uint16_t* addr, uint16_t* data) {
	uint8_t retval = 0xFF;
	return retval;
}

//1.1.6 Read TinyBMS Registers Block (MODBUS compatible)
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadRegBlockMODBUS
 *
 * @brief				-  Read from a register block (MODBUS compatible)
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 * @param[in]			-  addr - First register's block address (MSB first!!) (uint16_t)
 * @param[in]			-  rl - Number (length) of registers to read (uint8_t) **Max 127 registers (0x7F)**
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-  A memory block is a group of one or more contiguous bytes of memory allocated by malloc(size_t size).
 * 						   MODBUS addresses and data is MSB first (Big Endian). CRC however is still LSB First! (Little Endian)
 * 							  Individual Register Response from BMS [OK]:
 * 						 	  Byte n*2+2  Byte n*2+3  Byte n*2+4  Byte n*2+5
 * 						 	  DATAn:MSB   DATAn:LSB   CRC:LSB     CRC:MSB
 * 						 	        [UINT16]
 */
static uint8_t TinyBMS_ReadRegBlockMODBUS(UART_HandleTypeDef *huart2, uint16_t addr, uint8_t rl) {
	uint8_t retval = 0xFF;
	return retval;
}

//1.1.7 Write TinyBMS Registers Block (MODBUS compatible)
/* ******************************************************************************
 * @fn					-  TinyBMS_WriteRegBlockMODBUS
 *
 * @brief				-  Write to a register block (MODBUS compatible)
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 * @param[in]			-  addr - First register's block address (MSB first!!) (uint16_t)
 * @param[in]			-  rl - Number (length) of registers to write (uint8_t) **Max 100 registers (0x64)**
 * @param[in]			-  pl - Payload Length in bytes (2*n registers) (uint8_t)
 * @param[in]			-  data - a pointer to the first address of data structure containing the register
 * 							values (MSB first!!) (uint16_t*) *NOTE*
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-  A memory block is a group of one or more contiguous bytes of memory allocated by malloc(size_t size).
 * 						   MODBUS addresses and data is MSB first (Big Endian). CRC however is still LSB First! (Little Endian)
 * 							  Individual Register Response from BMS [OK]:
 * 						 	  Byte n*2+6  Byte n*2+7  Byte n*2+8  Byte n*2+9
 * 						 	  DATAn:MSB   DATAn:LSB   CRC:LSB     CRC:MSB
 * 						 	        [UINT16]
 */
static uint8_t TinyBMS_WriteRegBlockMODBUS(UART_HandleTypeDef *huart2, uint16_t addr, uint8_t rl, uint8_t pl, uint16_t* data) {
	uint8_t retval = 0xFF;
	return retval;
}

//1.1.8 Reset TinyBMS, Clear Events and Statistics
/* ******************************************************************************
 * @fn					-  TinyBMS_ResetClearEventsStatistics
 *
 * @brief				-  Reset BMS, Clear Events, or Clear Statistics depending on option
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 * @param[in]			-  option (uint8_t) *NOTE*
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-  0x01 - Clear Events , 0x02 - Clear Statistics , 0x05 - Reset BMS
 *
 */
uint8_t TinyBMS_ResetClearEventsStatistics(UART_HandleTypeDef *huart2, uint8_t option) {
	printf("TinyBMS_ResetClearEventsStatistics\n");
	uint8_t retval = 0xFF;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	//Check if invalid option
	if((option != 0x01) && (option != 0x02) && (option != 0x05)) {
		printf("Invalid option: 0x%02X\n", option);
		retval = 0xFF;
		return retval;
	}

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x02;
	tx_buffer[2] = option; //check notes above

	CRC_request = CRC16(tx_buffer, 3);
	CRC_LSB = (CRC_request & 0xFF);
	CRC_MSB = ((CRC_request >> 8) & 0xFF);
	/*
	printf("CRC_request: 0x%04X\n", CRC_request);
	printf("CRC_LSB: 0x%02X\n", CRC_LSB);
	printf("CRC_MSB: 0x%02X\n", CRC_MSB);
	*/

	tx_buffer[3] = CRC_LSB;
	tx_buffer[4] = CRC_MSB;

	HAL_UART_Transmit_IT(huart2, (uint8_t *)tx_buffer, 5);

	/* Response from BMS */
	HAL_UART_Receive_IT(huart2, (uint8_t *)rx_buffer, 2); //read bytes 1-2 to check ACK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == 0x00) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

			uint8_t error = rx_buffer[3];

			CRC_reply = ((rx_buffer[5] << 8) | rx_buffer[4]);
			CRC_calc = CRC16(rx_buffer, 4); //Calc CRC for bytes 1-4 of ERROR response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");
				printf("ERROR Code: 0x%02X\n", error); //valid error code
				retval = error;
			} else {
				printf("CRC fail in BMS ERROR\n");
				retval = 0xFF; //failure
			}

		//[ACK]
		} else if(rx_buffer[1] == 0x01) {
			printf("Response from BMS [ACK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 3); //read bytes 3-5 from ACK reply

			CRC_reply = ((rx_buffer[4] << 8) | rx_buffer[3]);
			CRC_calc = CRC16(rx_buffer, 3); //Calc CRC for bytes 1-3 of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");
				retval = 0xAA; //success
			} else {
				printf("CRC fail in BMS ACK\n");
				retval = 0xFF; //failure
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x1B but was 0x%02X\n", rx_buffer[1]);
			retval = 0xFF; //failure
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = 0xFF; //failure
	}

	return retval;
}

//1.1.9 Read TinyBMS Newest Events
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadNewestEvents
 *
 * @brief				-  Read BMS newest events with timestamps and event ID's
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-  PL: Payload length in bytes [UINT8]. BTSP: BMS timestamp in seconds [UINT32].
 * 						   TSP: Event timestamp in seconds [UINT24]. EVENT: BMS Event ID [UINT8].
 */
uint8_t TinyBMS_ReadNewestEvents(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_ReadNewestEvents\n");
	uint8_t retval = 0xFF;

	uint8_t tx_buffer[50], rx_buffer[1000];
	uint8_t PL = 0, CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x11;

	CRC_request = CRC16(tx_buffer, 2);
	CRC_LSB = (CRC_request & 0xFF);
	CRC_MSB = ((CRC_request >> 8) & 0xFF);
	/*
	printf("CRC_request: 0x%04X\n", CRC_request);
	printf("CRC_LSB: 0x%02X\n", CRC_LSB);
	printf("CRC_MSB: 0x%02X\n", CRC_MSB);
	*/

	tx_buffer[2] = CRC_LSB;
	tx_buffer[3] = CRC_MSB;

	HAL_UART_Transmit_IT(huart2, (uint8_t *)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(huart2, (uint8_t *)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == 0x00) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

			uint8_t error = rx_buffer[3];

			CRC_reply = ((rx_buffer[5] << 8) | rx_buffer[4]);
			CRC_calc = CRC16(rx_buffer, 4); //Calc CRC for bytes 1-4 of ERROR response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");
				printf("ERROR Code: 0x%02X\n", error); //valid error code
				retval = error;
			} else {
				printf("CRC fail in BMS ERROR\n");
				retval = 0xFF; //failure
			}

		//[OK]
		} else if(rx_buffer[1] == 0x11) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 1); //read byte 3 from OK reply

			//Total Events: (0x01-0x90) or 144 events
			PL = rx_buffer[2];
			//Payload Length = PL = 4n+7 bytes

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[3], PL); //read from byte 4 to 4n+9 from OK reply

			uint32_t BTSP = 0;
			uint32_t TSP[] = {0}; 	//uint24_t stored in a uint32_t
			uint8_t Event[] = {0};
			uint32_t n = 0;

			for(uint32_t i = 0; i < PL; i++) {
				if(i >= 3) {
					//BTSP BMS Timestamp in seconds -> LSB = Byte4, MSB = Byte7 (i.e. rx_buffer[3:6])
					BTSP |= (rx_buffer[3+i] << (8 * i));
				} else {
					//n signifies the event number, for 4 <= i < PL
					n = (i - 3);

					//TSPn Event Timestamp in seconds -> LSB = Byte4n+4, MSB = Byte4n+6 (i.e. rx_buffer[4n+4-1 : 4n+6-1])
					//*Remember that TSP data is a uint24_t stored in a uint32_t, so ignore the MSB of the array element*
					TSP[n-1] = ((rx_buffer[(4*n)+6-1] << 16) | (rx_buffer[(4*n)+5-1] << 8) | (rx_buffer[(4*n)+4-1]));

					//BMS Eventn ID -> Byte4n+7 (i.e. rx_buffer [4n+7-1])
					Event[n-1] |= rx_buffer[(4*n)+7-1];
				}
			}

			CRC_reply = ((rx_buffer[(4*n)+9-1] << 8) | rx_buffer[(4*n)+8-1]);
			CRC_calc = CRC16(rx_buffer, PL+3); //Calc CRC for bytes (PL+3 or Payload Value plus Bytes 1,2,3)) of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");
				retval = 0xAA; //success

				//Print "Newest Events" Timestamps with their Event IDs
				for(uint32_t i = 0; i < n; i++) {
					if(i == 0) {
						//could convert seconds into a meaningful hh:mm:ss time later
						printf("************ TinyBMS Newest Events ************");
						printf("BMS Timestamp (s): %lu\n", BTSP);
						printf("----------------------------------------");
					}
					//could convert seconds into a meaningful hh:mm:ss time later
					printf("Event Timestamp (s): %lu \t Event ID: 0x%02X\n", TSP[i], Event[i]);
				}

			} else {
				printf("CRC fail in BMS OK\n");
				retval = 0xFF; //failure
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x11 but was 0x%02X\n", rx_buffer[1]);
			retval = 0xFF; //failure
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = 0xFF; //failure
	}

	return retval;
}

//1.1.10 Read TinyBMS All Events
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadAllEvents
 *
 * @brief				-  Read BMS all events with timestamps and event ID's
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-  PL: Payload length in bytes [UINT8]. BTSP: BMS timestamp in seconds [UINT32].
 * 						   TSP: Event timestamp in seconds [UINT24]. EVENT: BMS Event ID [UINT8].
 */
uint8_t TinyBMS_ReadAllEvents(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_ReadAllEvents\n");
	uint8_t retval = 0xFF;

	uint8_t tx_buffer[50], rx_buffer[1000];
	uint8_t PL = 0, CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x12;

	CRC_request = CRC16(tx_buffer, 2);
	CRC_LSB = (CRC_request & 0xFF);
	CRC_MSB = ((CRC_request >> 8) & 0xFF);
	/*
	printf("CRC_request: 0x%04X\n", CRC_request);
	printf("CRC_LSB: 0x%02X\n", CRC_LSB);
	printf("CRC_MSB: 0x%02X\n", CRC_MSB);
	*/

	tx_buffer[2] = CRC_LSB;
	tx_buffer[3] = CRC_MSB;

	HAL_UART_Transmit_IT(huart2, (uint8_t *)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(huart2, (uint8_t *)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == 0x00) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

			uint8_t error = rx_buffer[3];

			CRC_reply = ((rx_buffer[5] << 8) | rx_buffer[4]);
			CRC_calc = CRC16(rx_buffer, 4); //Calc CRC for bytes 1-4 of ERROR response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");
				printf("ERROR Code: 0x%02X\n", error); //valid error code
				retval = error;
			} else {
				printf("CRC fail in BMS ERROR\n");
				retval = 0xFF; //failure
			}

		//[OK]
		} else if(rx_buffer[1] == 0x12) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 1); //read byte 3 from OK reply

			//Total Events: (0x01-0x90) or 144 events
			PL = rx_buffer[2];
			//Payload Length = PL = 4n+7 bytes

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[3], PL); //read from byte 4 to 4n+9 from OK reply

			uint32_t BTSP = 0;
			uint32_t TSP[] = {0}; 	//uint24_t stored in a uint32_t
			uint8_t Event[] = {0};
			uint32_t n = 0;

			for(uint32_t i = 0; i < PL; i++) {
				if(i >= 3) {
					//BTSP BMS Timestamp in seconds -> LSB = Byte4, MSB = Byte7 (i.e. rx_buffer[3:6])
					BTSP |= (rx_buffer[3+i] << (8 * i));
				} else {
					//n signifies the event number, for 4 <= i < PL
					n = (i - 3);

					//TSPn Event Timestamp in seconds -> LSB = Byte4n+4, MSB = Byte4n+6 (i.e. rx_buffer[4n+4-1 : 4n+6-1])
					//*Remember that TSP data is a uint24_t stored in a uint32_t, so ignore the MSB of the array element*
					TSP[n-1] = ((rx_buffer[(4*n)+6-1] << 16) | (rx_buffer[(4*n)+5-1] << 8) | (rx_buffer[(4*n)+4-1]));

					//BMS Eventn ID -> Byte4n+7 (i.e. rx_buffer [4n+7-1])
					Event[n-1] |= rx_buffer[(4*n)+7-1];
				}
			}

			CRC_reply = ((rx_buffer[(4*n)+9-1] << 8) | rx_buffer[(4*n)+8-1]);
			CRC_calc = CRC16(rx_buffer, PL+3); //Calc CRC for bytes (PL+3 or Payload Value plus Bytes 1,2,3)) of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");
				retval = 0xAA; //success

				//Print "All Events" Timestamps with their Event IDs
				for(uint32_t i = 0; i < n; i++) {
					if(i == 0) {
						//could convert seconds into a meaningful hh:mm:ss time later
						printf("************ TinyBMS All Events ************");
						printf("BMS Timestamp (s): %lu\n", BTSP);
						printf("----------------------------------------");
					}
					//could convert seconds into a meaningful hh:mm:ss time later
					printf("Event Timestamp (s): %lu \t Event ID: 0x%02X\n", TSP[i], Event[i]);
				}

			} else {
				printf("CRC fail in BMS OK\n");
				retval = 0xFF; //failure
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x12 but was 0x%02X\n", rx_buffer[1]);
			retval = 0xFF; //failure
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = 0xFF; //failure
	}

	return retval;
}

//1.1.11 Read Battery Pack Voltage
//Reg:36
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadBatteryPackVoltage
 *
 * @brief				-  Read Battery Pack Voltage - 1V Resolution
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-  			Response from BMS [OK]:
 * 						 	  Byte3      Byte4  Byte5  	Byte6 		Byte7 	Byte8
 * 						 	  DATA:LSB   DATA   DATA	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	               [FLOAT]
 */
uint8_t TinyBMS_ReadBatteryPackVoltage(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_ReadBatteryPackVoltage\n");
	uint8_t retval = 0xFF;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x14;

	CRC_request = CRC16(tx_buffer, 2);
	CRC_LSB = (CRC_request & 0xFF);
	CRC_MSB = ((CRC_request >> 8) & 0xFF);
	/*
	printf("CRC_request: 0x%04X\n", CRC_request);
	printf("CRC_LSB: 0x%02X\n", CRC_LSB);
	printf("CRC_MSB: 0x%02X\n", CRC_MSB);
	*/

	tx_buffer[2] = CRC_LSB;
	tx_buffer[3] = CRC_MSB;

	HAL_UART_Transmit_IT(huart2, (uint8_t *)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(huart2, (uint8_t *)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == 0x00) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

			uint8_t error = rx_buffer[3];

			CRC_reply = ((rx_buffer[5] << 8) | rx_buffer[4]);
			CRC_calc = CRC16(rx_buffer, 4); //Calc CRC for bytes 1-4 of ERROR response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");
				printf("ERROR Code: 0x%02X\n", error); //valid error code
				retval = error;
			} else {
				printf("CRC fail in BMS ERROR\n");
				retval = 0xFF; //failure
			}

		//[OK]
		} else if(rx_buffer[1] == 0x14) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 6); //read bytes 3-8 from OK reply

			uint32_t data = ((rx_buffer[5] << 24) | (rx_buffer[4] << 16) | (rx_buffer[3] << 8) | (rx_buffer[2]));
		    //printf("0x%08X\n", data);

			CRC_reply = ((rx_buffer[7] << 8) | rx_buffer[6]);
			CRC_calc = CRC16(rx_buffer, 6); //Calc CRC for bytes 1-6 of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");
				retval = 0xAA; //success

				float batteryPackVoltage = data;
				printf("%f\n", batteryPackVoltage);

			} else {
				printf("CRC fail in BMS OK\n");
				retval = 0xFF; //failure
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x14 but was 0x%02X\n", rx_buffer[1]);
			retval = 0xFF; //failure
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = 0xFF; //failure
	}

	return retval;
}

//1.1.12 Read Battery Pack Current
//Reg:38
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadBatteryPackCurrent
 *
 * @brief				-  Read Battery Pack Current - 1A Resolution
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-  			Response from BMS [OK]:
 * 						 	  Byte3      Byte4  Byte5  	Byte6 		Byte7 	Byte8
 * 						 	  DATA:LSB   DATA   DATA	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	               [FLOAT]
 */
uint8_t TinyBMS_ReadBatteryPackCurrent(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_ReadBatteryPackCurrent\n");
	uint8_t retval = 0xFF;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x15;

	CRC_request = CRC16(tx_buffer, 2);
	CRC_LSB = (CRC_request & 0xFF);
	CRC_MSB = ((CRC_request >> 8) & 0xFF);
	/*
	printf("CRC_request: 0x%04X\n", CRC_request);
	printf("CRC_LSB: 0x%02X\n", CRC_LSB);
	printf("CRC_MSB: 0x%02X\n", CRC_MSB);
	*/

	tx_buffer[2] = CRC_LSB;
	tx_buffer[3] = CRC_MSB;

	HAL_UART_Transmit_IT(huart2, (uint8_t *)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(huart2, (uint8_t *)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == 0x00) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

			uint8_t error = rx_buffer[3];

			CRC_reply = ((rx_buffer[5] << 8) | rx_buffer[4]);
			CRC_calc = CRC16(rx_buffer, 4); //Calc CRC for bytes 1-4 of ERROR response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");
				printf("ERROR Code: 0x%02X\n", error); //valid error code
				retval = error;
			} else {
				printf("CRC fail in BMS ERROR\n");
				retval = 0xFF; //failure
			}

		//[OK]
		} else if(rx_buffer[1] == 0x15) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 6); //read bytes 3-8 from OK reply

			uint32_t data = ((rx_buffer[5] << 24) | (rx_buffer[4] << 16) | (rx_buffer[3] << 8) | (rx_buffer[2]));
		    //printf("0x%08X\n", data);

			CRC_reply = ((rx_buffer[7] << 8) | rx_buffer[6]);
			CRC_calc = CRC16(rx_buffer, 6); //Calc CRC for bytes 1-6 of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");
				retval = 0xAA; //success

				float batteryPackCurrent = data;
				printf("%f\n", batteryPackCurrent);

			} else {
				printf("CRC fail in BMS OK\n");
				retval = 0xFF; //failure
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x15 but was 0x%02X\n", rx_buffer[1]);
			retval = 0xFF; //failure
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = 0xFF; //failure
	}

	return retval;
}

//1.1.13 Read Battery Pack Max Cell Voltage
//Reg:41
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadBatteryPackMaxCellVoltage
 *
 * @brief				-  Read Battery Pack Maximum Cell Voltage - 1mV Resolution
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-  			Response from BMS [OK]:
 * 						 	  Byte3        	Byte4 		Byte5 	Byte6
 * 						 	  DATA:LSB   	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        [UINT16]
 */
uint8_t TinyBMS_ReadBatteryPackMaxCellVoltage(UART_HandleTypeDef *huart2) {
	uint8_t retval = 0xFF;
	return retval;
}

//1.1.14 Read Battery Pack Min Cell Voltage
//Reg:40
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadBatteryPackMinCellVoltage
 *
 * @brief				-  Read Battery Pack Minimum Cell Voltage - 1mV Resolution
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				- 			Response from BMS [OK]:
 * 						 	  Byte3        	Byte4 		Byte5 	Byte6
 * 						 	  DATA:LSB   	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        [UINT16]
 */
uint8_t TinyBMS_ReadBatteryPackMinCellVoltage(UART_HandleTypeDef *huart2) {
	uint8_t retval = 0xFF;
	return retval;
}

//1.1.15 Read TinyBMS Online Status
//Reg:50
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadOnlineStatus
 *
 * @brief				-  Read TinyBMS Online Status
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-			Response from BMS [OK]:
 * 						 	  Byte3        	Byte4 		Byte5 	Byte6
 * 						 	  DATA:LSB   	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        [UINT16]
 * 						   0x91 - Charging [INFO], 0x92 - Fully Charged [INFO]
 * 						   0x93 - Discharging [INFO], 0x94 - Regeneration [INFO]
 * 						   0x97 - Idle [INFO], 0x9B - Fault [ERROR]
 */
uint8_t TinyBMS_ReadOnlineStatus(UART_HandleTypeDef *huart2) {
	uint8_t retval = 0xFF;
	return retval;
}

//1.1.16 Read TinyBMS Lifetime Counter
//Reg:32
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadLifetimeCounter
 *
 * @brief				-  Read TinyBMS Lifetime Counter - 1s Resolution
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  0xAA Success, 0xFF CRC Failure in BMS OK, (uint8_t) error code
 *
 * @note				-  				Response from BMS [OK]:
 * 						 	  Byte3      Byte4 	Byte5 	Byte6		Byte5 	Byte6
 * 						 	  DATA:LSB   DATA	DATA	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        	  [UINT32]
 */
uint8_t TinyBMS_ReadLifetimeCounter(UART_HandleTypeDef *huart2) {
	uint8_t retval = 0xFF;
	return retval;
}

//1.1.17 Read TinyBMS Estimated SOC Value
//Reg:46
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadEstimatedSOCValue
 *
 * @brief				-  Read TinyBMS Estimated SOC (State of Charge) Value - 0.000 001 % Resolution
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-  				Response from BMS [OK]:
 * 						 	  Byte3      Byte4 	Byte5 	Byte6		Byte5 	Byte6
 * 						 	  DATA:LSB   DATA	DATA	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        	  [UINT32]
 */
uint8_t TinyBMS_ReadEstimatedSOCValue(UART_HandleTypeDef *huart2) {
	uint8_t retval = 0xFF;
	return retval;
}

//1.1.18 Read TinyBMS Device Temperatures
//Reg:48,42,43
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadDeviceTemperatures
 *
 * @brief				-  Read TinyBMS Device Temperatures - 0.1°C Resolution
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-  				Response from BMS [OK]:
 * 						 	Byte4      Byte5 		Byte6 		Byte7		Byte 8	 	Byte 9 		Byte10 	Byte11
 * 						 	DATA1:LSB  DATA1:MSB	DATA2:LSB	DATA2:MSB	DATA3:LSB	DATA3:MSB   CRC:LSB CRC:MSB
 * 						 	      [INT16]				   [INT16] 				  [INT16]
 * 						  	(Reg 48) DATA1 - TinyBMS Internal Temperature
 * 						  	(Reg 42) DATA2 - External Temp Sensor #1 (value of -327689 if NC)
 * 						  	(Reg 43) DATA3 - External Temp Sensor #2 (value of -327689 if NC)
 */
uint8_t TinyBMS_ReadDeviceTemperatures(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_ReadDeviceTemperatures\n");
	uint8_t retval = 0xFF;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t PL = 0, CRC_LSB = 0, CRC_MSB = 0;
	int16_t DATA1 = 0, DATA2 = 0, DATA3 = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x1B; //command

	//Verified!
	CRC_request = CRC16(tx_buffer, 2);		//should be 0x1B3F
	CRC_LSB = (CRC_request & 0xFF); 		//should be 0x3F
	CRC_MSB = ((CRC_request >> 8) & 0xFF);	//should be 0x1B
	/*
	printf("CRC_request: 0x%04X\n", CRC_request);
	printf("CRC_LSB: 0x%02X\n", CRC_LSB);
	printf("CRC_MSB: 0x%02X\n", CRC_MSB);
	*/

	tx_buffer[2] = CRC_LSB;
	tx_buffer[3] = CRC_MSB;

	HAL_UART_Transmit_IT(huart2, (uint8_t *)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(huart2, (uint8_t *)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == 0x00) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

			uint8_t error = rx_buffer[3];

			CRC_reply = ((rx_buffer[5] << 8) | rx_buffer[4]);
			CRC_calc = CRC16(rx_buffer, 4); //Calc CRC for bytes 1-4 of ERROR response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");
				printf("ERROR Code: 0x%02X\n", error); //valid error code
				retval = error;
			} else {
				printf("CRC fail in BMS ERROR\n");
				retval = 0xFF; //failure
			}

		//[OK]
		} else if(rx_buffer[1] == 0x1B) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 9); //read bytes 3-11 from OK reply

			PL = rx_buffer[2]; //payload length
			printf("Payload Length: 0x%02X\n", PL);

			DATA1 = ((rx_buffer[4] << 8) | rx_buffer[3]);
			int16_t internalTemp = DATA1;  //TinyBMS internal temperature

			DATA2 = ((rx_buffer[6] << 8) | rx_buffer[5]); //value of -32768 if not connected
			int16_t externalTemp1 = DATA2; //External Temp Sensor #1

			DATA3 = ((rx_buffer[8] << 8) | rx_buffer[7]); //value of -32768 if not connected
			int16_t externalTemp2 = DATA3; //External Temp Sensor #2

			CRC_reply = ((rx_buffer[10] << 8) | rx_buffer[9]);
			CRC_calc = CRC16(rx_buffer, 9); //Calc CRC for bytes 1-9 of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");
				printf("TinyBMS internal temperature: 0x%04X\n", internalTemp);
				printf("External sensor 1 temperature: 0x%04X\n", externalTemp1);
				printf("External sensor 2 temperature: 0x%04X\n", externalTemp2);
				retval = 0xAA; //success
			} else {
				printf("CRC fail in BMS OK\n");
				retval = 0xFF; //failure
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x1B but was 0x%02X\n", rx_buffer[1]);
			retval = 0xFF; //failure
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = 0xFF; //failure
	}

	return retval;
}

//1.1.19 Read Battery Pack Cell Voltages
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadBatteryPackCellVoltages
 *
 * @brief				-  Read TinyBMS Battery Pack Cell Voltages
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				- 			Response from BMS [OK]:
 * 						 	  Byte n*2+2    Byte n*2+3 		Byte n*2+4 	Byte n*2+5
 * 						 	  DATAn:LSB   	DATAn:MSB   	CRC:LSB 	CRC:MSB
 * 						 	       	 [UINT16]
 */
uint8_t TinyBMS_ReadBatteryPackCellVoltages(UART_HandleTypeDef *huart2) {
	uint8_t retval = 0xFF;
	return retval;
}

//1.1.20 Read TinyBMS Settings Values (min, max, default, current)
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadSettingsValues
 *
 * @brief				-  Read TinyBMS Settings values (min, max, default, current)
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 * @param[in]			-  option - (uint8_t) *NOTE*
 * @param[in]			-  rl - (uint8_t) *NOTE*
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-			Response from BMS [OK]:
 * 						 	  Byte n*2+2    Byte n*2+3 		Byte n*2+4 	Byte n*2+5
 * 						 	  DATAn:LSB   	DATAn:MSB   	CRC:LSB 	CRC:MSB
 * 						 	       	 [UINT16]
 * 						   Options:
 * 						   0x01 - Min. settings     0x02 - Max. settings,
 * 						   0x03 - Default settings  0x04 - Current settings
 *
 * 						   RL - Registers to read. Max. 100 (0x64) registers
 */
uint8_t TinyBMS_ReadSettingsValues(UART_HandleTypeDef *huart2, uint8_t option, uint8_t rl) {
	uint8_t retval = 0xFF;
	return retval;
}

//1.1.21 Read TinyBMS Version
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadVersion
 *
 * @brief				-  Read TinyBMS Version
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-  				Response from BMS [OK]:
 * 						 	  Byte4   Byte5   Byte6  Byte7  	Byte8 		Byte9 	Byte10
 * 						 	  DATA1   DATA2   DATA3  DATA4:LSB 	DATA4:MSB 	CRC:LSB CRC:MSB
 * 						 	 [UINT8] [UINT8] [UINT8] 		[UINT16]
 * 						 	 DATA1 - Hardware version
 * 						 	 DATA2 - Hardware changes version
 * 						 	 DATA3 - Firmware public version
 * 						 	 DATA4 - Firmware internal version
 */
uint8_t TinyBMS_ReadVersion(UART_HandleTypeDef *huart2) {
	uint8_t retval = 0xFF;
	return retval;
}

//1.1.22 Read TinyBMS Extended Version
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadVersionExtended
 *
 * @brief				-  Read TinyBMS Extended Version
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-  				Response from BMS [OK]:
 * 						 	  Byte4   Byte5   Byte6  Byte7  	Byte8 	  Byte9  Byte10	Byte11 	Byte12
 * 						 	  DATA1   DATA2   DATA3  DATA4:LSB 	DATA4:MSB DATA5	 DATA6	CRC:LSB CRC:MSB
 * 						 	 [UINT8] [UINT8] [UINT8] 		[UINT16]	 [UINT8][UINT8]
 * 						 	 DATA1 - Hardware version
 * 						 	 DATA2 - Hardware changes version
 * 						 	 DATA3 - Firmware public version
 * 						 	 DATA4 - Firmware internal version
 * 						 	 DATA5 - Bootloader version
 * 						 	 DATA6 - Register map version
 */
uint8_t TinyBMS_ReadVersionExtended(UART_HandleTypeDef *huart2) {
	uint8_t retval = 0xFF;
	return retval;
}

//1.1.23 Read TinyBMS Calculated Speed, Distance Left and Estimated Time Left Values
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadCalcSpeedDistanceLeftEstTimeLeft
 *
 * @brief				-  Read TinyBMS Calculated Speed, Distance Left, Estimated Time Left
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  0xAA Success, 0xFF Failure, (uint8_t) error code
 *
 * @note				-  								Response from BMS [OK]:
 * 						 	 Byte3 		Byte4  Byte5  Byte6  		Byte7  	  Byte8  Byte9 Byte10		Byte11 	  Byte12 Byte13 Byte14
 * 						 	 DATA1:LSB  DATA1  DATA1  DATA1:MSB 	DATA2:LSB DATA2	 DATA2 DATA2:MSB	DATA3:LSB DATA3  DATA3  DATA3:MSB
 * 						 	              [FLOAT]                             [UINT32]                              [UINT32]
 * 						 	 Byte15  Byte16
 * 						 	 CRC:LSB CRC:MSB
 *
 * 						 	 DATA1 - Speed (km/h)
 * 						 	 DATA2 - Distance left until empty battery (km)
 * 						 	 DATA3 - Estimated time left until empty battery (s)
 */
uint8_t TinyBMS_ReadCalcSpeedDistanceLeftEstTimeLeft(UART_HandleTypeDef *huart2) {
	uint8_t retval = 0xFF;
	return retval;
}


/*********** CRC Calculation ***********/
static uint16_t CRC16(const uint8_t* data, uint16_t length) {
	uint8_t tmp;
	uint16_t crcWord = 0xFFFF;

	while(length--) {
		tmp = *data++ ^ crcWord;
		crcWord >>= 8;
		crcWord ^= crcTable[tmp];
	}
	return crcWord;
}
