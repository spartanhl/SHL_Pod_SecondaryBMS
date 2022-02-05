/***********************************************
* @file TinyBMS.c
* @brief TinyBMS Library - UART and CAN API
* @author Oliver Moore
* @version 1.2
* @date 02-03-2022
***********************************************/

/*
 * UART API Testing:
 *
 *   #    	Testing   Complete    	Function
 *  1.1.1 		[x] 		[]		TinyBMS_UART_ACK
 *  1.1.2 		[x] 		[]		TinyBMS_UART_ReadRegBlock
 *  1.1.3 		[x] 		[]		TinyBMS_UART_ReadRegIndividual
 *  1.1.4 		[x] 		[]		TinyBMS_UART_WriteRegBlock
 *  1.1.5 		[x] 		[]		TinyBMS_UART_WriteRegIndividual
 *  1.1.6 		[x] 		[]		TinyBMS_UART_ReadRegBlockMODBUS
 *  1.1.7 		[x] 		[]		TinyBMS_UART_WriteRegBlockMODBUS
 *
 *  1.1.8 		[x] 		[] 		TinyBMS_UART_ResetClearEventsStatistics
 *  1.1.9 		[x] 		[]		TinyBMS_UART_ReadNewestEvents
 *  1.1.10 		[x] 		[]		TinyBMS_UART_ReadAllEvents
 *  1.1.11 		[x] 		[]		TinyBMS_UART_ReadBatteryPackVoltage
 *  1.1.12 		[x] 		[]		TinyBMS_UART_ReadBatteryPackCurrent
 *  1.1.13 		[x] 		[]		TinyBMS_UART_ReadBatteryPackMaxCellVoltage
 *  1.1.14 		[x] 		[]		TinyBMS_UART_ReadBatteryPackMinCellVoltage
 *  1.1.15 		[x] 		[]		TinyBMS_UART_ReadOnlineStatus
 *  1.1.16 		[x] 		[]		TinyBMS_UART_ReadLifetimeCounter
 *  1.1.17 		[x] 		[]		TinyBMS_UART_ReadEstimatedSOCValue
 *  1.1.18 		[x] 		[]		TinyBMS_UART_ReadDeviceTemperatures
 *  1.1.19 		[x] 		[]		TinyBMS_UART_ReadBatteryPackCellVoltages
 *  1.1.20 		[x] 		[]		TinyBMS_UART_ReadSettingsValues
 *  1.1.21 		[x] 		[]		TinyBMS_UART_ReadVersion
 *  1.1.22 		[x] 		[]		TinyBMS_UART_ReadVersionExtended
 *  1.1.23 		[x] 		[]		TinyBMS_UART_ReadCalcSpeedDistanceLeftEstTimeLeft
 *
 *
 * CAN API Testing:
 *   #    	Testing   Complete    	Function
 *  2.1.1 		[] 		[]			TinyBMS_CAN_ResetClearEventsStatistics
 *  2.1.2 		[] 		[]			TinyBMS_CAN_ReadRegBlock
 *  2.1.3 		[] 		[]			TinyBMS_CAN_WriteRegBlock
 *  2.1.4 		[] 		[]			TinyBMS_CAN_ReadNewestEvents
 *  2.1.5 		[] 		[]			TinyBMS_CAN_ReadAllEvents
 *  2.1.6 		[] 		[]			TinyBMS_CAN_ReadBatteryPackVoltage
 *  2.1.7 		[] 		[]			TinyBMS_CAN_ReadBatteryPackCurrent
 *  2.1.8 		[] 		[]			TinyBMS_CAN_ReadBatteryPackMaxCellVoltage
 *  2.1.9 		[] 		[]			TinyBMS_CAN_ReadBatteryPackMinCellVoltage
 *  2.1.10 		[] 		[]			TinyBMS_CAN_ReadOnlineStatus
 *  2.1.11 		[] 		[]			TinyBMS_CAN_ReadLifetimeCounter
 *  2.1.12 		[] 		[]			TinyBMS_CAN_ReadEstimatedSOCValue
 *  2.1.13 		[] 		[]			TinyBMS_CAN_ReadDeviceTemperatures
 *  2.1.14 		[] 		[]			TinyBMS_CAN_ReadBatteryPackCellVoltages
 *  2.1.15 		[] 		[]			TinyBMS_CAN_ReadSettingsValues
 *  2.1.16 		[] 		[]			TinyBMS_CAN_ReadVersion
 *  2.1.17 		[] 		[]			TinyBMS_CAN_ReadCalcSpeedDistanceLeftEstTimeLeft
 *  2.1.18 		[] 		[]			TinyBMS_CAN_ReadNodeID
 *  2.1.19 		[] 		[]			TinyBMS_CAN_WriteNodeID
 *
 */


/**************** Header Files ****************/
#include "TinyBMS.h"

/*************** Static Function Prototypes **************/
/* CRC Calculation Function Prototype (only used with UART) */
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
 * @fn					-  TinyBMS_UART_ACK
 *
 * @brief				-  Deciphers the handshake (ACK or NACK) with the TinyBMS
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  0x00 - CMD ERROR , 0x01 - CRC ERROR
 *
 */
uint8_t TinyBMS_UART_ACK(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_UART_ACK\n");
	uint8_t retval = CMD_FAILURE;

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
				retval = CMD_FAILURE;
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
				printf("ACK!\n");
				retval = CMD_SUCCESS;
			} else {
				printf("CRC fail in BMS ACK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x01 but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}
	return retval;
}

//1.1.2 Read TinyBMS Registers Block
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ReadRegBlock
 *
 * @brief				-  Read from a register block
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 * @param[in]			-  rl - Number (length) of registers to read (uint8_t)
 * @param[in]			-  addr - First register's block address (LSB first) (uint16_t)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  A memory block is a group of one or more contiguous bytes of memory allocated
 * 						   by malloc(size_t size).
 */
uint8_t TinyBMS_UART_ReadRegBlock(UART_HandleTypeDef *huart2, uint8_t rl, uint16_t addr) {
	printf("TinyBMS_UART_ReadRegBlock\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[1000];
	uint8_t ADDR_LSB = 0, ADDR_MSB = 0, CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x07;

	tx_buffer[2] = rl;

	ADDR_LSB = (addr & 0xFF);
	ADDR_MSB = ((addr >> 8) & 0xFF);
	tx_buffer[3] = ADDR_LSB;
	tx_buffer[4] = ADDR_MSB;

	CRC_request = CRC16(tx_buffer, 5);
	CRC_LSB = (CRC_request & 0xFF);
	CRC_MSB = ((CRC_request >> 8) & 0xFF);
	/*
	printf("CRC_request: 0x%04X\n", CRC_request);
	printf("CRC_LSB: 0x%02X\n", CRC_LSB);
	printf("CRC_MSB: 0x%02X\n", CRC_MSB);
	*/

	tx_buffer[5] = CRC_LSB;
	tx_buffer[6] = CRC_MSB;

	HAL_UART_Transmit_IT(huart2, (uint8_t *)tx_buffer, 7);

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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x07) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 1); //read byte 3 from OK reply

			//Should be equal to (RL * 2) because 2 bytes per register
			uint8_t PL = rx_buffer[2];

			uint8_t bit7 = ((PL >> 7) & 1); //extract bit 7
			uint8_t bits05 = (PL & 63); //extract bits 0-5

			//Todo: Figure out the reasoning behind the differing PL byte configuration
			//if bit7 == 1
			if(bit7) {
				printf("Current packet ID: 0x%02X\n", bits05);
			//else bit7 == 0
			} else {
				printf("Payload size in bytes (last packet): 0x%02X\n", bits05);
			}

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[3], PL+2); //read from byte 4 to 2*n+5 from OK reply

			uint16_t DATA[] = {0};
			uint32_t numDATA = (PL / 2); //2 bytes per data
			uint32_t n = 0;

			for(uint32_t i = 0; i < numDATA; i++) {
				n++;
				//i=0,1,..,n-1 or n = 1,2,..,n
				DATA[i] = (rx_buffer[n+4-1] << 8) | rx_buffer[n+3-1];
			}

			CRC_reply = ((rx_buffer[n+6-1] << 8) | rx_buffer[n+5-1]);
			CRC_calc = CRC16(rx_buffer, PL+3); //Calc CRC for bytes (PL+3 or Payload Value plus Bytes 1,2,3)) of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				//Print the Register Block Contents
				printf("************ TinyBMS Register Block Contents ************\n");
				printf("Printing Register Contents ranging from: 0x%04X to 0x%04X.\n", addr, addr+PL);
				for(uint16_t i = 0; i < numDATA; i++) {
					printf("Register 0x%04X: Value: %u\n", addr+i, DATA[i]);
				}
				printf("----------------------------------------\n");
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x07 but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.3 Read TinyBMS Individual Registers
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ReadRegIndividual
 *
 * @brief				-  Read from a single (or multiple) individual registers
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 * @param[in]			-  pl - Payload Length in bytes (byte length = 2*n registers . Every register
 * 							contains 2 bytes) (uint8_t)
 * @param[in]			-  addr - a pointer to the first address of data structure containing the register
 * 							addresses to be read from (LSB first) (uint16_t*)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-			Individual Register Response from BMS [OK]:
 * 						 	Byte n*4   Byte n*4+1  Byte n*4+2  Byte n*4+3  Byte n*4+4  Byte n*4+5
 * 						 	ADDRn:LSB  ADDRn:MSB   DATAn:LSB   DATAn:MSB   CRC:LSB     CRC:MSB
 * 						 	      [UINT16]			     [UINT16]
 */
uint8_t TinyBMS_UART_ReadRegIndividual(UART_HandleTypeDef *huart2, uint8_t pl, uint16_t addr[]) {
	printf("TinyBMS_UART_ReadRegIndividual\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[1000], rx_buffer[1000];
	uint8_t ADDR_LSB = 0, ADDR_MSB = 0, CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	uint8_t pl_request = pl;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x09;

	tx_buffer[2] = pl_request;
	uint16_t numAddresses = (pl_request / 2); //2 bytes per address
	uint16_t n = 0;

	//input array addr[] contains n uint16_t elements
	for(uint16_t i = 0; i < numAddresses; i++) {
		n++; //n = 1,2,3,.. addr#
		ADDR_LSB = (addr[i] & 0xFF);
		ADDR_MSB = ((addr[i] >> 8) & 0xFF);
		tx_buffer[(2*n)+2-1] = ADDR_LSB; //for n=1,2,3,.. index: 3,5,7,..
		tx_buffer[(2*n)+3-1] = ADDR_MSB; //for n=1,2,3,.. index: 4,6,8,..
	}

	CRC_request = CRC16(tx_buffer, pl_request+3); //Bytes 1:2n+3 or pl+3
	CRC_LSB = (CRC_request & 0xFF);
	CRC_MSB = ((CRC_request >> 8) & 0xFF);
	/*
	printf("CRC_request: 0x%04X\n", CRC_request);
	printf("CRC_LSB: 0x%02X\n", CRC_LSB);
	printf("CRC_MSB: 0x%02X\n", CRC_MSB);
	*/

	tx_buffer[(2*n)+4-1] = CRC_LSB;
	tx_buffer[(2*n)+5-1] = CRC_MSB;

	HAL_UART_Transmit_IT(huart2, (uint8_t *)tx_buffer, pl_request+5); //Bytes 1:2n+5 or pl+5

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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x09) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 1); //read byte 3 from OK reply

			//equal to (RL * 2) because 2 bytes per register
			uint8_t pl_response = rx_buffer[2];

			uint8_t bit7 = ((pl_response >> 7) & 1); //extract bit 7
			uint8_t bits05 = (pl_response & 63); //extract bits 0-5

			//Todo: Figure out the reasoning behind the differing PL byte configuration
			//if bit7 == 1
			if(bit7) {
				printf("Current packet ID: 0x%02X\n", bits05);
			//else bit7 == 0
			} else {
				printf("Payload size in bytes (last packet): 0x%02X\n", bits05);
			}

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[3], pl_response+2); //read from byte 4 to 4*n+5 from OK reply

			uint16_t ADDR[] = {0}, DATA[] = {0};
			uint16_t numDATA = (pl_response / 4); //4 bytes per data
			uint16_t n = 0;

			for(uint16_t i = 0; i < numDATA; i++) {
				n++; //n = 1,2,3,.. addr# data#
				//i=0,1,..,numDATA-1 or n = 1,2,..,numDATA
				ADDR[i] = (rx_buffer[(4*n)+1-1] << 8) | rx_buffer[(4*n)-1];
				DATA[i] = (rx_buffer[(4*n)+3-1] << 8) | rx_buffer[(4*n)+2-1];
			}

			CRC_reply = ((rx_buffer[(4*n)+5-1] << 8) | rx_buffer[(4*n)+4-1]);
			CRC_calc = CRC16(rx_buffer, pl_response+3); //Calc CRC for bytes (PL+3 or Payload Value plus Bytes 1,2,3)) of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				//Print the Register Block Contents
				printf("************ TinyBMS Individual Register Contents ************\n");
				printf("Printing Individual Register Contents:\n");
				for(uint16_t i = 0; i < numDATA; i++) {
					printf("Register 0x%04X: %u\n", ADDR[i], DATA[i]);
				}
				printf("----------------------------------------\n");
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x09 but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.4 Write TinyBMS Registers Block
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_WriteRegBlock
 *
 * @brief				-  Write to a register block
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 * @param[in]			-  pl - Payload Length in bytes (2*n registers) (uint8_t)
 * @param[in]			-  addr - First register's block address (LSB first) (uint16_t)
 * @param[in]			-  data - a pointer to the first address of data structure containing the register
 * 							values (LSB first) (uint16_t*)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  A memory block is a group of one or more contiguous bytes of memory allocated by malloc(size_t size).
 */
uint8_t TinyBMS_UART_WriteRegBlock(UART_HandleTypeDef *huart2, uint8_t pl, uint16_t addr, uint16_t data[]) {
	printf("TinyBMS_UART_WriteRegBlock\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[1000], rx_buffer[50];
	uint8_t ADDR_LSB = 0, ADDR_MSB = 0, DATA_LSB = 0, DATA_MSB = 0, CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x0B;

	tx_buffer[2] = pl;

	uint8_t bit7 = ((pl >> 7) & 1); //extract bit 7
	uint8_t bits05 = (pl & 63); //extract bits 0-5

	//Todo: Figure out the reasoning behind the differing PL byte configuration
	//if bit7 == 1
	if(bit7) {
		printf("Current packet ID: 0x%02X\n", bits05);
	//else bit7 == 0
	} else {
		printf("Payload size in bytes (last packet): 0x%02X\n", bits05);
	}

	//Address out of bounds
	if((addr < 0x012C) || (addr > 0x018F)) {
		printf("Invalid - register address out of bounds. Must be between 0x012C-0x018F\n");
		retval = CMD_FAILURE;
		return retval;
	}

	ADDR_LSB = (addr & 0xFF);
	ADDR_MSB = ((addr >> 8) & 0xFF);
	tx_buffer[3] = ADDR_LSB;
	tx_buffer[4] = ADDR_MSB;

	uint16_t numDATA = ((pl - 2) / 2); //Subtract out 2 bytes for address, then 2 bytes per data
	uint16_t n = 0;

	//input array data[] contains n uint16_t elements
	for(uint16_t i = 0; i < numDATA; i++) {
		n++; //n = 1,2,3,.. data#
		DATA_LSB = (data[i] & 0xFF);
		DATA_MSB = ((data[i] >> 8) & 0xFF);
		tx_buffer[(2*n)+4-1] = DATA_LSB; //for n=1,2,3,.. index: 3,5,7,..
		tx_buffer[(2*n)+5-1] = DATA_MSB; //for n=1,2,3,.. index: 4,6,8,..
	}

	CRC_request = CRC16(tx_buffer, pl+3);
	CRC_LSB = (CRC_request & 0xFF);
	CRC_MSB = ((CRC_request >> 8) & 0xFF);
	/*
	printf("CRC_request: 0x%04X\n", CRC_request);
	printf("CRC_LSB: 0x%02X\n", CRC_LSB);
	printf("CRC_MSB: 0x%02X\n", CRC_MSB);
	*/

	tx_buffer[(2*n)+6-1] = CRC_LSB;
	tx_buffer[(2*n)+7-1] = CRC_MSB;

	HAL_UART_Transmit_IT(huart2, (uint8_t *)tx_buffer, pl+5);

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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x01) {
			printf("Response from BMS [ACK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 3); //read byte 3-5 from ACK reply

			CRC_reply = ((rx_buffer[5] << 8) | rx_buffer[4]);
			CRC_calc = CRC16(rx_buffer, 3); //Calc CRC for bytes 1-3 of ACK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");
				printf("ACK!\n");
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x01 but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.5 Write TinyBMS Individual Registers
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_WriteRegIndividual
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
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  The order of data values given coincides with the order of addresses given.
 * 										Individual Register request to BMS:
 * 						 	Byte n*4   Byte n*4+1  Byte n*4+2  Byte n*4+3  Byte n*4+4  Byte n*4+5
 * 						 	ADDRn:LSB  ADDRn:MSB   DATAn:LSB   DATAn:MSB   CRC:LSB     CRC:MSB
 * 						 	      [UINT16]			     [UINT16]
 */
uint8_t TinyBMS_UART_WriteRegIndividual(UART_HandleTypeDef *huart2, uint8_t pl, uint16_t addr[], uint16_t data[]) {
	printf("TinyBMS_UART_WriteRegIndividual\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[1000], rx_buffer[1000];
	uint8_t ADDR_LSB = 0, ADDR_MSB = 0, DATA_LSB = 0, DATA_MSB = 0, CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x0D;

	tx_buffer[2] = pl;

	uint8_t bit7 = ((pl >> 7) & 1); //extract bit 7
	uint8_t bits05 = (pl & 63); //extract bits 0-5

	//Todo: Figure out the reasoning behind the differing PL byte configuration
	//if bit7 == 1
	if(bit7) {
		printf("Current packet ID: 0x%02X\n", bits05);
	//else bit7 == 0
	} else {
		printf("Payload size in bytes (last packet): 0x%02X\n", bits05);
	}

	uint16_t numELEMENTS = (pl / 4); //2 bytes per address, 2 bytes per data
	uint16_t n = 0;

	//input arrays addr[] data[] contains n uint16_t elements
	for(uint16_t i = 0; i < numELEMENTS; i++) {
		n++; //n = 1,2,3,.. addr# data#

		//Address out of bounds
		if((addr[i] < 0x012C) || (addr[i] > 0x018F)) {
			printf("Invalid - register address out of bounds from index %u. Must be between 0x012C-0x018F\n", i);
			retval = CMD_FAILURE;
			return retval;
		}
		ADDR_LSB = (addr[i] & 0xFF);
		ADDR_MSB = ((addr[i] >> 8) & 0xFF);
		tx_buffer[(4*n)-1] = ADDR_LSB;
		tx_buffer[(4*n)+1-1] = ADDR_MSB;

		DATA_LSB = (data[i] & 0xFF);
		DATA_MSB = ((data[i] >> 8) & 0xFF);
		tx_buffer[(4*n)+2-1] = DATA_LSB; //for n=1,2,3,.. index: 3,5,7,..
		tx_buffer[(4*n)+3-1] = DATA_MSB; //for n=1,2,3,.. index: 4,6,8,..
	}

	CRC_request = CRC16(tx_buffer, pl+3);
	CRC_LSB = (CRC_request & 0xFF);
	CRC_MSB = ((CRC_request >> 8) & 0xFF);
	/*
	printf("CRC_request: 0x%04X\n", CRC_request);
	printf("CRC_LSB: 0x%02X\n", CRC_LSB);
	printf("CRC_MSB: 0x%02X\n", CRC_MSB);
	*/

	tx_buffer[(4*n)+4-1] = CRC_LSB;
	tx_buffer[(4*n)+5-1] = CRC_MSB;

	HAL_UART_Transmit_IT(huart2, (uint8_t *)tx_buffer, pl+5);

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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x01) {
			printf("Response from BMS [ACK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 3); //read bytes 3-5 from ACK reply

			CRC_reply = ((rx_buffer[4] << 8) | rx_buffer[3]);
			CRC_calc = CRC16(rx_buffer, 3); //Calc CRC for bytes (PL+3 or Payload Value plus Bytes 1,2,3)) of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");
				printf("ACK!\n");
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x01 but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.6 Read TinyBMS Registers Block (MODBUS compatible)
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ReadRegBlockMODBUS
 *
 * @brief				-  Read from a register block (MODBUS compatible)
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 * @param[in]			-  addr - First register's block address (MSB first!!) (uint16_t)
 * @param[in]			-  rl - Number (length) of registers to read (uint8_t) **Max 127 registers (0x7F)**
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  A memory block is a group of one or more contiguous bytes of memory allocated by malloc(size_t size).
 * 						   MODBUS addresses and data is MSB first (Big Endian). CRC however is still LSB First! (Little Endian)
 * 							  Individual Register Response from BMS [OK]:
 * 						 	  Byte n*2+2  Byte n*2+3  Byte n*2+4  Byte n*2+5
 * 						 	  DATAn:MSB   DATAn:LSB   CRC:LSB     CRC:MSB
 * 						 	        [UINT16]
 */
uint8_t TinyBMS_UART_ReadRegBlockMODBUS(UART_HandleTypeDef *huart2, uint16_t addr, uint8_t rl) {
	printf("TinyBMS_UART_ReadRegBlockMODBUS\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[1000];
	uint8_t ADDR_LSB = 0, ADDR_MSB = 0, CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x03;

	//First register's block address
	ADDR_MSB = ((addr >> 8) & 0xFF);
	ADDR_LSB = (addr & 0xFF);
	//MSB first
	tx_buffer[2] = ADDR_MSB;
	tx_buffer[3] = ADDR_LSB;

	tx_buffer[4] = 0x00;

	//rl out of bounds
	if((rl <= 0x00) || (rl > 0x7F)) {
		printf("Invalid - registers to read value out of bounds. Max 127 (0x7F)");
		retval = CMD_FAILURE;
		return retval;
	}
	tx_buffer[5] = rl;

	CRC_request = CRC16(tx_buffer, 6);
	CRC_LSB = (CRC_request & 0xFF);
	CRC_MSB = ((CRC_request >> 8) & 0xFF);
	/*
	printf("CRC_request: 0x%04X\n", CRC_request);
	printf("CRC_LSB: 0x%02X\n", CRC_LSB);
	printf("CRC_MSB: 0x%02X\n", CRC_MSB);
	*/

	tx_buffer[6] = CRC_LSB;
	tx_buffer[7] = CRC_MSB;

	HAL_UART_Transmit_IT(huart2, (uint8_t *)tx_buffer, 8);

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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x03) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 1); //read byte 3 from OK reply

			uint8_t PL = rx_buffer[2];

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[3], PL+2); //read from byte 4 to 2*n+5 from OK reply

			uint16_t DATA[] = {0};
			uint32_t numDATA = (PL / 2); //2 bytes per data
			uint32_t n = 0;

			for(uint32_t i = 0; i < numDATA; i++) {
				n++;
				//MSB first
				DATA[i] = (rx_buffer[(2*n)+2-1] << 8) | rx_buffer[(2*n)+3-1];
			}

			CRC_reply = ((rx_buffer[(2*n)+5-1] << 8) | rx_buffer[(2*n)+4-1]);
			CRC_calc = CRC16(rx_buffer, PL+3); //Calc CRC for bytes (PL+3 or Payload Value plus Bytes 1,2,3)) of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				//Print the Register Block Contents
				printf("************ TinyBMS Register Block Contents ************\n");
				for(uint16_t i = 0; i < numDATA; i++) {
					printf("Register 0x%04X: Value: %u\n", addr+i, DATA[i]);
				}
				printf("----------------------------------------\n");
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x03 but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.7 Write TinyBMS Registers Block (MODBUS compatible)
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_WriteRegBlockMODBUS
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
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  A memory block is a group of one or more contiguous bytes of memory allocated by malloc(size_t size).
 * 						   MODBUS addresses and data is MSB first (Big Endian). CRC however is still LSB First! (Little Endian)
 * 							  Individual Register Response from BMS [OK]:
 * 						 	  Byte n*2+6  Byte n*2+7  Byte n*2+8  Byte n*2+9
 * 						 	  DATAn:MSB   DATAn:LSB   CRC:LSB     CRC:MSB
 * 						 	        [UINT16]
 */
uint8_t TinyBMS_UART_WriteRegBlockMODBUS(UART_HandleTypeDef *huart2, uint16_t addr, uint8_t rl, uint8_t pl, uint16_t data[]) {
	printf("TinyBMS_UART_WriteRegBlockMODBUS\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[1000], rx_buffer[50];
	uint8_t ADDR_LSB = 0, ADDR_MSB = 0, DATA_LSB = 0, DATA_MSB = 0, CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x10;

	//MSB first
	ADDR_MSB = ((addr >> 8) & 0xFF);
	ADDR_LSB = (addr & 0xFF);
	tx_buffer[2] = ADDR_MSB;
	tx_buffer[3] = ADDR_LSB;

	tx_buffer[4] = 0x00;

	//rl out of bounds
	if((rl <= 0x00) || (rl > 0x64)) {
		printf("Invalid - registers to write value out of bounds. Max 100 (0x64)");
		retval = CMD_FAILURE;
		return retval;
	}
	tx_buffer[5] = rl;
	tx_buffer[6] = pl;

	uint16_t numDATA = (pl / 2); //2 bytes per data
	uint16_t n = 0;

	//input arrays data[] contains n uint16_t elements
	for(uint16_t i = 0; i < numDATA; i++) {
		n++; //n = 1,2,3,.. data#

		//MSB first
		DATA_MSB = ((data[i] >> 8) & 0xFF);
		DATA_LSB = (data[i] & 0xFF);
		tx_buffer[(2*n)+6-1] = DATA_MSB; //for n=1,2,3,.. index: 4,6,8,..
		tx_buffer[(2*n)+7-1] = DATA_LSB; //for n=1,2,3,.. index: 3,5,7,..
	}

	CRC_request = CRC16(tx_buffer, pl+7);
	CRC_LSB = (CRC_request & 0xFF);
	CRC_MSB = ((CRC_request >> 8) & 0xFF);
	/*
	printf("CRC_request: 0x%04X\n", CRC_request);
	printf("CRC_LSB: 0x%02X\n", CRC_LSB);
	printf("CRC_MSB: 0x%02X\n", CRC_MSB);
	*/

	tx_buffer[(2*n)+8-1] = CRC_LSB;
	tx_buffer[(2*n)+9-1] = CRC_MSB;

	HAL_UART_Transmit_IT(huart2, (uint8_t *)tx_buffer, pl+9);

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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x10) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 6); //read bytes 3-8 from OK reply

			//MSB first
			uint16_t addr_response = ((rx_buffer[2] << 8) | rx_buffer[3]);

			uint8_t rl_response = rx_buffer[5];

			CRC_reply = ((rx_buffer[7] << 8) | rx_buffer[6]);
			CRC_calc = CRC16(rx_buffer, 6); //Calc CRC for bytes 1-5 of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");
				printf("Successful block write of %u registers starting from address: 0x%04X\n", rl_response, addr_response);
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x10 but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.8 Reset TinyBMS, Clear Events and Statistics
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ResetClearEventsStatistics
 *
 * @brief				-  Reset BMS, Clear Events, or Clear Statistics depending on option
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 * @param[in]			-  option (uint8_t) *NOTE*
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  Options: 0x01 - Clear Events , 0x02 - Clear Statistics , 0x05 - Reset BMS
 *
 */
uint8_t TinyBMS_UART_ResetClearEventsStatistics(UART_HandleTypeDef *huart2, uint8_t option) {
	printf("TinyBMS_UART_ResetClearEventsStatistics\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	//Check if invalid option
	if((option != 0x01) && (option != 0x02) && (option != 0x05)) {
		printf("Invalid option: 0x%02X\n", option);
		retval = CMD_FAILURE;
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
				retval = CMD_FAILURE;
			}

		//[ACK]
		} else if(rx_buffer[1] == 0x01) {
			printf("Response from BMS [ACK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 3); //read bytes 3-5 from ACK reply

			CRC_reply = ((rx_buffer[4] << 8) | rx_buffer[3]);
			CRC_calc = CRC16(rx_buffer, 3); //Calc CRC for bytes 1-3 of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				if(option == 0x01)
					printf("Option 0x%02X - Clear Events\n", option);
				else if(option == 0x02)
					printf("Option 0x%02X - Clear Statistics\n", option);
				else if(option == 0x05)
					printf("Option 0x%02X - Reset BMS\n", option);

				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS ACK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x1B but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.9 Read TinyBMS Newest Events
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ReadNewestEvents
 *
 * @brief				-  Read BMS newest events with timestamps and event ID's
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  PL: Payload length in bytes [UINT8]. BTSP: BMS timestamp in seconds [UINT32].
 * 						   TSP: Event timestamp in seconds [UINT24]. EVENT: BMS Event ID [UINT8].
 */
uint8_t TinyBMS_UART_ReadNewestEvents(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_UART_ReadNewestEvents\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[1000];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x11) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 1); //read byte 3 from OK reply

			//Total Events: (0x01-0x90) or 144 events
			uint8_t PL = rx_buffer[2];
			//Payload Length = PL = 4n+4 bytes

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[3], PL+2); //read from byte 4 to 4n+9 from OK reply

			uint32_t BTSP = 0;
			uint32_t TSP[] = {0}; 	//uint24_t stored in a uint32_t
			uint8_t Event[] = {0};
			uint32_t numEvents = ((PL - 4) / 4); //4 bytes for BTSP + 4 bytes per TSP+EVENT reading
			//**1 event = Payload length of 8 Bytes (BTSP + TSP + EVENT), 2 events = 12 Bytes, etc..**
			uint32_t n = 1; //event index

			for(uint32_t i = 0; i < numEvents; i++) {
				if(i == 0) {
					//BTSP BMS Timestamp in seconds -> LSB = Byte4, MSB = Byte7 (i.e. rx_buffer[3:6])
					BTSP = ((rx_buffer[(4*n)+3-1] << 24) | (rx_buffer[(4*n)+2-1] << 16) | (rx_buffer[(4*n)+1-1] << 8) | (rx_buffer[(4*n)-1]));
				}

				//TSPn Event Timestamp in seconds -> LSB = Byte4n+4, MSB = Byte4n+6 (i.e. rx_buffer[4n+4-1 : 4n+6-1])
				//**Remember that TSP data is a uint24_t stored in a uint32_t, so ignore the MSB of the array element**
				TSP[i] = ((rx_buffer[(4*n)+6-1] << 16) | (rx_buffer[(4*n)+5-1] << 8) | (rx_buffer[(4*n)+4-1]));

				//BMS Eventn ID -> Byte(4n+7) (i.e. rx_buffer [4n+7-1])
				Event[i] = rx_buffer[(4*n)+7-1];

				n++;
			}

			CRC_reply = ((rx_buffer[(4*n)+9-1] << 8) | rx_buffer[(4*n)+8-1]);
			CRC_calc = CRC16(rx_buffer, PL+3); //Calc CRC for bytes (PL+3 or Payload Value plus Bytes 1,2,3)) of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				//Print "Newest Events" Timestamps with their Event IDs
				printf("************ TinyBMS Newest Events ************\n");
				for(uint32_t i = 0; i < numEvents; i++) {
					if(i == 0) {
						//could convert seconds into a meaningful hh:mm:ss time later
						printf("BMS Timestamp (s): %lu\n", BTSP);
					}
					//could convert seconds into a meaningful hh:mm:ss time later
					printf("Event Timestamp (s): %lu \t Event ID: 0x%02X\n", TSP[i], Event[i]);
				}
				printf("----------------------------------------\n");
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x11 but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.10 Read TinyBMS All Events
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ReadAllEvents
 *
 * @brief				-  Read BMS all events with timestamps and event ID's
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  PL: Payload length in bytes [UINT8]. BTSP: BMS timestamp in seconds [UINT32].
 * 						   TSP: Event timestamp in seconds [UINT24]. EVENT: BMS Event ID [UINT8].
 */
uint8_t TinyBMS_UART_ReadAllEvents(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_UART_ReadAllEvents\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[1000];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x12) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 1); //read byte 3 from OK reply

			//Total Events: (0x01-0x90) or 144 events
			uint8_t PL = rx_buffer[2];
			//Payload Length = PL = 4n+4 bytes

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[3], PL+2); //read from byte 4 to 4n+9 from OK reply

			uint32_t BTSP = 0;
			uint32_t TSP[] = {0}; 	//uint24_t stored in a uint32_t
			uint8_t Event[] = {0};
			uint32_t numEvents = ((PL - 4) / 4); //4 bytes for BTSP + 4 bytes per TSP+EVENT reading
			//**1 event = Payload length of 8 Bytes (BTSP + TSP + EVENT), 2 events = 12 Bytes, etc..**
			uint32_t n = 1; //event index

			for(uint32_t i = 0; i < numEvents; i++) {
				if(i == 0) {
					//BTSP BMS Timestamp in seconds -> LSB = Byte4, MSB = Byte7 (i.e. rx_buffer[3:6])
					BTSP = ((rx_buffer[(4*n)+3-1] << 24) | (rx_buffer[(4*n)+2-1] << 16) | (rx_buffer[(4*n)+1-1] << 8) | (rx_buffer[(4*n)-1]));
				}

				//TSPn Event Timestamp in seconds -> LSB = Byte4n+4, MSB = Byte4n+6 (i.e. rx_buffer[4n+4-1 : 4n+6-1])
				//**Remember that TSP data is a uint24_t stored in a uint32_t, so ignore the MSB of the array element**
				TSP[i] = ((rx_buffer[(4*n)+6-1] << 16) | (rx_buffer[(4*n)+5-1] << 8) | (rx_buffer[(4*n)+4-1]));

				//BMS Eventn ID -> Byte(4n+7) (i.e. rx_buffer [4n+7-1])
				Event[i] = rx_buffer[(4*n)+7-1];

				n++;
			}

			CRC_reply = ((rx_buffer[(4*n)+9-1] << 8) | rx_buffer[(4*n)+8-1]);
			CRC_calc = CRC16(rx_buffer, PL+3); //Calc CRC for bytes (PL+3 or Payload Value plus Bytes 1,2,3)) of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				//Print "All Events" Timestamps with their Event IDs
				printf("************ TinyBMS All Events ************\n");
				for(uint32_t i = 0; i < numEvents; i++) {
					if(i == 0) {
						//could convert seconds into a meaningful hh:mm:ss time later
						printf("BMS Timestamp (s): %lu\n", BTSP);
					}
					//could convert seconds into a meaningful hh:mm:ss time later
					printf("Event Timestamp (s): %lu \t Event ID: 0x%02X\n", TSP[i], Event[i]);
				}
				printf("----------------------------------------\n");
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x12 but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.11 Read Battery Pack Voltage
//Reg:36
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ReadBatteryPackVoltage
 *
 * @brief				-  Read Battery Pack Voltage - 1V Resolution
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  			Response from BMS [OK]:
 * 						 	  Byte3      Byte4  Byte5  	Byte6 		Byte7 	Byte8
 * 						 	  DATA:LSB   DATA   DATA	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	               [FLOAT]
 */
uint8_t TinyBMS_UART_ReadBatteryPackVoltage(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_UART_ReadBatteryPackVoltage\n");
	uint8_t retval = CMD_FAILURE;

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
				retval = CMD_FAILURE;
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

				float batteryPackVoltage = data;
				printf("Battery Pack Voltage: %f (V)\n", batteryPackVoltage);
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x14 but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.12 Read Battery Pack Current
//Reg:38
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ReadBatteryPackCurrent
 *
 * @brief				-  Read Battery Pack Current - 1A Resolution
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  			Response from BMS [OK]:
 * 						 	  Byte3      Byte4  Byte5  	Byte6 		Byte7 	Byte8
 * 						 	  DATA:LSB   DATA   DATA	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	               [FLOAT]
 */
uint8_t TinyBMS_UART_ReadBatteryPackCurrent(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_UART_ReadBatteryPackCurrent\n");
	uint8_t retval = CMD_FAILURE;

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
				retval = CMD_FAILURE;
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

				float batteryPackCurrent = data;
				printf("Battery Pack Current: %f (A)\n", batteryPackCurrent);
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x15 but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.13 Read Battery Pack Max Cell Voltage
//Reg:41
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ReadBatteryPackMaxCellVoltage
 *
 * @brief				-  Read Battery Pack Maximum Cell Voltage - 1mV Resolution
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  			Response from BMS [OK]:
 * 						 	  Byte3        	Byte4 		Byte5 	Byte6
 * 						 	  DATA:LSB   	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        [UINT16]
 */
uint8_t TinyBMS_UART_ReadBatteryPackMaxCellVoltage(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_UART_ReadBatteryPackMaxCellVoltage\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x16;

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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x16) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 4); //read bytes 3-6 from OK reply

			uint16_t batteryPackMaxCellVoltage = ((rx_buffer[3] << 8) | (rx_buffer[2]));

			CRC_reply = ((rx_buffer[5] << 8) | rx_buffer[4]);
			CRC_calc = CRC16(rx_buffer, 4); //Calc CRC for bytes 1-4 of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				printf("Battery Pack Maximum Cell Voltage: %u (mV)\n", batteryPackMaxCellVoltage);
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x16 but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.14 Read Battery Pack Min Cell Voltage
//Reg:40
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ReadBatteryPackMinCellVoltage
 *
 * @brief				-  Read Battery Pack Minimum Cell Voltage - 1mV Resolution
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				- 			Response from BMS [OK]:
 * 						 	  Byte3        	Byte4 		Byte5 	Byte6
 * 						 	  DATA:LSB   	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        [UINT16]
 */
uint8_t TinyBMS_UART_ReadBatteryPackMinCellVoltage(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_UART_ReadBatteryPackMinCellVoltage\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x17;

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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x17) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 4); //read bytes 3-6 from OK reply

			uint16_t batteryPackMinCellVoltage = ((rx_buffer[3] << 8) | (rx_buffer[2]));

			CRC_reply = ((rx_buffer[5] << 8) | rx_buffer[4]);
			CRC_calc = CRC16(rx_buffer, 4); //Calc CRC for bytes 1-4 of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				printf("Battery Pack Minimum Cell Voltage: %u (mV)\n", batteryPackMinCellVoltage);
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x17 but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.15 Read TinyBMS Online Status
//Reg:50
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ReadOnlineStatus
 *
 * @brief				-  Read TinyBMS Online Status
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-			Response from BMS [OK]:
 * 						 	  Byte3        	Byte4 		Byte5 	Byte6
 * 						 	  DATA:LSB   	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        [UINT16]
 * 						   0x91 - Charging [INFO], 0x92 - Fully Charged [INFO]
 * 						   0x93 - Discharging [INFO], 0x94 - Regeneration [INFO]
 * 						   0x97 - Idle [INFO], 0x9B - Fault [ERROR]
 */
uint8_t TinyBMS_UART_ReadOnlineStatus(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_UART_ReadOnlineStatus\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x18;

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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x18) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 4); //read bytes 3-6 from OK reply

			uint16_t onlineStatus = ((rx_buffer[3] << 8) | (rx_buffer[2]));

			CRC_reply = ((rx_buffer[5] << 8) | rx_buffer[4]);
			CRC_calc = CRC16(rx_buffer, 4); //Calc CRC for bytes 1-4 of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				//Todo: Need to test onlineStatus output, unsure why these values are stored in a 2 byte container (uint16_t)
				//0x0091? MSB 0x09 0x01 LSB?
				printf("************ TinyBMS Online Status: ************\n");
				switch(onlineStatus) {
					case 0x91:
						printf("0x91 - Charging [INFO]\n");
						break;
					case 0x92:
						printf("0x92 - Fully Charged [INFO]\n");
						break;
					case 0x93:
						printf("0x93 - Discharging [INFO]\n");
						break;
					case 0x96:
						printf("0x96 - Regeneration [INFO]\n");
						break;
					case 0x97:
						printf("0x97 - Idle [INFO]\n");
						break;
					case 0x9B:
						printf("0x9B - Fault [ERROR]\n");
						break;
					default:
						printf("Invalid onlineStatus\n");
						break;
				}
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x18 but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.16 Read TinyBMS Lifetime Counter
//Reg:32
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ReadLifetimeCounter
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
uint8_t TinyBMS_UART_ReadLifetimeCounter(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_UART_ReadLifetimeCounter\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x19;

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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x19) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 6); //read bytes 3-8 from OK reply

			uint32_t lifetimeCounter = ((rx_buffer[5] << 24) | (rx_buffer[4] << 16) | (rx_buffer[3] << 8) | (rx_buffer[2]));

			CRC_reply = ((rx_buffer[7] << 8) | rx_buffer[6]);
			CRC_calc = CRC16(rx_buffer, 6); //Calc CRC for bytes 1-6 of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				printf("TinyBMS Lifetime Counter: %lu (s)\n", lifetimeCounter);
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x19 but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.17 Read TinyBMS Estimated SOC Value
//Reg:46
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ReadEstimatedSOCValue
 *
 * @brief				-  Read TinyBMS Estimated SOC (State of Charge) Value - 0.000 001 % Resolution
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  				Response from BMS [OK]:
 * 						 	  Byte3      Byte4 	Byte5 	Byte6		Byte5 	Byte6
 * 						 	  DATA:LSB   DATA	DATA	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        	  [UINT32]
 */
uint8_t TinyBMS_UART_ReadEstimatedSOCValue(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_UART_ReadEstimatedSOCValue\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x1A;

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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x1A) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 6); //read bytes 3-8 from OK reply

			uint32_t estSOC = ((rx_buffer[5] << 24) | (rx_buffer[4] << 16) | (rx_buffer[3] << 8) | (rx_buffer[2]));

			CRC_reply = ((rx_buffer[7] << 8) | rx_buffer[6]);
			CRC_calc = CRC16(rx_buffer, 6); //Calc CRC for bytes 1-6 of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				printf("Estimated State of Charge (SOC): %lu %% (0.000 001 %% resolution)\n", estSOC);
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x19 but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.18 Read TinyBMS Device Temperatures
//Reg:48,42,43
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ReadDeviceTemperatures
 *
 * @brief				-  Read TinyBMS Device Temperatures - 0.1°C Resolution
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  				Response from BMS [OK]:
 * 						 	Byte4      Byte5 		Byte6 		Byte7		Byte 8	 	Byte 9 		Byte10 	Byte11
 * 						 	DATA1:LSB  DATA1:MSB	DATA2:LSB	DATA2:MSB	DATA3:LSB	DATA3:MSB   CRC:LSB CRC:MSB
 * 						 	      [INT16]				   [INT16] 				  [INT16]
 * 						  	(Reg 48) DATA1 - TinyBMS Internal Temperature
 * 						  	(Reg 42) DATA2 - External Temp Sensor #1 (value of -327689 if NC)
 * 						  	(Reg 43) DATA3 - External Temp Sensor #2 (value of -327689 if NC)
 */
uint8_t TinyBMS_UART_ReadDeviceTemperatures(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_UART_ReadDeviceTemperatures\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x1B) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 1); //read byte 3 from OK reply

			uint8_t PL = rx_buffer[2]; //payload length in bytes
			printf("Payload Length: 0x%02X\n", PL);
			//Payload Length = PL = 2n bytes -> where n = 1,2,3 (DATA1,DATA2,DATA3)

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[3], PL+2); //read bytes 4-2n+5 from OK reply

			int16_t DATA[] = {0};
			uint32_t numData = (PL / 2); //2 bytes per data reading
			uint32_t n = 1; //data index

			for(uint32_t i = 0; i < numData; i++) {
				//TinyBMS Device Temperatures -> LSB = Byte(2n+2) MSB = Byte(2n+3) (i.e. rx_buffer[2n+2 : 2n+3])
				DATA[i] = ((rx_buffer[(2*n)+3-1] << 8) | (rx_buffer[(2*n)+2-1]));
				n++;
			}

			CRC_reply = ((rx_buffer[(2*n)+5-1] << 8) | rx_buffer[(2*n)+4-1]);
			CRC_calc = CRC16(rx_buffer, PL+3); //Calc CRC for bytes 1-2n+3 of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				int16_t internalTemp = DATA[0];  //TinyBMS internal temperature
				printf("TinyBMS internal temperature: %d°C (0.1°C)\n", internalTemp);

				if((numData == 2) || (numData == 3)) {
					int16_t externalTemp1 = DATA[1];  //External Temp Sensor #1
					printf("External sensor 1 temperature: %d°C (0.1°C)\n", externalTemp1);
					//value of -32768 if not connected

					if(numData == 3) {
						int16_t externalTemp2 = DATA[2];  //External Temp Sensor #2
						printf("External sensor 2 temperature: %d°C (0.1°C)\n", externalTemp2);
						//value of -32768 if not connected
					}
				}
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x1B but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.19 Read Battery Pack Cell Voltages
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ReadBatteryPackCellVoltages
 *
 * @brief				-  Read TinyBMS Battery Pack Cell Voltages
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				- 			Response from BMS [OK]:
 * 						 	  Byte n*2+2    Byte n*2+3 		Byte n*2+4 	Byte n*2+5
 * 						 	  DATAn:LSB   	DATAn:MSB   	CRC:LSB 	CRC:MSB
 * 						 	       	 [UINT16]
 */
uint8_t TinyBMS_UART_ReadBatteryPackCellVoltages(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_UART_ReadBatteryPackCellVoltages\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[1000];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x1C;

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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x1C) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 1); //read byte 3 from OK reply

			//Up to 16 Cells in Series: (Registers: 0x00-0x15) Cell 1 - Cell 16
			uint8_t PL = rx_buffer[2];
			//Payload Length = PL = 2n bytes

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[3], PL+2); //read from byte 4 to 2n+5 from OK reply

			uint16_t cellVoltage[] = {0};
			uint32_t numCells = (PL / 2); //2 bytes per data reading
			uint32_t n = 1; //cell or data index, where 1 <= n <= 16

			for(uint32_t i = 0; i < numCells; i++) {
				//DATAn Cell Voltage (0.1mV resolution) LSB = Byte(2n+2), MSB = Byte(2n+3) (i.e. rx_buffer[2n+2:2n+3])
				cellVoltage[i] = ((rx_buffer[(2*n)+3-1] << 8) | (rx_buffer[(2*n)+2-1]));
				n++;
			}

			CRC_reply = ((rx_buffer[(2*n)+5-1] << 8) | rx_buffer[(2*n)+4-1]);
			CRC_calc = CRC16(rx_buffer, PL+3); //Calc CRC for bytes (PL+3 or Payload Value plus Bytes 1,2,3)) of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				//Print the Individual Cell Voltages of Battery Pack
				printf("************ TinyBMS Secondary Pack Cell Voltages************\n");
				for(uint16_t i = 0; i < numCells; i++) {
					printf("Cell%d Voltage: %u (0.1mV resolution)\n", i+1, cellVoltage[i]);
				}
				printf("----------------------------------------\n");
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x1C but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.20 Read TinyBMS Settings Values (min, max, default, current)
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ReadSettingsValues
 *
 * @brief				-  Read TinyBMS Settings values (min, max, default, current)
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 * @param[in]			-  option - (uint8_t) *NOTE*
 * @param[in]			-  rl - (uint8_t) *NOTE*
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
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
uint8_t TinyBMS_UART_ReadSettingsValues(UART_HandleTypeDef *huart2, uint8_t option, uint8_t rl) {
	printf("TinyBMS_UART_ReadSettingsValues\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[1000];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x1D;

	//Check input
	if((option != 0x01) && (option != 0x02) && (option != 0x03) && (option != 0x04)) {
		printf("Input 'option' invalid, must be: 0x01 - Min, 0x02 - Max, 0x03 - Default, or 0x04 - Current.\n");
		retval = CMD_FAILURE;
		return retval;
	}
	tx_buffer[2] = option;

	tx_buffer[3] = 0x00;

	//Check input
	if((rl < 1) || (rl > 100)) {
		printf("Input 'rl' (registers to read) out of acceptable range. Please enter a value between 1-100.\n");
		retval = CMD_FAILURE;
		return retval;
	}
	tx_buffer[4] = rl;


	CRC_request = CRC16(tx_buffer, 5);
	CRC_LSB = (CRC_request & 0xFF);
	CRC_MSB = ((CRC_request >> 8) & 0xFF);
	/*
	printf("CRC_request: 0x%04X\n", CRC_request);
	printf("CRC_LSB: 0x%02X\n", CRC_LSB);
	printf("CRC_MSB: 0x%02X\n", CRC_MSB);
	*/

	tx_buffer[5] = CRC_LSB;
	tx_buffer[6] = CRC_MSB;

	HAL_UART_Transmit_IT(huart2, (uint8_t *)tx_buffer, 7);

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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x1D) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 1); //read byte 3 from OK reply

			//Can read up to maximum of 100 (0x64) registers
			uint8_t PL = rx_buffer[2];
			//Payload Length = PL = 2n bytes

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[3], PL+2); //read from byte 4 to 2n+5 from OK reply

			uint16_t settingsValues[] = {0};
			uint32_t numSettings = (PL / 2); //2 bytes per data reading
			uint32_t n = 1; //settings or data index

			for(uint32_t i = 0; i < numSettings; i++) {
				settingsValues[i] = ((rx_buffer[(2*n)+3-1] << 8) | (rx_buffer[(2*n)+2-1]));
				n++;
			}

			CRC_reply = ((rx_buffer[(2*n)+5-1] << 8) | rx_buffer[(2*n)+4-1]);
			CRC_calc = CRC16(rx_buffer, PL+3); //Calc CRC for bytes (PL+3 or Payload Value plus Bytes 1,2,3)) of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				//Print the TinyBMS Settings Values (min, max, default, or current)
				printf("************ TinyBMS Settings Values ************\n");
				if(option == 0x01) {
					printf("0x01 - Minimum Settings\n");
				} else if(option == 0x02) {
					printf("0x02 - Maximum Settings\n");
				} else if(option == 0x03) {
					printf("0x03 - Default Settings\n");
				} else if(option == 0x04) {
					printf("0x04 - Current Settings\n");
				}
				for(uint16_t i = 0; i < numSettings; i++) {
					printf("Register %u: %u\n", i, settingsValues[i]);
				}
				printf("----------------------------------------\n");
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x1D but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.21 Read TinyBMS Version
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ReadVersion
 *
 * @brief				-  Read TinyBMS Version
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
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
uint8_t TinyBMS_UART_ReadVersion(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_UART_ReadVersion\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x1E;


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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x1E) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 1); //read byte 3 from OK reply

			//PL: (uint8_t)(uint8_t)(uint8_t)(uint16_t)
			uint8_t PL = rx_buffer[2];
			//Payload Length = PL = n bytes

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[3], PL+2); //read from byte 4 to  from OK reply

			uint8_t versions8[] = {0};
			uint16_t version16 = 0;
			uint32_t numVersions = PL; //1 bytes per data reading (except DATA4 which is 2 bytes)
			if(PL >= 5) {
				numVersions = numVersions - 1; //accounting for DATA4 being 2 bytes instead of 1
			}
			uint32_t n = 0; //version or data index

			//PL = 5 (max)
			for(uint32_t i = 0; i < numVersions; i++) {
				n++;
				//DATA1,2,3
				//n=1,2,3 or i=0,1,2
				if((i >= 0) && (i < 3)) {
					versions8[i] = rx_buffer[n+3-1];
				}
				//DATA4
				//n=4 or i=3
				if(i == 3) {
					version16 = ((rx_buffer[n+4-1] << 8) | rx_buffer[n+3-1]);
				}
			}

			//n=4
			CRC_reply = ((rx_buffer[n+6-1] << 8) | rx_buffer[n+5-1]);
			CRC_calc = CRC16(rx_buffer, PL+3); //Calc CRC for bytes (PL+3 or Payload Value plus Bytes 1,2,3)) of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				//Print the TinyBMS Versions (Hardware, Hardware Changes, Firmware Public, or Firmware Internal)
				printf("************ TinyBMS Versions ************\n");
				if(PL >= 1) {
					printf("Hardware Version: %u\n", versions8[0]);
				}
				if(PL >= 2) {
					printf("Hardware Changes Version: %u\n", versions8[1]);
				}
				if(PL >= 3) {
					printf("Firmware Public Version: %u\n", versions8[2]);
				}
				if(PL >= 5) {
					printf("Firmware Internal Version: %u\n", version16);
				}
				printf("----------------------------------------\n");
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x1E but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.22 Read TinyBMS Extended Version
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ReadVersionExtended
 *
 * @brief				-  Read TinyBMS Extended Version
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
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
uint8_t TinyBMS_UART_ReadVersionExtended(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_UART_ReadVersionExtended\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x1F;


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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x1F) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[2], 1); //read byte 3 from OK reply

			//PL: (uint8_t)(uint8_t)(uint8_t)(uint16_t)(uint8_t)(uint8_t)
			uint8_t PL = rx_buffer[2];
			//Payload Length = PL = n bytes

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[3], PL+2); //read from byte 4 to  from OK reply

			uint8_t versions8[] = {0};
			uint16_t version16 = 0;
			uint32_t numVersions = PL; //1 bytes per data reading (except DATA4 which is 2 bytes)
			if(PL >= 5) {
				numVersions = numVersions - 1; //accounting for DATA4 being 2 bytes instead of 1
			}
			uint32_t n = 0; //version or data index

			//PL = 7 (max)
			for(uint32_t i = 0; i < numVersions; i++) {
				n++;
				//DATA1,2,3
				//n=1,2,3 or i=0,1,2
				if((i >= 0) && (i < 3)) {
					versions8[i] = rx_buffer[n+3-1];
				}
				//DATA4
				//n=4 or i=3
				if(i == 3) {
					version16 = ((rx_buffer[n+4-1] << 8) | rx_buffer[n+3-1]);
				}
				//DATA5,6
				//n=5,6 or i=4,5
				if((i >= 4) && (i < 6)) {
					versions8[i-1] = rx_buffer[n+4-1];
				}
			}

			//n=6
			CRC_reply = ((rx_buffer[n+6-1] << 8) | rx_buffer[n+5-1]);
			CRC_calc = CRC16(rx_buffer, PL+3); //Calc CRC for bytes (PL+3 or Payload Value plus Bytes 1,2,3)) of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				//Print the TinyBMS Versions (Hardware, Hardware Changes, Firmware Public, Firmware Internal, Bootloader, or Register Map)
				printf("************ TinyBMS Extended Versions ************\n");
				if(PL >= 1) {
					printf("Hardware Version: %u\n", versions8[0]);
				}
				if(PL >= 2) {
					printf("Hardware Changes Version: %u\n", versions8[1]);
				}
				if(PL >= 3) {
					printf("Firmware Public Version: %u\n", versions8[2]);
				}
				if(PL >= 5) {
					printf("Firmware Internal Version: %u\n", version16);
				}
				if(PL >= 6) {
					printf("Bootloader Version: %u\n", versions8[3]);
				}
				if(PL >= 7) {
					printf("Register Map Version: %u\n", versions8[4]);
				}
				printf("----------------------------------------\n");
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x1F but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

//1.1.23 Read TinyBMS Calculated Speed, Distance Left and Estimated Time Left Values
/* ******************************************************************************
 * @fn					-  TinyBMS_UART_ReadCalcSpeedDistanceLeftEstTimeLeft
 *
 * @brief				-  Read TinyBMS Calculated Speed, Distance Left, Estimated Time Left
 *
 * @param[in]			-  huart2 - base address of UART2 Handle (UART_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
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
uint8_t TinyBMS_UART_ReadCalcSpeedDistanceLeftEstTimeLeft(UART_HandleTypeDef *huart2) {
	printf("TinyBMS_UART_ReadCalcSpeedDistanceLeftEstTimeLeft\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x20;

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
				retval = CMD_FAILURE;
			}

		//[OK]
		} else if(rx_buffer[1] == 0x20) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(huart2, (uint8_t *)&rx_buffer[3], 14); //read from byte 3 to 16 from OK reply

			//DATA1 - Speed (km/h)
			uint32_t DATA1 = ((rx_buffer[5] << 24) | (rx_buffer[4] << 16) | (rx_buffer[3] << 8) | (rx_buffer[2]));
			float speed = DATA1;

			//DATA2 - Distance Left Until Empty Battery (km)
			uint32_t distanceLeftUntilEmpty = ((rx_buffer[9] << 24) | (rx_buffer[8] << 16) | (rx_buffer[7] << 8) | (rx_buffer[6]));

			//DATA3 - Estimated Time Left Until Empty Battery (seconds)
			uint32_t timeLeftUntilEmpty = ((rx_buffer[13] << 24) | (rx_buffer[12] << 16) | (rx_buffer[11] << 8) | (rx_buffer[10]));

			CRC_reply = ((rx_buffer[15] << 8) | rx_buffer[14]);
			CRC_calc = CRC16(rx_buffer, 14);

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				//Print the TinyBMS Versions (Hardware, Hardware Changes, Firmware Public, Firmware Internal, Bootloader, or Register Map)
				printf("************ TinyBMS Calculated Speed, Distance & Estimated Time Left Until Empty ************\n");
				printf("Speed: %f (km/h)\n", speed);
				printf("Distance left until empty: %lu (km)\n", distanceLeftUntilEmpty);
				printf("Estimated time left until empty: %lu (s)\n", timeLeftUntilEmpty);
				printf("----------------------------------------\n");
				retval = CMD_SUCCESS;

			} else {
				printf("CRC fail in BMS OK\n");
				retval = CMD_FAILURE;
			}

		} else {
			printf("Error: Byte 2 should be 0x00 or 0x20 but was 0x%02X\n", rx_buffer[1]);
			retval = CMD_FAILURE;
		}

	} else {
		printf("Error: Byte 1 should be 0xAA but was 0x%02X\n", rx_buffer[0]);
		retval = CMD_FAILURE;
	}

	return retval;
}

/* ********************************
 *  TinyBMS CAN Communication API
 * ******************************** */

/* Note1: CAN bitrate 500 kbit/s (not allowed to be changed by user). Default node ID after firmware update is 0x01.
 *        When multi-slave CAN bus topology is used, node ID can be assigned with 19 TinyBMS CAN command.
 *        Automatic node ID assignment is not available. */

/* Note2: TinyBMS CAN-UART converter works aand CAN bus communication is available only when BMS device is in
 * 		  an active state (charging, discharging, or Ignition enabled).
 */

/* Note3: There is no CRC with CAN API. */


//2.1.1 Reset TinyBMS, Clear Events and Statistics
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_ResetClearEventsStatistics
 *
 * @brief				-  Reset BMS, Clear Events, or Clear Statistics depending on option
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 * @param[in]			-  option (uint8_t) *NOTE*
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  Options: 0x01 - Clear Events , 0x02 - Clear Statistics , 0x05 - Reset BMS
 *
 */
uint8_t TinyBMS_CAN_ResetClearEventsStatistics(CAN_HandleTypeDef *hcan1, uint8_t option) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}

//2.1.2 Read TinyBMS Registers Block
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_ReadRegBlock
 *
 * @brief				-  Read from a register block
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 * @param[in]			-  rl - Number (length) of registers to read (uint8_t)
 * @param[in]			-  addr - First register's block address (LSB first) (uint16_t)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  A memory block is a group of one or more contiguous bytes of memory allocated
 * 						   by malloc(size_t size).
 */
uint8_t TinyBMS_CAN_ReadRegBlock(CAN_HandleTypeDef *hcan1, uint8_t rl, uint16_t addr) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}

//2.1.3 Write TinyBMS Registers Block
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_WriteRegBlock
 *
 * @brief				-  Write to a register block
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 * @param[in]			-  rl - registers to write. Max 100 (0x64) registers (uint8_t)
 * @param[in]			-  addr - First register's block address (LSB first) (uint16_t)
 * @param[in]			-  data - a pointer to the first address of data structure containing the register
 * 							values (LSB first) (uint16_t*)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  A memory block is a group of one or more contiguous bytes of memory allocated by malloc(size_t size).
 */
uint8_t TinyBMS_CAN_WriteRegBlock(CAN_HandleTypeDef *hcan1, uint8_t rl, uint16_t addr, uint16_t data[]) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}

//2.1.4 Read TinyBMS Newest Events
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_ReadNewestEvents
 *
 * @brief				-  Read BMS newest events with timestamps and event ID's
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  PL: Payload length in bytes [UINT8]. BTSP: BMS timestamp in seconds [UINT32].
 * 						   TSP: Event timestamp in seconds [UINT24]. EVENT: BMS Event ID [UINT8].
 */
uint8_t TinyBMS_CAN_ReadNewestEvents(CAN_HandleTypeDef *hcan1) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}

//2.1.5 Read TinyBMS All Events
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_ReadAllEvents
 *
 * @brief				-  Read BMS all events with timestamps and event ID's
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  PL: Payload length in bytes [UINT8]. BTSP: BMS timestamp in seconds [UINT32].
 * 						   TSP: Event timestamp in seconds [UINT24]. EVENT: BMS Event ID [UINT8].
 */
uint8_t TinyBMS_CAN_ReadAllEvents(CAN_HandleTypeDef *hcan1) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}

//2.1.6 Read Battery Pack Voltage
//Reg:36
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_ReadBatteryPackVoltage
 *
 * @brief				-  Read Battery Pack Voltage - 1V Resolution
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  			Response from BMS [OK]:
 * 						 	  Byte3      Byte4  Byte5  	Byte6 		Byte7 	Byte8
 * 						 	  DATA:LSB   DATA   DATA	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	               [FLOAT]
 */
uint8_t TinyBMS_CAN_ReadBatteryPackVoltage(CAN_HandleTypeDef *hcan1) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}

//2.1.7 Read Battery Pack Current
//Reg:38
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_ReadBatteryPackCurrent
 *
 * @brief				-  Read Battery Pack Current - 1A Resolution
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  			Response from BMS [OK]:
 * 						 	  Byte3      Byte4  Byte5  	Byte6 		Byte7 	Byte8
 * 						 	  DATA:LSB   DATA   DATA	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	               [FLOAT]
 */
uint8_t TinyBMS_CAN_ReadBatteryPackCurrent(CAN_HandleTypeDef *hcan1) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}

//2.1.8 Read Battery Pack Max Cell Voltage
//Reg:41
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_ReadBatteryPackMaxCellVoltage
 *
 * @brief				-  Read Battery Pack Maximum Cell Voltage - 1mV Resolution
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  			Response from BMS [OK]:
 * 						 	  Byte3        	Byte4 		Byte5 	Byte6
 * 						 	  DATA:LSB   	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        [UINT16]
 */
uint8_t TinyBMS_CAN_ReadBatteryPackMaxCellVoltage(CAN_HandleTypeDef *hcan1) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}

//2.1.9 Read Battery Pack Min Cell Voltage
//Reg:40
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_ReadBatteryPackMinCellVoltage
 *
 * @brief				-  Read Battery Pack Minimum Cell Voltage - 1mV Resolution
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				- 			Response from BMS [OK]:
 * 						 	  Byte3        	Byte4 		Byte5 	Byte6
 * 						 	  DATA:LSB   	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        [UINT16]
 */
uint8_t TinyBMS_CAN_ReadBatteryPackMinCellVoltage(CAN_HandleTypeDef *hcan1) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}

//2.1.10 Read TinyBMS Online Status
//Reg:50
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_ReadOnlineStatus
 *
 * @brief				-  Read TinyBMS Online Status
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-			Response from BMS [OK]:
 * 						 	  Byte3        	Byte4 		Byte5 	Byte6
 * 						 	  DATA:LSB   	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        [UINT16]
 * 						   0x91 - Charging [INFO], 0x92 - Fully Charged [INFO]
 * 						   0x93 - Discharging [INFO], 0x94 - Regeneration [INFO]
 * 						   0x97 - Idle [INFO], 0x9B - Fault [ERROR]
 */
uint8_t TinyBMS_CAN_ReadOnlineStatus(CAN_HandleTypeDef *hcan1) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}

//2.1.11 Read TinyBMS Lifetime Counter
//Reg:32
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_ReadLifetimeCounter
 *
 * @brief				-  Read TinyBMS Lifetime Counter - 1s Resolution
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 *
 * @return				-  0xAA Success, 0xFF CRC Failure in BMS OK, (uint8_t) error code
 *
 * @note				-  				Response from BMS [OK]:
 * 						 	  Byte3      Byte4 	Byte5 	Byte6		Byte5 	Byte6
 * 						 	  DATA:LSB   DATA	DATA	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        	  [UINT32]
 */
uint8_t TinyBMS_CAN_ReadLifetimeCounter(CAN_HandleTypeDef *hcan1) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}

//2.1.12 Read TinyBMS Estimated SOC Value
//Reg:46
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_ReadEstimatedSOCValue
 *
 * @brief				-  Read TinyBMS Estimated SOC (State of Charge) Value - 0.000 001 % Resolution
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  				Response from BMS [OK]:
 * 						 	  Byte3      Byte4 	Byte5 	Byte6		Byte5 	Byte6
 * 						 	  DATA:LSB   DATA	DATA	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        	  [UINT32]
 */
uint8_t TinyBMS_CAN_ReadEstimatedSOCValue(CAN_HandleTypeDef *hcan1) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}

//2.1.13 Read TinyBMS Device Temperatures
//Reg:48,42,43
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_ReadDeviceTemperatures
 *
 * @brief				-  Read TinyBMS Device Temperatures - 0.1°C Resolution
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  				Response from BMS [OK]:
 * 						 	Byte4      Byte5 		Byte6 		Byte7		Byte 8	 	Byte 9 		Byte10 	Byte11
 * 						 	DATA1:LSB  DATA1:MSB	DATA2:LSB	DATA2:MSB	DATA3:LSB	DATA3:MSB   CRC:LSB CRC:MSB
 * 						 	      [INT16]				   [INT16] 				  [INT16]
 * 						  	(Reg 48) DATA1 - TinyBMS Internal Temperature
 * 						  	(Reg 42) DATA2 - External Temp Sensor #1 (value of -327689 if NC)
 * 						  	(Reg 43) DATA3 - External Temp Sensor #2 (value of -327689 if NC)
 */
uint8_t TinyBMS_CAN_ReadDeviceTemperatures(CAN_HandleTypeDef *hcan1) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}

//2.1.14 Read Battery Pack Cell Voltages
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_ReadBatteryPackCellVoltages
 *
 * @brief				-  Read TinyBMS Battery Pack Cell Voltages
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				- 			Response from BMS [OK]:
 * 						 	  Byte n*2+2    Byte n*2+3 		Byte n*2+4 	Byte n*2+5
 * 						 	  DATAn:LSB   	DATAn:MSB   	CRC:LSB 	CRC:MSB
 * 						 	       	 [UINT16]
 */
uint8_t TinyBMS_CAN_ReadBatteryPackCellVoltages(CAN_HandleTypeDef *hcan1) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}

//2.1.15 Read TinyBMS Settings Values (min, max, default, current)
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_ReadSettingsValues
 *
 * @brief				-  Read TinyBMS Settings values (min, max, default, current)
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 * @param[in]			-  option - (uint8_t) *NOTE*
 * @param[in]			-  rl - (uint8_t) *NOTE*
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
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
uint8_t TinyBMS_CAN_ReadSettingsValues(CAN_HandleTypeDef *hcan1, uint8_t option, uint8_t rl) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}

//2.1.16 Read TinyBMS Version
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_ReadVersion
 *
 * @brief				-  Read TinyBMS Version
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
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
uint8_t TinyBMS_CAN_ReadVersion(CAN_HandleTypeDef *hcan1) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}

//2.1.17 Read TinyBMS Calculated Speed, Distance Left and Estimated Time Left Values
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_ReadCalcSpeedDistanceLeftEstTimeLeft
 *
 * @brief				-  Read TinyBMS Calculated Speed, Distance Left, Estimated Time Left
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
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
uint8_t TinyBMS_CAN_ReadCalcSpeedDistanceLeftEstTimeLeft(CAN_HandleTypeDef *hcan1) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}

//2.1.18 Read CAN Node ID
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_ReadNodeID
 *
 * @brief				-  Read TinyBMS CAN Node ID
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  Default after firmware update is 0x01
 */
uint8_t TinyBMS_CAN_ReadNodeID(CAN_HandleTypeDef *hcan1) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}

//2.1.19 Write CAN Node ID
/* ******************************************************************************
 * @fn					-  TinyBMS_CAN_WriteNodeID
 *
 * @brief				-  Write TinyBMS CAN Node ID
 *
 * @param[in]			-  hcan1 - base address of CAN1 Handle (CAN_HandleTypeDef)
 *
 * @return				-  CMD_SUCCESS, CMD_FAILURE, (uint8_t) error code
 *
 * @note				-  Default after firmware update is 0x01
 */
uint8_t TinyBMS_CAN_WriteNodeID(CAN_HandleTypeDef *hcan1) {
	uint8_t retval = CMD_FAILURE;

	return retval;
}


/********************** CRC Calculation **********************/
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
