/***********************************************
* @file TinyBMS.c
* @brief TinyBMS Library - UART and CAN API
* @author Oliver Moore
* @version 1.3
* @date 02-17-2022
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
 *  2.1.1 		[x] 		[]		TinyBMS_CAN_ResetClearEventsStatistics
 *  2.1.2 		[x] 		[]		TinyBMS_CAN_ReadRegBlock
 *  2.1.3 		[x] 		[]		TinyBMS_CAN_WriteRegBlock
 *  2.1.4 		[x] 		[]		TinyBMS_CAN_ReadNewestEvents
 *  2.1.5 		[x] 		[]		TinyBMS_CAN_ReadAllEvents
 *  2.1.6 		[x] 		[]		TinyBMS_CAN_ReadBatteryPackVoltage
 *  2.1.7 		[x] 		[]		TinyBMS_CAN_ReadBatteryPackCurrent
 *  2.1.8 		[x] 		[]		TinyBMS_CAN_ReadBatteryPackMaxCellVoltage
 *  2.1.9 		[x] 		[]		TinyBMS_CAN_ReadBatteryPackMinCellVoltage
 *  2.1.10 		[x] 		[]		TinyBMS_CAN_ReadOnlineStatus
 *  2.1.11 		[x] 		[]		TinyBMS_CAN_ReadLifetimeCounter
 *  2.1.12 		[x] 		[]		TinyBMS_CAN_ReadEstimatedSOCValue
 *  2.1.13 		[x] 		[]		TinyBMS_CAN_ReadDeviceTemperatures
 *  2.1.14 		[x] 		[]		TinyBMS_CAN_ReadBatteryPackCellVoltages
 *  2.1.15 		[x] 		[]		TinyBMS_CAN_ReadSettingsValues
 *  2.1.16 		[x] 		[]		TinyBMS_CAN_ReadVersion
 *  2.1.17 		[x] 		[]		TinyBMS_CAN_ReadCalcSpeedDistanceLeftEstTimeLeft
 *  2.1.18 		[x] 		[]		TinyBMS_CAN_ReadNodeID
 *  2.1.19 		[x] 		[]		TinyBMS_CAN_WriteNodeID
 *
 */

/**************** Header Files ****************/
#include "TinyBMS.h"

/*************** Static Function Prototypes **************/
/* CRC Calculation Function Prototype (only used with UART) */
static uint16_t CRC16(const uint8_t* data, uint16_t length);

/***************** External Handles ******************/
extern UART_HandleTypeDef huart2;
extern CAN_HandleTypeDef hcan1;

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
uint8_t TinyBMS_UART_ACK(UART_HandleTypeDef *huart) {
	printf("TinyBMS_UART_ACK\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t rx_buffer[50];
	uint8_t cmd = 0;
	uint16_t CRC_calc = 0, CRC_reply = 0;

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check ACK/NACK

	if(rx_buffer[0] == 0xAA) {

		//[NACK]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [NACK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from NACK reply

			cmd = rx_buffer[2];
			printf("cmd: 0x%02X\n", cmd);

			uint8_t error = rx_buffer[3];

			if(error == CMD_ERROR) {
				printf("CMD ERROR\n");
			} else if(error == CRC_ERROR) {
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
		} else if(rx_buffer[1] == UART_TBMS_ACK) {
			printf("Response from BMS [ACK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 3); //read bytes 3-5 from ACK reply

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
uint8_t TinyBMS_UART_ReadRegBlock(UART_HandleTypeDef *huart, uint8_t rl, uint16_t addr) {
	printf("TinyBMS_UART_ReadRegBlock\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[1000];
	uint8_t ADDR_LSB = 0, ADDR_MSB = 0, CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_READ_REG_BLOCK;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, 7);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_READ_REG_BLOCK) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 1); //read byte 3 from OK reply

			//Should be equal to (RL * 2) because 2 bytes per register
			uint8_t PL = rx_buffer[2];

			uint8_t bit7 = ((PL >> 7) & 1); //extract bit 7
			uint8_t bits05 = (PL & 63); //extract bits 0-5

			//if bit7 == 1
			if(bit7) {
				printf("Current packet ID: 0x%02X\n", bits05);
			//else bit7 == 0
			} else {
				printf("Payload size in bytes (last packet): 0x%02X\n", bits05);
			}

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[3], PL+2); //read from byte 4 to 2*n+5 from OK reply

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
uint8_t TinyBMS_UART_ReadRegIndividual(UART_HandleTypeDef *huart, uint8_t pl, uint16_t addr[]) {
	printf("TinyBMS_UART_ReadRegIndividual\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[1000], rx_buffer[1000];
	uint8_t ADDR_LSB = 0, ADDR_MSB = 0, CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	uint8_t pl_request = pl;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_READ_INDIVIDUAL_REGS;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, pl_request+5); //Bytes 1:2n+5 or pl+5

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_READ_INDIVIDUAL_REGS) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 1); //read byte 3 from OK reply

			//equal to (RL * 2) because 2 bytes per register
			uint8_t pl_response = rx_buffer[2];

			uint8_t bit7 = ((pl_response >> 7) & 1); //extract bit 7
			uint8_t bits05 = (pl_response & 63); //extract bits 0-5

			//if bit7 == 1
			if(bit7) {
				printf("Current packet ID: 0x%02X\n", bits05);
			//else bit7 == 0
			} else {
				printf("Payload size in bytes (last packet): 0x%02X\n", bits05);
			}

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[3], pl_response+2); //read from byte 4 to 4*n+5 from OK reply

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
uint8_t TinyBMS_UART_WriteRegBlock(UART_HandleTypeDef *huart, uint8_t pl, uint16_t addr, uint16_t data[]) {
	printf("TinyBMS_UART_WriteRegBlock\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[1000], rx_buffer[50];
	uint8_t ADDR_LSB = 0, ADDR_MSB = 0, DATA_LSB = 0, DATA_MSB = 0, CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_WRITE_REG_BLOCK;

	tx_buffer[2] = pl;

	uint8_t bit7 = ((pl >> 7) & 1); //extract bit 7
	uint8_t bits05 = (pl & 63); //extract bits 0-5

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, pl+5);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == ACK) {
			printf("Response from BMS [ACK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 3); //read byte 3-5 from ACK reply

			if(rx_buffer[2] == UART_TBMS_WRITE_REG_BLOCK) {
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
uint8_t TinyBMS_UART_WriteRegIndividual(UART_HandleTypeDef *huart, uint8_t pl, uint16_t addr[], uint16_t data[]) {
	printf("TinyBMS_UART_WriteRegIndividual\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[1000], rx_buffer[1000];
	uint8_t ADDR_LSB = 0, ADDR_MSB = 0, DATA_LSB = 0, DATA_MSB = 0, CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_WRITE_INDIVIDUAL_REGS;

	tx_buffer[2] = pl;

	uint8_t bit7 = ((pl >> 7) & 1); //extract bit 7
	uint8_t bits05 = (pl & 63); //extract bits 0-5

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, pl+5);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == ACK) {
			printf("Response from BMS [ACK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 3); //read bytes 3-5 from ACK reply

			if(rx_buffer[2] == UART_TBMS_WRITE_INDIVIDUAL_REGS) {
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
uint8_t TinyBMS_UART_ReadRegBlockMODBUS(UART_HandleTypeDef *huart, uint16_t addr, uint8_t rl) {
	printf("TinyBMS_UART_ReadRegBlockMODBUS\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[1000];
	uint8_t ADDR_LSB = 0, ADDR_MSB = 0, CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_READ_REG_BLOCK_MODBUS;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, 8);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_READ_REG_BLOCK_MODBUS) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 1); //read byte 3 from OK reply

			uint8_t PL = rx_buffer[2];

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[3], PL+2); //read from byte 4 to 2*n+5 from OK reply

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
uint8_t TinyBMS_UART_WriteRegBlockMODBUS(UART_HandleTypeDef *huart, uint16_t addr, uint8_t rl, uint8_t pl, uint16_t data[]) {
	printf("TinyBMS_UART_WriteRegBlockMODBUS\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[1000], rx_buffer[50];
	uint8_t ADDR_LSB = 0, ADDR_MSB = 0, DATA_LSB = 0, DATA_MSB = 0, CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_WRITE_REG_BLOCK_MODBUS;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, pl+9);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_WRITE_REG_BLOCK_MODBUS) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 6); //read bytes 3-8 from OK reply

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
uint8_t TinyBMS_UART_ResetClearEventsStatistics(UART_HandleTypeDef *huart, uint8_t option) {
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
	tx_buffer[1] = UART_TBMS_RESET_CLEAR_EVENTS_STATS;
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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, 5);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check ACK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == ACK) {
			printf("Response from BMS [ACK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 3); //read bytes 3-5 from ACK reply
			if(rx_buffer[2] == UART_TBMS_RESET_CLEAR_EVENTS_STATS) {
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
uint8_t TinyBMS_UART_ReadNewestEvents(UART_HandleTypeDef *huart) {
	printf("TinyBMS_UART_ReadNewestEvents\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[1000];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_READ_NEWEST_EVENTS;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_READ_NEWEST_EVENTS) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 1); //read byte 3 from OK reply

			//Total Events: (0x01-0x90) or 144 events
			uint8_t PL = rx_buffer[2];
			//Payload Length = PL = 4n+4 bytes

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[3], PL+2); //read from byte 4 to 4n+9 from OK reply

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
uint8_t TinyBMS_UART_ReadAllEvents(UART_HandleTypeDef *huart) {
	printf("TinyBMS_UART_ReadAllEvents\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[1000];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_READ_ALL_EVENTS;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_READ_ALL_EVENTS) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 1); //read byte 3 from OK reply

			//Total Events: (0x01-0x90) or 144 events
			uint8_t PL = rx_buffer[2];
			//Payload Length = PL = 4n+4 bytes

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[3], PL+2); //read from byte 4 to 4n+9 from OK reply

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
uint8_t TinyBMS_UART_ReadBatteryPackVoltage(UART_HandleTypeDef *huart) {
	printf("TinyBMS_UART_ReadBatteryPackVoltage\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_READ_PACK_VOLTAGE;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_READ_PACK_VOLTAGE) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 6); //read bytes 3-8 from OK reply

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
uint8_t TinyBMS_UART_ReadBatteryPackCurrent(UART_HandleTypeDef *huart) {
	printf("TinyBMS_UART_ReadBatteryPackCurrent\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_READ_PACK_CURRENT;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_READ_PACK_CURRENT) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 6); //read bytes 3-8 from OK reply

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
uint8_t TinyBMS_UART_ReadBatteryPackMaxCellVoltage(UART_HandleTypeDef *huart) {
	printf("TinyBMS_UART_ReadBatteryPackMaxCellVoltage\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_READ_MAX_CELL_VOLTAGE;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_READ_MAX_CELL_VOLTAGE) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from OK reply

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
uint8_t TinyBMS_UART_ReadBatteryPackMinCellVoltage(UART_HandleTypeDef *huart) {
	printf("TinyBMS_UART_ReadBatteryPackMinCellVoltage\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_READ_MIN_CELL_VOLTAGE;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_READ_MIN_CELL_VOLTAGE) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from OK reply

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
uint8_t TinyBMS_UART_ReadOnlineStatus(UART_HandleTypeDef *huart) {
	printf("TinyBMS_UART_ReadOnlineStatus\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_READ_ONLINE_STATUS;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_READ_ONLINE_STATUS) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from OK reply

			uint16_t onlineStatus = ((rx_buffer[3] << 8) | (rx_buffer[2]));

			CRC_reply = ((rx_buffer[5] << 8) | rx_buffer[4]);
			CRC_calc = CRC16(rx_buffer, 4); //Calc CRC for bytes 1-4 of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				printf("************ TinyBMS Online Status: ************\n");
				switch(onlineStatus) {
					case TBMS_STATUS_CHARGING:
						printf("0x91 - Charging [INFO]\n");
						break;
					case TBMS_STATUS_FULLYCHARGED:
						printf("0x92 - Fully Charged [INFO]\n");
						break;
					case TBMS_STATUS_DISCHARGING:
						printf("0x93 - Discharging [INFO]\n");
						break;
					case TBMS_STATUS_REGENERATION:
						printf("0x96 - Regeneration [INFO]\n");
						break;
					case TBMS_STATUS_IDLE:
						printf("0x97 - Idle [INFO]\n");
						break;
					case TBMS_STATUS_FAULT:
						printf("0x9B - Fault [ERROR]\n");
						break;
					default:
						printf("Invalid onlineStatus\n");
						retval = CMD_FAILURE;
						return retval;
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
uint8_t TinyBMS_UART_ReadLifetimeCounter(UART_HandleTypeDef *huart) {
	printf("TinyBMS_UART_ReadLifetimeCounter\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_READ_LIFETIME_COUNTER;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_READ_LIFETIME_COUNTER) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 6); //read bytes 3-8 from OK reply

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
uint8_t TinyBMS_UART_ReadEstimatedSOCValue(UART_HandleTypeDef *huart) {
	printf("TinyBMS_UART_ReadEstimatedSOCValue\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_READ_EST_SOC;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_READ_EST_SOC) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 6); //read bytes 3-8 from OK reply

			uint32_t estSOC = ((rx_buffer[5] << 24) | (rx_buffer[4] << 16) | (rx_buffer[3] << 8) | (rx_buffer[2]));

			CRC_reply = ((rx_buffer[7] << 8) | rx_buffer[6]);
			CRC_calc = CRC16(rx_buffer, 6); //Calc CRC for bytes 1-6 of OK response

			if(CRC_calc == CRC_reply) {
				printf("CRC pass\n");

				printf("Estimated State of Charge (SOC): %lu (0.000 001 %% resolution)\n", estSOC);
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
uint8_t TinyBMS_UART_ReadDeviceTemperatures(UART_HandleTypeDef *huart) {
	printf("TinyBMS_UART_ReadDeviceTemperatures\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_READ_TEMPS;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_READ_TEMPS) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 1); //read byte 3 from OK reply

			uint8_t PL = rx_buffer[2]; //payload length in bytes
			printf("Payload Length: 0x%02X\n", PL);
			//Payload Length = PL = 2n bytes -> where n = 1,2,3 (DATA1,DATA2,DATA3)

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[3], PL+2); //read bytes 4-2n+5 from OK reply

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
uint8_t TinyBMS_UART_ReadBatteryPackCellVoltages(UART_HandleTypeDef *huart) {
	printf("TinyBMS_UART_ReadBatteryPackCellVoltages\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[1000];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_READ_CELL_VOLTAGES;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_READ_CELL_VOLTAGES) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 1); //read byte 3 from OK reply

			//Up to 16 Cells in Series: (Registers: 0x00-0x15) Cell 1 - Cell 16
			uint8_t PL = rx_buffer[2];
			//Payload Length = PL = 2n bytes

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[3], PL+2); //read from byte 4 to 2n+5 from OK reply

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
uint8_t TinyBMS_UART_ReadSettingsValues(UART_HandleTypeDef *huart, uint8_t option, uint8_t rl) {
	printf("TinyBMS_UART_ReadSettingsValues\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[1000];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_READ_SETTINGS_VALUES;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, 7);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_READ_SETTINGS_VALUES) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 1); //read byte 3 from OK reply

			//Can read up to maximum of 100 (0x64) registers
			uint8_t PL = rx_buffer[2];
			//Payload Length = PL = 2n bytes

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[3], PL+2); //read from byte 4 to 2n+5 from OK reply

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
uint8_t TinyBMS_UART_ReadVersion(UART_HandleTypeDef *huart) {
	printf("TinyBMS_UART_ReadVersion\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_READ_VERSION;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_READ_VERSION) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 1); //read byte 3 from OK reply

			//PL: (uint8_t)(uint8_t)(uint8_t)(uint16_t)
			uint8_t PL = rx_buffer[2];
			//Payload Length = PL = n bytes

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[3], PL+2); //read from byte 4 to  from OK reply

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
				switch(PL) {
				case 1:
					printf("Hardware Version: %u\n", versions8[0]);
					break;
				case 2:
					printf("Hardware Version: %u\n", versions8[0]);
					printf("Hardware Changes Version: %u\n", versions8[1]);
					break;
				case 3:
					printf("Hardware Version: %u\n", versions8[0]);
					printf("Hardware Changes Version: %u\n", versions8[1]);
					printf("Firmware Public Version: %u\n", versions8[2]);
					break;
				case 5:
					printf("Hardware Version: %u\n", versions8[0]);
					printf("Hardware Changes Version: %u\n", versions8[1]);
					printf("Firmware Public Version: %u\n", versions8[2]);
					printf("Firmware Internal Version: %u\n", version16);
					break;
				default:
					printf("Invalid Payload Value\n");
					retval = CMD_FAILURE;
					return retval;
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
uint8_t TinyBMS_UART_ReadVersionExtended(UART_HandleTypeDef *huart) {
	printf("TinyBMS_UART_ReadVersionExtended\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_READ_VERSION_EXTENDED;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_READ_VERSION_EXTENDED) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 1); //read byte 3 from OK reply

			//PL: (uint8_t)(uint8_t)(uint8_t)(uint16_t)(uint8_t)(uint8_t)
			uint8_t PL = rx_buffer[2];
			//Payload Length = PL = n bytes

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[3], PL+2); //read from byte 4 to  from OK reply

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

				switch(PL) {
				case 1:
					printf("Hardware Version: %u\n", versions8[0]);
					break;
				case 2:
					printf("Hardware Version: %u\n", versions8[0]);
					printf("Hardware Changes Version: %u\n", versions8[1]);
					break;
				case 3:
					printf("Hardware Version: %u\n", versions8[0]);
					printf("Hardware Changes Version: %u\n", versions8[1]);
					printf("Firmware Public Version: %u\n", versions8[2]);
					break;
				case 5:
					printf("Hardware Version: %u\n", versions8[0]);
					printf("Hardware Changes Version: %u\n", versions8[1]);
					printf("Firmware Public Version: %u\n", versions8[2]);
					printf("Firmware Internal Version: %u\n", version16);
					break;
				case 6:
					printf("Hardware Version: %u\n", versions8[0]);
					printf("Hardware Changes Version: %u\n", versions8[1]);
					printf("Firmware Public Version: %u\n", versions8[2]);
					printf("Firmware Internal Version: %u\n", version16);
					printf("Bootloader Version: %u\n", versions8[3]);
					break;
				case 7:
					printf("Hardware Version: %u\n", versions8[0]);
					printf("Hardware Changes Version: %u\n", versions8[1]);
					printf("Firmware Public Version: %u\n", versions8[2]);
					printf("Firmware Internal Version: %u\n", version16);
					printf("Bootloader Version: %u\n", versions8[3]);
					printf("Register Map Version: %u\n", versions8[4]);
					break;
				default:
					printf("Invalid Payload Value\n");
					retval = CMD_FAILURE;
					return retval;
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
uint8_t TinyBMS_UART_ReadCalcSpeedDistanceLeftEstTimeLeft(UART_HandleTypeDef *huart) {
	printf("TinyBMS_UART_ReadCalcSpeedDistanceLeftEstTimeLeft\n");
	uint8_t retval = CMD_FAILURE;

	uint8_t tx_buffer[50], rx_buffer[50];
	uint8_t CRC_LSB = 0, CRC_MSB = 0;
	uint16_t CRC_request = 0, CRC_calc = 0, CRC_reply = 0;

	/* Request to BMS */
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = UART_TBMS_READ_SPEED_DISTANCETIME_LEFT;

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

	HAL_UART_Transmit_IT(&huart2, (uint8_t*)tx_buffer, 4);

	/* Response from BMS */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 2); //read bytes 1-2 to check OK/ERROR

	if(rx_buffer[0] == 0xAA) {

		//[ERROR]
		if(rx_buffer[1] == NACK) {
			printf("Response from BMS [ERROR]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[2], 4); //read bytes 3-6 from ERROR reply

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
		} else if(rx_buffer[1] == UART_TBMS_READ_SPEED_DISTANCETIME_LEFT) {
			printf("Response from BMS [OK]\n");

			HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[3], 14); //read from byte 3 to 16 from OK reply

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
uint8_t TinyBMS_CAN_ResetClearEventsStatistics(CAN_HandleTypeDef *hcan, uint8_t option) {
	printf("TinyBMS_CAN_ResetClearEventsStatistics\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8];

	/* Request to BMS */
	switch(option) {
	case 0x01:
		printf("0x01 Clear Events\n");
		break;
	case 0x02:
		printf("0x02 Clear Statistics\n");
		break;
	case 0x05:
		printf("0x05 Reset BMS\n");
		break;
	default:
		printf("Invalid option\n");
		retval = CMD_FAILURE;
		return retval;
	}

	uint8_t tx_msg[8] = {0x02, option, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	CAN1_Tx(tx_msg);

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_RESET_CLEAR_EVENTS_STATS)) {
			printf("Response from BMS [OK]\n");
			printf("CAN_TBMS_RESET_CLEAR_EVENTS_STATS | CMD: 0x%02X\n", rx_msg[1]);

		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
uint8_t TinyBMS_CAN_ReadRegBlock(CAN_HandleTypeDef *hcan, uint8_t rl, uint16_t addr) {
	printf("TinyBMS_CAN_ReadRegBlock\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8];
	uint8_t addr_MSB = 0, addr_LSB = 0, msg_count = 1, pl = 0;
	uint16_t data = 0;

	/* Request to BMS */
	//Check if number of registers to write is within bounds
	if((rl <= 0) || (rl > 0x7F)) {
		retval = CMD_FAILURE;
		return retval;
	}

	//Starting address of Register Block
	addr_MSB = ((addr >> 8) & 0xFF);
	addr_LSB = (addr & 0xFF);

	uint8_t tx_msg[8] = {CAN_TBMS_READ_REG_BLOCK, addr_MSB, addr_LSB, 0x00, rl, 0x00, 0x00, 0x00};
	CAN1_Tx(tx_msg);

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_READ_REG_BLOCK)) {
			if(msg_count == 1) {
				printf("Response from BMS [OK]\n");
				printf("CAN_TBMS_READ_REG_BLOCK | CMD: 0x%02X\n", rx_msg[1]);
			}
			pl = rx_msg[2];

			//If DATAn is 2 bytes in length and Byte 6 counter is correct
			//Message counter range: 1 to n vs Byte 6: 0 to n-1
			if((pl == 2) && (rx_msg[5] == (msg_count-1))) {
				data = ((rx_msg[3] << 8) | rx_msg[4]);

				// MSG1 - Addr: 0xABCD - Data: 0x1234
				// MSG2 - Addr: 0xABDD - Data: 0x5678
				printf("MSG%u - ", msg_count);
				printf("Addr: 0x%04X - ", (addr+(sizeof(addr)*(msg_count-1))) );
				printf("Data: 0x%04X\n", data);
				msg_count++;
			} else {
				printf("Data Corruption\n");
				retval = CMD_FAILURE;
				return retval;
			}

		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
uint8_t TinyBMS_CAN_WriteRegBlock(CAN_HandleTypeDef *hcan, uint8_t rl, uint16_t addr, uint16_t data[]) {
	printf("TinyBMS_CAN_WriteRegBlock\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8], tx_msg[8];
	uint8_t addr_MSB = 0, addr_LSB = 0, data_MSB = 0, data_LSB = 0, msg_count = 1;

	/* Request to BMS */
	//Check if register block start address is within bounds
	if((addr < 0x12C) || (addr > 0x18F)) {
		retval = CMD_FAILURE;
		return retval;
	}

	//Check if number of registers to write is within bounds
	if((rl <= 0) || (rl > 0x64)) {
		retval = CMD_FAILURE;
		return retval;
	}

	//Starting address of Register Block
	addr_MSB = ((addr >> 8) & 0xFF);
	addr_LSB = (addr & 0xFF);

	for(uint32_t i = 0; i < rl; i++) {
		data_MSB = ((data[i] >> 8) & 0xFF);
		data_LSB = (data[i] & 0xFF);

		tx_msg[0] = CAN_TBMS_WRITE_REG_BLOCK;
		tx_msg[1] = addr_MSB;
		tx_msg[2] = addr_LSB;
		tx_msg[3] = 0x00;
		tx_msg[4] = rl;
		tx_msg[5] = data_MSB;
		tx_msg[6] = data_LSB;
		tx_msg[7] = (msg_count-1);
		CAN1_Tx(tx_msg);

		//Message counter range: 1 to n vs Byte 8: 0 to n-1
		msg_count++;
	}

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_WRITE_REG_BLOCK)) {
			printf("Response from BMS [OK]\n");
			printf("CAN_TBMS_WRITE_REG_BLOCK | CMD: 0x%02X\n", rx_msg[1]);

			//If Byte 5 is 0x00, Byte 6's rl matches input rl, and Bytes[3:4]'s address matches input address
			uint16_t addr_check = ((rx_msg[2] << 8) | rx_msg[3]);
			if((rx_msg[4] == 0x00) && (rx_msg[5] == rl) && (addr_check == addr)) {
				printf("Success! Wrote a block of %u registers starting at address 0x%04X\n", rl, addr_check);
			} else {
				printf("Data Corruption\n");
				retval = CMD_FAILURE;
				return retval;
			}

		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
uint8_t TinyBMS_CAN_ReadNewestEvents(CAN_HandleTypeDef *hcan) {
	printf("TinyBMS_CAN_ReadNewestEvents\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8];
	uint8_t pl = 0, msg_count = 1, IDn = 0;
	uint32_t BTSP = 0, TSP = 0;

	/* Request to BMS */
	uint8_t tx_msg[8] = {CAN_TBMS_READ_NEWEST_EVENTS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	CAN1_Tx(tx_msg);

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_READ_NEWEST_EVENTS)) {
			if(msg_count == 1) {
				printf("Response from BMS [OK]\n");
				printf("CAN_TBMS_READ_NEWEST_EVENTS | CMD: 0x%02X\n", rx_msg[1]);
			}
			pl = rx_msg[2];

			//MSG1 - TinyBMS Timestamp
			//If payload is 4 Bytes and Byte 8 is 0x00
			if((rx_msg[7] == 0x00) && (pl == 4)) {
				BTSP = ((rx_msg[6] << 24) | (rx_msg[5] << 16) | (rx_msg[4] << 8) | (rx_msg[3]));
				printf("TinyBMS Timestamp (s): [%lu]\n", BTSP);
				msg_count++;
			//MSG2..n - Newest Event ID + Timestamp
			//If payload is 4 Bytes and Byte 8 is 1..n-1
			} else if((rx_msg[7] == (msg_count-1)) && (pl == 4)) {
				TSP = ((0x00 << 24) | (rx_msg[5] << 16) | (rx_msg[4] << 8) | (rx_msg[3]));
				IDn = rx_msg[6];
				printf("Event - ID: 0x%02X | Timestamp (s): [%lu]\n", IDn, TSP);
				msg_count++;
			} else {
				printf("Data Corruption\n");
				retval = CMD_FAILURE;
				return retval;
			}

		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
uint8_t TinyBMS_CAN_ReadAllEvents(CAN_HandleTypeDef *hcan) {
	printf("TinyBMS_CAN_ReadAllEvents\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8];
	uint8_t pl = 0, msg_count = 1, IDn = 0;
	uint32_t BTSP = 0, TSP = 0;

	/* Request to BMS */
	uint8_t tx_msg[8] = {CAN_TBMS_READ_ALL_EVENTS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	CAN1_Tx(tx_msg);

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_READ_ALL_EVENTS)) {
			if(msg_count == 1) {
				printf("Response from BMS [OK]\n");
				printf("CAN_TBMS_READ_ALL_EVENTS | CMD: 0x%02X\n", rx_msg[1]);
			}
			pl = rx_msg[2];

			//MSG1 - TinyBMS Timestamp
			//If payload is 4 Bytes and Byte 8 is 0x00
			if((rx_msg[7] == 0x00) && (pl == 4)) {
				BTSP = ((rx_msg[6] << 24) | (rx_msg[5] << 16) | (rx_msg[4] << 8) | (rx_msg[3]));
				printf("TinyBMS Timestamp (s): [%lu]\n", BTSP);
				msg_count++;
			//MSG2..n - Event ID + Timestamp
			//If payload is 4 Bytes and Byte 8 is 1..n-1
			} else if((rx_msg[7] == (msg_count-1)) && (pl == 4)) {
				TSP = ((0x00 << 24) | (rx_msg[5] << 16) | (rx_msg[4] << 8) | (rx_msg[3]));
				IDn = rx_msg[6];
				printf("Event - ID: 0x%02X | Timestamp (s): [%lu]\n", IDn, TSP);
				msg_count++;
			} else {
				printf("Data Corruption\n");
				retval = CMD_FAILURE;
				return retval;
			}

		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
uint8_t TinyBMS_CAN_ReadBatteryPackVoltage(CAN_HandleTypeDef *hcan) {
	printf("TinyBMS_CAN_ReadBatteryPackVoltage\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8];

	/* Request to BMS */
	uint8_t tx_msg[8] = {CAN_TBMS_READ_PACK_VOLTAGE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	CAN1_Tx(tx_msg);

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_READ_PACK_VOLTAGE)) {
			printf("Response from BMS [OK]\n");
			printf("CAN_TBMS_READ_PACK_VOLTAGE | CMD: 0x%02X\n", rx_msg[1]);
			uint32_t data = ((rx_msg[5] << 24) | (rx_msg[4] << 16) | (rx_msg[3] << 8) | (rx_msg[2]));
			float packVoltage = data;
			printf("Secondary Battery Pack Voltage: %f (V)\n", packVoltage);

		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
uint8_t TinyBMS_CAN_ReadBatteryPackCurrent(CAN_HandleTypeDef *hcan) {
	printf("TinyBMS_CAN_ReadBatteryPackCurrent\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8];

	/* Request to BMS */
	uint8_t tx_msg[8] = {CAN_TBMS_READ_PACK_CURRENT, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	CAN1_Tx(tx_msg);

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_READ_PACK_CURRENT)) {
			printf("Response from BMS [OK]\n");
			printf("CAN_TBMS_READ_PACK_CURRENT | CMD: 0x%02X\n", rx_msg[1]);
			uint32_t data = ((rx_msg[5] << 24) | (rx_msg[4] << 16) | (rx_msg[3] << 8) | (rx_msg[2]));
			float packCurrent = data;
			printf("Secondary Battery Pack Current: %f (A)\n", packCurrent);

		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
uint8_t TinyBMS_CAN_ReadBatteryPackMaxCellVoltage(CAN_HandleTypeDef *hcan) {
	printf("TinyBMS_CAN_ReadBatteryPackMaxCellVoltage\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8];

	/* Request to BMS */
	uint8_t tx_msg[8] = {CAN_TBMS_READ_MAX_CELL_VOLTAGE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	CAN1_Tx(tx_msg);

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_READ_MAX_CELL_VOLTAGE)) {
			printf("Response from BMS [OK]\n");
			printf("CAN_TBMS_READ_MAX_CELL_VOLTAGE | CMD: 0x%02X\n", rx_msg[1]);
			uint16_t maxCellVoltage = ((rx_msg[3] << 8) | (rx_msg[2]));
			printf("Secondary Battery Pack Maximum Cell Voltage: %u (mV)\n", maxCellVoltage);

		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
uint8_t TinyBMS_CAN_ReadBatteryPackMinCellVoltage(CAN_HandleTypeDef *hcan) {
	printf("TinyBMS_CAN_ReadBatteryPackMinCellVoltage\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8];

	/* Request to BMS */
	uint8_t tx_msg[8] = {CAN_TBMS_READ_MIN_CELL_VOLTAGE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	CAN1_Tx(tx_msg);

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_READ_MIN_CELL_VOLTAGE)) {
			printf("Response from BMS [OK]\n");
			printf("CAN_TBMS_READ_MIN_CELL_VOLTAGE | CMD: 0x%02X\n", rx_msg[1]);
			uint16_t minCellVoltage = ((rx_msg[3] << 8) | (rx_msg[2]));
			printf("Secondary Battery Pack Minimum Cell Voltage: %u (mV)\n", minCellVoltage);

		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
uint8_t TinyBMS_CAN_ReadOnlineStatus(CAN_HandleTypeDef *hcan) {
	printf("TinyBMS_CAN_ReadOnlineStatus\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8];

	/* Request to BMS */
	uint8_t tx_msg[8] = {CAN_TBMS_READ_ONLINE_STATUS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	CAN1_Tx(tx_msg);

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_READ_ONLINE_STATUS)) {
			printf("Response from BMS [OK]\n");
			printf("CAN_TBMS_READ_ONLINE_STATUS | CMD: 0x%02X\n", rx_msg[1]);
			uint16_t onlineStatus = ((rx_msg[3] << 8) | (rx_msg[2]));

			switch(onlineStatus) {
			case TBMS_STATUS_CHARGING:
				printf("TinyBMS Online Status: 0x%02X - Charging [INFO]\n", onlineStatus);
				break;
			case TBMS_STATUS_FULLYCHARGED:
				printf("TinyBMS Online Status: 0x%02X - Fully Charged [INFO]\n", onlineStatus);
				break;
			case TBMS_STATUS_DISCHARGING:
				printf("TinyBMS Online Status: 0x%02X - Discharging [INFO]\n", onlineStatus);
				break;
			case TBMS_STATUS_REGENERATION:
				printf("TinyBMS Online Status: 0x%02X - Regeneration [INFO]\n", onlineStatus);
				break;
			case TBMS_STATUS_IDLE:
				printf("TinyBMS Online Status: 0x%02X - Idle [INFO]\n", onlineStatus);
				break;
			case TBMS_STATUS_FAULT:
				printf("TinyBMS Online Status: 0x%02X - Fault [Error]\n", onlineStatus);
				break;
			default:
				printf("Invalid TinyBMS OnlineStatus received\n");
				retval = CMD_FAILURE;
				return retval;
			}

		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
uint8_t TinyBMS_CAN_ReadLifetimeCounter(CAN_HandleTypeDef *hcan) {
	printf("TinyBMS_CAN_ReadLifetimeCounter\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8];

	/* Request to BMS */
	uint8_t tx_msg[8] = {CAN_TBMS_READ_LIFETIME_COUNTER, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	CAN1_Tx(tx_msg);

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_READ_LIFETIME_COUNTER)) {
			printf("Response from BMS [OK]\n");
			printf("CAN_TBMS_READ_LIFETIME_COUNTER | CMD: 0x%02X\n", rx_msg[1]);
			uint32_t bms_lifetime = ((rx_msg[5] << 24) | (rx_msg[4] << 16) | (rx_msg[3] << 8) | (rx_msg[2]));
			printf("TinyBMS Lifetime Counter: %lu (s)\n", bms_lifetime);

		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
uint8_t TinyBMS_CAN_ReadEstimatedSOCValue(CAN_HandleTypeDef *hcan) {
	printf("TinyBMS_CAN_ReadEstimatedSOCValue\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8];

	/* Request to BMS */
	uint8_t tx_msg[8] = {CAN_TBMS_READ_EST_SOC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	CAN1_Tx(tx_msg);

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_READ_EST_SOC)) {
			printf("Response from BMS [OK]\n");
			printf("CAN_TBMS_READ_EST_SOC | CMD: 0x%02X\n", rx_msg[1]);
			uint32_t est_soc = ((rx_msg[5] << 24) | (rx_msg[4] << 16) | (rx_msg[3] << 8) | (rx_msg[2]));
			printf("TinyBMS Estimated StateOfCharge: %lu (0.000 001 %% Resolution)\n", est_soc);

		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
uint8_t TinyBMS_CAN_ReadDeviceTemperatures(CAN_HandleTypeDef *hcan) {
	printf("TinyBMS_CAN_ReadDeviceTemperatures\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8];
	uint8_t msg_count = 1, pl = 0;

	/* Request to BMS */
	uint8_t tx_msg[8] = {CAN_TBMS_READ_TEMPS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	CAN1_Tx(tx_msg);

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_READ_TEMPS)) {
			if(msg_count == 1) {
				printf("Response from BMS [OK]\n");
				printf("CAN_TBMS_READ_TEMPS | CMD: 0x%02X\n", rx_msg[1]);
			}
			pl = rx_msg[2];

			//MSG1
			if((pl == 2) && (rx_msg[5] == 0x00)) {
				int16_t temp1 = ((rx_msg[4] << 8) | (rx_msg[3]));
				printf("TinyBMS   Internal Temp: %d (°C)\n", temp1);
				msg_count++;
			//MSG2
			} else if((pl == 2) && (rx_msg[5] == 0x01)) {
				int16_t temp2 = ((rx_msg[4] << 8) | (rx_msg[3]));
				printf("TinyBMS External Temp 1: %d (°C)\n", temp2);
				msg_count++;
			//MSG3
			} else if((pl == 2) && (rx_msg[5] == 0x02)) {
				int16_t temp3 = ((rx_msg[4] << 8) | (rx_msg[3]));
				printf("TinyBMS External Temp 2: %d (°C)\n", temp3);
			} else {
				printf("Data Corruption\n");
				retval = CMD_FAILURE;
				return retval;
			}

		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
uint8_t TinyBMS_CAN_ReadBatteryPackCellVoltages(CAN_HandleTypeDef *hcan) {
	printf("TinyBMS_CAN_ReadBatteryPackCellVoltages\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8];
	uint8_t msg_count = 1, pl = 0;
	uint16_t cellVoltage = 0;

	/* Request to BMS */
	uint8_t tx_msg[8] = {CAN_TBMS_READ_CELL_VOLTAGES, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	CAN1_Tx(tx_msg);

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_READ_CELL_VOLTAGES)) {
			if(msg_count == 1) {
				printf("Response from BMS [OK]\n");
				printf("CAN_TBMS_READ_CELL_VOLTAGES | CMD: 0x%02X\n", rx_msg[1]);
			}
			pl = rx_msg[2];

			//If DATAn is 2 bytes in length and Byte 6 counter is correct
			//Message counter range: 1 to n vs Byte 6: 0 to n-1
			//msg_count is equal to the cell_count
			if((pl == 2) && (rx_msg[5] == (msg_count-1))) {
				cellVoltage = ((rx_msg[4] << 8) | rx_msg[3]);
				printf("Secondary Battery Pack - Cell#: %u | Voltage: %u (0.1mV Resolution)\n", msg_count, cellVoltage);
				msg_count++;
			} else {
				printf("Data Corruption\n");
				retval = CMD_FAILURE;
				return retval;
			}

		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
uint8_t TinyBMS_CAN_ReadSettingsValues(CAN_HandleTypeDef *hcan, uint8_t option, uint8_t rl) {
	printf("TinyBMS_CAN_ReadSettingsValues\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8];
	uint8_t msg_count = 1, pl = 0;
	uint16_t data = 0;

	switch(option) {
	case 0x01:
		printf("0x01 Min Settings\n");
		break;
	case 0x02:
		printf("0x02 Max Settings\n");
		break;
	case 0x03:
		printf("0x03 Default Settings\n");
		break;
	case 0x04:
		printf("0x04 Current Settings\n");
	default:
		printf("Invalid option\n");
		retval = CMD_FAILURE;
		return retval;
	}

	//Check if number of registers to read is within bounds
	if((rl <= 0) || (rl > 0x64)) {
		retval = CMD_FAILURE;
		return retval;
	}

	/* Request to BMS */
	uint8_t tx_msg[8] = {CAN_TBMS_READ_SETTINGS_VALUES, option, 0x00, rl, 0x00, 0x00, 0x00, 0x00};
	CAN1_Tx(tx_msg);

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_READ_SETTINGS_VALUES)) {
			if(msg_count == 1) {
				printf("Response from BMS [OK]\n");
				printf("CAN_TBMS_READ_SETTINGS_VALUES | CMD: 0x%02X\n", rx_msg[1]);
			}
			pl = rx_msg[2];

			//If DATAn is 2 bytes in length and Byte 6 counter is correct
			//Message counter range: 1 to n vs Byte 6: 0 to n-1
			//msg_count is equal to the settings_count
			if((pl == 2) && (rx_msg[5] == (msg_count-1))) {
				data = ((rx_msg[4] << 8) | rx_msg[3]);
				printf("TinyBMS Setting#: %u | Value: %u\n", msg_count, data);
				msg_count++;
			} else {
				printf("Data Corruption\n");
				retval = CMD_FAILURE;
				return retval;
			}

		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
uint8_t TinyBMS_CAN_ReadVersion(CAN_HandleTypeDef *hcan) {
	printf("TinyBMS_CAN_ReadVersion\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8];
	uint8_t pl = 0;
	uint8_t hw_version = 0, hw_changes = 0, firmware_public = 0;
	uint16_t firmware_internal = 0;

	/* Request to BMS */
	uint8_t tx_msg[8] = {CAN_TBMS_READ_VERSION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	CAN1_Tx(tx_msg);

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_READ_VERSION)) {
			printf("Response from BMS [OK]\n");
			printf("CAN_TBMS_READ_VERSION | CMD: 0x%02X\n", rx_msg[1]);
			pl = rx_msg[2];

			switch(pl) {
			case 1: //DATA1
				hw_version = rx_msg[3];
				printf("TinyBMS Hardware Version: %u\n", hw_version);
				break;
			case 2: //DATA1 + DATA2
				hw_version = rx_msg[3];
				hw_changes = rx_msg[4];
				printf("TinyBMS Hardware Version: %u\n", hw_version);
				printf("TinyBMS Hardware Changes Version: %u\n", hw_changes);
				break;
			case 3: //DATA1 + DATA2 + DATA3
				hw_version = rx_msg[3];
				hw_changes = rx_msg[4];
				firmware_public = rx_msg[5];
				printf("TinyBMS Hardware Version: %u\n", hw_version);
				printf("TinyBMS Hardware Changes Version: %u\n", hw_changes);
				printf("TinyBMS Firmware Public Version: %u\n", firmware_public);
				break;
			case 5: //DATA1 + DATA2 + DATA3 + DATA4
				hw_version = rx_msg[3];
				hw_changes = rx_msg[4];
				firmware_public = rx_msg[5];
				firmware_internal = ((rx_msg[7] << 8) | rx_msg[6]);
				printf("TinyBMS Hardware Version: %u\n", hw_version);
				printf("TinyBMS Hardware Changes Version: %u\n", hw_changes);
				printf("TinyBMS Firmware Public Version: %u\n", firmware_public);
				printf("TinyBMS Firmware Internal Version: %u\n", firmware_internal);
				break;
			default:
				printf("Invalid Payload Value\n");
				retval = CMD_FAILURE;
				return retval;
			}
		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
uint8_t TinyBMS_CAN_ReadCalcSpeedDistanceLeftEstTimeLeft(CAN_HandleTypeDef *hcan) {
	printf("TinyBMS_CAN_ReadCalcSpeedDistanceLeftEstTimeLeft\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8];
	uint8_t pl = 0, msg_count = 1;

	/* Request to BMS */
	uint8_t tx_msg[8] = {CAN_TBMS_READ_SPEED_DISTANCETIME_LEFT, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	CAN1_Tx(tx_msg);

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_READ_SPEED_DISTANCETIME_LEFT)) {
			if(msg_count == 1) {
				printf("Response from BMS [OK]\n");
				printf("CAN_TBMS_READ_SPEED_DISTANCETIME_LEFT | CMD: 0x%02X\n", rx_msg[1]);
			}
			pl = rx_msg[2];

			//MSG1 - SPEED
			if((pl == 4) && (rx_msg[7] == 0x00)) {
				uint32_t data1 = ((rx_msg[6] << 24) | (rx_msg[5] << 16) | (rx_msg[4] << 8) | (rx_msg[3]));
				float speed = data1;
				printf("TinyBMS Speed: %f (km/h)\n", speed);
				msg_count++;
			//MSG2 - DISTANCE LEFT
			} else if((pl == 4) && (rx_msg[7] == 0x01)) {
				uint32_t distanceLeft = ((rx_msg[6] << 24) | (rx_msg[5] << 16) | (rx_msg[4] << 8) | (rx_msg[3]));
				printf("TinyBMS Distance Left until Empty: %lu (km)\n", distanceLeft);
				msg_count++;
			//MSG3 - ESTIMATED TIME LEFT
			} else if((pl == 4) && (rx_msg[7] == 0x02)) {
				uint32_t timeLeft = ((rx_msg[6] << 24) | (rx_msg[5] << 16) | (rx_msg[4] << 8) | (rx_msg[3]));;
				printf("TinyBMS Time Left until Empty: %lu (s)\n", timeLeft);
			} else {
				printf("Data Corruption\n");
				retval = CMD_FAILURE;
				return retval;
			}

		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
uint8_t TinyBMS_CAN_ReadNodeID(CAN_HandleTypeDef *hcan) {
	printf("TinyBMS_CAN_ReadNodeID\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8];

	/* Request to BMS */
	uint8_t tx_msg[8] = {CAN_TBMS_READ_CAN_NODEID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	CAN1_Tx(tx_msg);

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_READ_CAN_NODEID)) {
			printf("Response from BMS [OK]\n");
			printf("CAN_TBMS_READ_CAN_NODEID | CMD: 0x%02X\n", rx_msg[1]);
			uint8_t nodeID_current = rx_msg[2];
			printf("TinyBMS Current CAN NodeID: %d\n", nodeID_current);

		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
uint8_t TinyBMS_CAN_WriteNodeID(CAN_HandleTypeDef *hcan, uint8_t nodeID) {
	printf("TinyBMS_CAN_WriteNodeID\n");
	uint8_t retval = CMD_FAILURE;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rx_msg[8];

	/* Request to BMS */
	uint8_t tx_msg[8] = {CAN_TBMS_WRITE_CAN_NODEID, nodeID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	CAN1_Tx(tx_msg);

	/* Response from BMS */
	//Loop until there are no more remaining messages in CAN_RX_FIFO0
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_msg) != HAL_OK) {
			Error_Handler();
		}
		//Activate Notifications (Interrupts) by setting CAN_IER bits
		if(HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) != HAL_OK) {
			Error_Handler();
		}

		//[ERROR]
		if(rx_msg[0] == NACK) {
			printf("Response from BMS [Error]\n");
			printf("CMD: 0x%02X | ERROR Code: 0x%02X\n", rx_msg[1], rx_msg[2]);
			uint8_t error = rx_msg[2];
			retval = error;
			return retval;

		//[OK]
		} else if((rx_msg[0] == ACK) && (rx_msg[1] == CAN_TBMS_WRITE_CAN_NODEID)) {
			printf("Response from BMS [OK]\n");
			printf("CAN_TBMS_WRITE_CAN_NODEID | CMD: 0x%02X\n", rx_msg[1]);
			uint8_t nodeID_new = rx_msg[2];
			printf("TinyBMS New CAN NodeID: %d\n", nodeID_new);

		} else {
			printf("Data Corruption\n");
			retval = CMD_FAILURE;
			return retval;
		}
	}
	retval = CMD_SUCCESS;
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
