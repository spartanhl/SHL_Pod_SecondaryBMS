/***********************************************
* @file TinyBMS.c
* @brief TinyBMS Library
* @author Oliver Moore
* @version 1.0
* @date 11-21-2021
***********************************************/

/**************** Header Files ****************/
#include "TinyBMS.h"

/*************** Static Function Prototypes **************/
static int TinyBMS_ACK(uint8_t cmd);
static int TinyBMS_ReadRegBlock(uint8_t rl, uint16_t addr);
static int TinyBMS_ReadRegIndividual(uint8_t pl, uint16_t* addr);
static int TinyBMS_WriteRegBlock(uint8_t pl, uint16_t addr, uint16_t* data);
static int TinyBMS_WriteRegIndividual(uint8_t pl, uint16_t* addr, uint16_t* data);
static int TinyBMS_ReadRegBlockMODBUS(uint16_t addr, uint8_t rl);
static int TinyBMS_WriteRegBlockMODBUS(uint16_t addr, uint8_t rl, uint8_t pl, uint16_t* data);

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
 * @param[in]			-  cmd - Command code (uint8_t)
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				-  0x00 - CMD ERROR , 0x01 - CRC ERROR
 *
 */
static int TinyBMS_ACK(uint8_t cmd) {

	int retval = -1;
	uint8_t error;

	return retval;
}

//1.1.2 Read TinyBMS Registers Block
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadRegBlock
 *
 * @brief				-  Read from a register block
 *
 * @param[in]			-  rl - Number (length) of registers to read (uint8_t)
 * @param[in]			-  addr - First register's block address (LSB first) (uint16_t)
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				-  Returns a pointer to the address of the first word of the data block (LSB first).
 * 						   	A memory block is a group of one or more contiguous bytes of memory allocated
 * 						  	by malloc(size_t size).
 */
static int TinyBMS_ReadRegBlock(uint8_t rl, uint16_t addr) {

	int retval = -1;

	return retval;
}

//1.1.3 Read TinyBMS Individual Registers
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadRegIndividual
 *
 * @brief				-  Read from a single (or multiple) individual registers
 *
 * @param[in]			-  pl - Payload Length in bytes (byte length = 2*n registers . Every register
 * 							contains 2 bytes) (uint8_t)
 * @param[in]			-  addr - a pointer to the first address of data structure containing the register
 * 							addresses to be read from (LSB first) (uint16_t*)
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				-  The order of data values returned will match the order of addresses given.
 * 										Individual Register Response from BMS [OK]:
 * 						 	Byte n*4   Byte n*4+1  Byte n*4+2  Byte n*4+3  Byte n*4+4  Byte n*4+5
 * 						 	ADDRn:LSB  ADDRn:MSB   DATAn:LSB   DATAn:MSB   CRC:LSB     CRC:MSB
 * 						 	      [UINT16]			     [UINT16]
 */
static int TinyBMS_ReadRegIndividual(uint8_t pl, uint16_t* addr) {

	int retval = -1;
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
 * @param[in]			-  pl - Payload Length in bytes (2*n registers) (uint8_t)
 * @param[in]			-  addr - First register's block address (LSB first) (uint16_t)
 * @param[in]			-  data - a pointer to the first address of data structure containing the register
 * 							values (LSB first) (uint16_t*)
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				-  A memory block is a group of one or more contiguous bytes of memory allocated by malloc(size_t size).
 */
static int TinyBMS_WriteRegBlock(uint8_t pl, uint16_t addr, uint16_t* data) {
	int retval = -1;
	return retval;
}

//1.1.5 Write TinyBMS Individual Registers
/* ******************************************************************************
 * @fn					-  TinyBMS_WriteRegIndividual
 *
 * @brief				-  Write to single (or multiple) individual registers
 *
 * @param[in]			-  pl - Payload Length in bytes (2*n registers) (uint8_t)
 * @param[in]			-  addr - a pointer to the first address of data structure containing the register
 * 							addresses to be read from (LSB first) (uint16_t*) *NOTE*
 * @param[in]			-  data - a pointer to the first address of data structure containing the register
 * 							values (LSB first) (uint16_t*) *NOTE*
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				-  The order of data values given coincides with the order of addresses given.
 * 										Individual Register request to BMS:
 * 						 	Byte n*4   Byte n*4+1  Byte n*4+2  Byte n*4+3  Byte n*4+4  Byte n*4+5
 * 						 	ADDRn:LSB  ADDRn:MSB   DATAn:LSB   DATAn:MSB   CRC:LSB     CRC:MSB
 * 						 	      [UINT16]			     [UINT16]
 */
static int TinyBMS_WriteRegIndividual(uint8_t pl, uint16_t* addr, uint16_t* data) {
	int retval = -1;
	return retval;
}

//1.1.6 Read TinyBMS Registers Block (MODBUS compatible)
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadRegBlockMODBUS
 *
 * @brief				-  Read from a register block (MODBUS compatible)
 *
 * @param[in]			-  addr - First register's block address (MSB first!!) (uint16_t)
 * @param[in]			-  rl - Number (length) of registers to read (uint8_t) **Max 127 registers (0x7F)**
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				-  A memory block is a group of one or more contiguous bytes of memory allocated by malloc(size_t size).
 * 						   MODBUS addresses and data is MSB first (Big Endian). CRC however is still LSB First! (Little Endian)
 * 							  Individual Register Response from BMS [OK]:
 * 						 	  Byte n*2+2  Byte n*2+3  Byte n*2+4  Byte n*2+5
 * 						 	  DATAn:MSB   DATAn:LSB   CRC:LSB     CRC:MSB
 * 						 	        [UINT16]
 */
static int TinyBMS_ReadRegBlockMODBUS(uint16_t addr, uint8_t rl) {
	int retval = -1;
	return retval;
}

//1.1.7 Write TinyBMS Registers Block (MODBUS compatible)
/* ******************************************************************************
 * @fn					-  TinyBMS_WriteRegBlockMODBUS
 *
 * @brief				-  Write to a register block (MODBUS compatible)
 *
 * @param[in]			-  addr - First register's block address (MSB first!!) (uint16_t)
 * @param[in]			-  rl - Number (length) of registers to write (uint8_t) **Max 100 registers (0x64)**
 * @param[in]			-  pl - Payload Length in bytes (2*n registers) (uint8_t)
 * @param[in]			-  data - a pointer to the first address of data structure containing the register
 * 							values (MSB first!!) (uint16_t*) *NOTE*
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				-  A memory block is a group of one or more contiguous bytes of memory allocated by malloc(size_t size).
 * 						   MODBUS addresses and data is MSB first (Big Endian). CRC however is still LSB First! (Little Endian)
 * 							  Individual Register Response from BMS [OK]:
 * 						 	  Byte n*2+6  Byte n*2+7  Byte n*2+8  Byte n*2+9
 * 						 	  DATAn:MSB   DATAn:LSB   CRC:LSB     CRC:MSB
 * 						 	        [UINT16]
 */
static int TinyBMS_WriteRegBlockMODBUS(uint16_t addr, uint8_t rl, uint8_t pl, uint16_t* data) {
	int retval = -1;
	return retval;
}

//1.1.8 Reset TinyBMS, Clear Events and Statistics
/* ******************************************************************************
 * @fn					-  TinyBMS_ResetClearEventsStatistics
 *
 * @brief				-  Reset BMS, Clear Events, or Clear Statistics depending on option
 *
 * @param[in]			-  option (uint8_t) *NOTE*
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				-  0x01 - Clear Events , 0x02 - Clear Statistics , 0x05 - Reset BMS
 *
 */
int TinyBMS_ResetClearEventsStatistics(uint8_t option) {
	int retval = -1;
	return retval;
}

//1.1.9 Read TinyBMS Newest Events
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadNewestEvents
 *
 * @brief				-  Read BMS newest events with timestamps and event ID's
 *
 * @param[in]			-  N/A
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				-  This function does not take any arguments or return timestamp/event ID info.
 * 						   The BMS response is a bit complex to parse through, but will be output/printed
 * 						   from within the API.
 * 						   PL: Payload length in bytes [UINT8]. BTSP: BMS timestamp in seconds [UINT32].
 * 						   TSP: Event timestamp in seconds [UINT24]. EVENT: BMS Event ID [UINT8].
 */
int TinyBMS_ReadNewestEvents(void) {
	int retval = -1;
	return retval;
}

//1.1.10 Read TinyBMS All Events
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadAllEvents
 *
 * @brief				-  Read BMS all events with timestamps and event ID's
 *
 * @param[in]			-  N/A
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				-  This function does not take any arguments or return timestamp/event ID info.
 * 						   The BMS response is a bit complex to parse through, but will be output/printed
 * 						   from within the API.
 * 						   PL: Payload length in bytes [UINT8]. BTSP: BMS timestamp in seconds [UINT32].
 * 						   TSP: Event timestamp in seconds [UINT24]. EVENT: BMS Event ID [UINT8].
 */
int TinyBMS_ReadAllEvents(void) {
	int retval = -1;
	return retval;
}

//1.1.11 Read Battery Pack Voltage
//Reg:36
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadBatteryPackVoltage
 *
 * @brief				-  Read Battery Pack Voltage - 1V Resolution
 *
 * @param[in]			-  N/A
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				-  			Response from BMS [OK]:
 * 						 	  Byte3      Byte4  Byte5  	Byte6 		Byte7 	Byte8
 * 						 	  DATA:LSB   DATA   DATA	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	               [FLOAT]
 */
int TinyBMS_ReadBatteryPackVoltage(void) {
	int retval = -1;
	return retval;
}

//1.1.12 Read Battery Pack Current
//Reg:38
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadBatteryPackCurrent
 *
 * @brief				-  Read Battery Pack Current - 1A Resolution
 *
 * @param[in]			-  N/A
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				-  			Response from BMS [OK]:
 * 						 	  Byte3      Byte4  Byte5  	Byte6 		Byte7 	Byte8
 * 						 	  DATA:LSB   DATA   DATA	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	               [FLOAT]
 */
int TinyBMS_ReadBatteryPackCurrent(void) {
	int retval = -1;
	return retval;
}

//1.1.13 Read Battery Pack Max Cell Voltage
//Reg:41
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadBatteryPackMaxCellVoltage
 *
 * @brief				-  Read Battery Pack Maximum Cell Voltage - 1mV Resolution
 *
 * @param[in]			-  N/A
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				-  			Response from BMS [OK]:
 * 						 	  Byte3        	Byte4 		Byte5 	Byte6
 * 						 	  DATA:LSB   	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        [UINT16]
 */
int TinyBMS_ReadBatteryPackMaxCellVoltage(void) {
	int retval = -1;
	return retval;
}

//1.1.14 Read Battery Pack Min Cell Voltage
//Reg:40
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadBatteryPackMinCellVoltage
 *
 * @brief				-  Read Battery Pack Minimum Cell Voltage - 1mV Resolution
 *
 * @param[in]			-  N/A
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				- 			Response from BMS [OK]:
 * 						 	  Byte3        	Byte4 		Byte5 	Byte6
 * 						 	  DATA:LSB   	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        [UINT16]
 */
int TinyBMS_ReadBatteryPackMinCellVoltage(void) {
	int retval = -1;
	return retval;
}

//1.1.15 Read TinyBMS Online Status
//Reg:50
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadOnlineStatus
 *
 * @brief				-  Read TinyBMS Online Status
 *
 * @param[in]			-  N/A
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				-			Response from BMS [OK]:
 * 						 	  Byte3        	Byte4 		Byte5 	Byte6
 * 						 	  DATA:LSB   	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        [UINT16]
 * 						   0x91 - Charging [INFO], 0x92 - Fully Charged [INFO]
 * 						   0x93 - Discharging [INFO], 0x94 - Regeneration [INFO]
 * 						   0x97 - Idle [INFO], 0x9B - Fault [ERROR]
 */
int TinyBMS_ReadOnlineStatus(void) {
	int retval = -1;
	return retval;
}

//1.1.16 Read TinyBMS Lifetime Counter
//Reg:32
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadLifetimeCounter
 *
 * @brief				-  Read TinyBMS Lifetime Counter - 1s Resolution
 *
 * @param[in]			-  N/A
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				-  				Response from BMS [OK]:
 * 						 	  Byte3      Byte4 	Byte5 	Byte6		Byte5 	Byte6
 * 						 	  DATA:LSB   DATA	DATA	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        	  [UINT32]
 */
int TinyBMS_ReadLifetimeCounter(void) {
	int retval = -1;
	return retval;
}

//1.1.17 Read TinyBMS Estimated SOC Value
//Reg:46
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadEstimatedSOCValue
 *
 * @brief				-  Read TinyBMS Estimated SOC (State of Charge) Value - 0.000 001 % Resolution
 *
 * @param[in]			-  N/A
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				-  				Response from BMS [OK]:
 * 						 	  Byte3      Byte4 	Byte5 	Byte6		Byte5 	Byte6
 * 						 	  DATA:LSB   DATA	DATA	DATA:MSB   	CRC:LSB CRC:MSB
 * 						 	        	  [UINT32]
 */
int TinyBMS_ReadEstimatedSOCValue(void) {
	int retval = -1;
	return retval;
}

//1.1.18 Read TinyBMS Device Temperatures
//Reg:48,42,43
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadDeviceTemperatures
 *
 * @brief				-  Read TinyBMS Device Temperatures - 0.1°C Resolution
 *
 * @param[in]			-  N/A
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				-  				Response from BMS [OK]:
 * 						 	Byte4      Byte5 		Byte6 		Byte7		Byte 8	 	Byte 9 		Byte10 	Byte11
 * 						 	DATA1:LSB  DATA1:MSB	DATA2:LSB	DATA2:MSB	DATA3:LSB	DATA3:MSB   CRC:LSB CRC:MSB
 * 						 	      [INT16]				   [INT16] 				  [INT16]
 * 						  	(Reg 48) DATA1 - TinyBMS Internal Temperature
 * 						  	(Reg 42) DATA2 - External Temp Sensor #1 (value of -327689 if NC)
 * 						  	(Reg 43) DATA3 - External Temp Sensor #2 (value of -327689 if NC)
 */
int TinyBMS_ReadDeviceTemperatures(void) {
	int retval = -1;
	return retval;
}

//1.1.19 Read Battery Pack Cell Voltages
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadBatteryPackCellVoltages
 *
 * @brief				-  Read TinyBMS Battery Pack Cell Voltages
 *
 * @param[in]			-  N/A
 *
 * @return				-  0 Success, -1 Failure
 *
 * @note				- 			Response from BMS [OK]:
 * 						 	  Byte n*2+2    Byte n*2+3 		Byte n*2+4 	Byte n*2+5
 * 						 	  DATAn:LSB   	DATAn:MSB   	CRC:LSB 	CRC:MSB
 * 						 	       	 [UINT16]
 */
int TinyBMS_ReadBatteryPackCellVoltages(void) {
	int retval = -1;
	return retval;
}

//1.1.20 Read TinyBMS Settings Values (min, max, default, current)
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadSettingsValues
 *
 * @brief				-  Read TinyBMS Settings values (min, max, default, current)
 *
 * @param[in]			-  option - (uint8_t) *NOTE*
 * @param[in]			-  rl - (uint8_t) *NOTE*
 *
 * @return				-  0 Success, -1 Failure
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
int TinyBMS_ReadSettingsValues(uint8_t option, uint8_t rl) {
	int retval = -1;
	return retval;
}

//1.1.21 Read TinyBMS Version
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadVersion
 *
 * @brief				-  Read TinyBMS Version
 *
 * @param[in]			-  N/A
 *
 * @return				-  0 Success, -1 Failure
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
int TinyBMS_ReadVersion(void) {
	int retval = -1;
	return retval;
}

//1.1.22 Read TinyBMS Extended Version
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadVersionExtended
 *
 * @brief				-  Read TinyBMS Extended Version
 *
 * @param[in]			-  N/A
 *
 * @return				-  0 Success, -1 Failure
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
int TinyBMS_ReadVersionExtended(void) {
	int retval = -1;
	return retval;
}

//1.1.23 Read TinyBMS Calculated Speed, Distance Left and Estimated Time Left Values
/* ******************************************************************************
 * @fn					-  TinyBMS_ReadCalcSpeedDistanceLeftEstTimeLeft
 *
 * @brief				-  Read TinyBMS Calculated Speed, Distance Left, Estimated Time Left
 *
 * @param[in]			-  N/A
 *
 * @return				-  0 Success, -1 Failure
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
int TinyBMS_ReadCalcSpeedDistanceLeftEstTimeLeft(void) {
	int retval = -1;
	return retval;
}


/*********** CRC Calculation ***********/
uint16_t CRC16(const uint8_t* data, uint16_t length) {
	uint8_t tmp;
	uint16_t crcWord = 0xFFFF;

	while(length--) {
		tmp = *data++ ^ crcWord;
		crcWord >>= 8;
		crcWord ^= crcTable[tmp];
	}
	return crcWord;
}

