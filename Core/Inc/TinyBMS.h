/***********************************************
* @file TinyBMS.h
* @brief TinyBMS Library Header
* @author Oliver Moore
* @version 1.4
* @date 03-02-2022
***********************************************/

#ifndef INC_TINYBMS_H_
#define INC_TINYBMS_H_

/**************** Header Files ****************/
#include <stdint.h>
#include <string.h>
#include "main.h"

/************ TinyBMS Macros ************/
#define ACK										0x01
#define NACK									0x00

#define CMD_SUCCESS								0
#define CMD_FAILURE								-1

#define CMD_ERROR								0x00
#define CRC_ERROR								0x01

/************ TinyBMS CAN Identifier Macros ************/

#define TINYBMS_CAN_NODEID_MIN					0x01
#define TINYBMS_CAN_NODEID_MAX					0x3F

#define TINYBMS_CAN_REQUEST_BASE_STDID			0x200
#define TINYBMS_CAN_REQUEST_STDID_MIN			(TINYBMS_CAN_REQUEST_BASE_STDID + TINYBMS_CAN_NODEID_MIN)
#define TINYBMS_CAN_REQUEST_STDID_MAX			(TINYBMS_CAN_REQUEST_BASE_STDID + TINYBMS_CAN_NODEID_MAX)

#define TINYBMS_CAN_RESPONSE_BASE_STDID			0x240
#define TINYBMS_CAN_RESPONSE_STDID_MIN			(TINYBMS_CAN_RESPONSE_BASE_STDID + TINYBMS_CAN_NODEID_MIN)
#define TINYBMS_CAN_RESPONSE_STDID_MAX			(TINYBMS_CAN_RESPONSE_BASE_STDID + TINYBMS_CAN_NODEID_MAX)

#define TINYBMS_DEFAULT_CAN_NODEID				0x01
#define TINYBMS_CAN_REQUEST_DEFAULT_STDID		(TINYBMS_CAN_REQUEST_BASE_STDID + TINYBMS_DEFAULT_CAN_NODEID)
#define TINYBMS_CAN_RESPONSE_DEFAULT_STDID		(TINYBMS_CAN_RESPONSE_BASE_STDID + TINYBMS_DEFAULT_CAN_NODEID)

/************ TinyBMS UART Command Macros ************/
#define UART_TBMS_ACK							0x01
#define UART_TBMS_READ_REG_BLOCK				0x07
#define UART_TBMS_READ_INDIVIDUAL_REGS			0x09
#define UART_TBMS_WRITE_REG_BLOCK				0x0B
#define UART_TBMS_WRITE_INDIVIDUAL_REGS			0x0D
#define UART_TBMS_READ_REG_BLOCK_MODBUS			0x03
#define UART_TBMS_WRITE_REG_BLOCK_MODBUS		0x10
#define UART_TBMS_RESET_CLEAR_EVENTS_STATS		0x02
#define UART_TBMS_READ_NEWEST_EVENTS			0x11
#define UART_TBMS_READ_ALL_EVENTS				0x12
#define UART_TBMS_READ_PACK_VOLTAGE				0x14
#define UART_TBMS_READ_PACK_CURRENT				0x15
#define UART_TBMS_READ_MAX_CELL_VOLTAGE			0x16
#define UART_TBMS_READ_MIN_CELL_VOLTAGE			0x17
#define UART_TBMS_READ_ONLINE_STATUS			0x18
#define UART_TBMS_READ_LIFETIME_COUNTER			0x19
#define UART_TBMS_READ_EST_SOC					0x1A
#define UART_TBMS_READ_TEMPS					0x1B
#define UART_TBMS_READ_CELL_VOLTAGES			0x1C
#define UART_TBMS_READ_SETTINGS_VALUES			0x1D
#define UART_TBMS_READ_VERSION					0x1E
#define UART_TBMS_READ_VERSION_EXTENDED			0x1F
#define UART_TBMS_READ_SPEED_DISTANCETIME_LEFT	0x20

/************ TinyBMS CAN Command Macros ************/
#define CAN_TBMS_RESET_CLEAR_EVENTS_STATS		0x02
#define CAN_TBMS_READ_REG_BLOCK					0x03
#define CAN_TBMS_WRITE_REG_BLOCK				0x10
#define CAN_TBMS_READ_NEWEST_EVENTS				0x11
#define CAN_TBMS_READ_ALL_EVENTS				0x12
#define CAN_TBMS_READ_PACK_VOLTAGE				0x14
#define CAN_TBMS_READ_PACK_CURRENT				0x15
#define CAN_TBMS_READ_MAX_CELL_VOLTAGE			0x16
#define CAN_TBMS_READ_MIN_CELL_VOLTAGE			0x17
#define CAN_TBMS_READ_ONLINE_STATUS				0x18
#define CAN_TBMS_READ_LIFETIME_COUNTER			0x19
#define CAN_TBMS_READ_EST_SOC					0x1A
#define CAN_TBMS_READ_TEMPS						0x1B
#define CAN_TBMS_READ_CELL_VOLTAGES				0x1C
#define CAN_TBMS_READ_SETTINGS_VALUES			0x1D
#define CAN_TBMS_READ_VERSION					0x1E
#define CAN_TBMS_READ_SPEED_DISTANCETIME_LEFT	0x20
#define CAN_TBMS_READ_CAN_NODEID				0x28
#define CAN_TBMS_WRITE_CAN_NODEID				0x29

/************ TinyBMS Reset/Clear Macros ************/
#define TINYBMS_CLEAR_EVENTS					0x01
#define TINYBMS_CLEAR_STATS						0x02
#define TINYBMS_RESET_BMS						0x05
/************ TinyBMS Online Status Macros ************/
#define TINYBMS_STATUS_CHARGING					0x91
#define TINYBMS_STATUS_FULLYCHARGED				0x92
#define TINYBMS_STATUS_DISCHARGING				0x93
#define TINYBMS_STATUS_REGENERATION				0x96
#define TINYBMS_STATUS_IDLE						0x97
#define TINYBMS_STATUS_FAULT					0x9B
/************ TinyBMS Settings Values Macros ************/
#define TINYBMS_SETTINGS_MIN					0x01
#define TINYBMS_SETTINGS_MAX					0x02
#define TINYBMS_SETTINGS_DEFAULT				0x03
#define TINYBMS_SETTINGS_CURRENT				0x04


/**************************** Register Addresses ****************************/

/*********** Tiny BMS Live Data ***********/
#define CELL1_VOLTAGE 							0		// [UINT_16] 0.1mV Resolution 					(R)
#define CELL2_VOLTAGE 							1		// [UINT_16] 0.1mV Resolution 					(R)
#define CELL3_VOLTAGE 							2		// [UINT_16] 0.1mV Resolution 					(R)
#define CELL4_VOLTAGE 							3		// [UINT_16] 0.1mV Resolution 					(R)
#define CELL5_VOLTAGE 							4		// [UINT_16] 0.1mV Resolution 					(R)
#define CELL6_VOLTAGE 							5		// [UINT_16] 0.1mV Resolution 					(R)
#define CELL7_VOLTAGE 							6		// [UINT_16] 0.1mV Resolution 					(R)
#define CELL8_VOLTAGE 							7		// [UINT_16] 0.1mV Resolution 					(R)
#define CELL9_VOLTAGE 							8		// [UINT_16] 0.1mV Resolution 					(R)
#define CELL10_VOLTAGE 							9		// [UINT_16] 0.1mV Resolution 					(R)
#define CELL11_VOLTAGE 							10		// [UINT_16] 0.1mV Resolution 					(R)
#define CELL12_VOLTAGE 							11		// [UINT_16] 0.1mV Resolution 					(R)
#define CELL13_VOLTAGE 							12		// [UINT_16] 0.1mV Resolution 					(R)
#define CELL14_VOLTAGE 							13		// [UINT_16] 0.1mV Resolution 					(R)
#define CELL15_VOLTAGE 							14		// [UINT_16] 0.1mV Resolution 					(R)
#define CELL16_VOLTAGE 							15		// [UINT_16] 0.1mV Resolution 					(R)
/* RESERVED 16-31 */
#define BMS_LIFETIME_COUNTER					32 		// [UINT_32] 1s Resolution    					(R)   	//High Word: 33
#define ESTIMATED_TIME_LEFT						34 		// [UINT_32] 1s Resolution    					(R)   	//High Word: 35
#define BATTERY_PACK_VOLTAGE					36 		// [FLOAT]   1V Resolution    					(R)   	//High Word: 37
#define BATTERY_PACK_CURRENT					38 		// [FLOAT]   1A Resolution    					(R)   	//High Word: 39
#define MINIMUM_CELL_VOLTAGE					40 		// [UINT_16] 1mV Resolution  					(R)
#define MAXIMUM_CELL_VOLTAGE					41 		// [UINT_16] 1mV Resolution   					(R)
#define EXTERNAL_TEMP_SENSOR1_TEMP				42		// [INT_16]  0.1°C Resolution 					(R)
#define EXTERNAL_TEMP_SENSOR2_TEMP				43		// [INT_16]  0.1°C Resolution 					(R)
#define DISTANCE_LEFT_TO_EMPTY					44		// [UINT_16] 1km Resolution   					(R)
/* RESERVED 45 */
#define STATE_OF_CHARGE_LIVE					46    	// [UINT_32] 0.000 001 % Resolution 			(R)		//High Word: 47
#define BMS_INTERNAL_TEMP						48		// [INT_16]  0.1°C Resolution 					(R)
/* RESERVED 49 */
#define BMS_ONLINE_STATUS						50		// [UINT_16] 									(R)
// ^ 91-Charging, 92-Fully Charged, 93-Discharging, 96-Regeneration, 97-Idle, 9B-Fault

#define BALANCING_DECISION_BITS					51		// [UINT_16] 									(R)
// ^ First Cell - LSB Bit of Lower Byte: 1-needs balancing, 0-doesn't need balancing

#define REAL_BALANCING_BITS						52		// [UINT_16]									(R)
// ^ First Cell - LSB Bit of Lower Byte: 1-balancing, 0-not balancing

#define NUMBER_OF_DETECTED_CELLS				53		// [UINT_16] 									(R)
#define SPEED									54 		// [FLOAT]   km/h 								(R)		//High Word: 55
/* RESERVED 56-99 */

/*********** Tiny BMS Statistics Data ***********/
#define TOTAL_DISTANCE							100		// [UINT_32] 0.01km Resolution 					(R)		//High Word: 101
#define MAXIMUM_DISCHARGE_CURRENT				102		// [UINT_16] 100mA Resolution					(R)
#define MAXIMUM_CHARGE_CURRENT					103		// [UINT_16] 100mA Resolution					(R)
#define MAXIMUM_CELL_VOLTAGE_DIFFERENCE			104		// [UINT_16] 0.1mV Resolution					(R)
#define UNDERVOLTAGE_PROTECTION_COUNT			105		// [UINT_16] 1 Count Resolution					(R)
#define OVERVOLTAGE_PROTECTION_COUNT			106		// [UINT_16] 1 Count Resolution					(R)
#define DISCHARGE_OVERCURRENT_PROTECTION_COUNT	107		// [UINT_16] 1 Count Resolution					(R)
#define CHARGE_OVERCURRENT_PROTECTION_COUNT		108		// [UINT_16] 1 Count Resolution					(R)
#define OVERHEAT_PROTECTION_COUNT				109		// [UINT_16] 1 Count Resolution					(R)
/* RESERVED 110 */
#define CHARGING_COUNT							111		// [UINT_16] 1 Count Resolution					(R)
#define FULL_CHARGE_COUNT						112		// [UINT_16] 1 Count Resolution					(R)
#define MIN_MAX_PACK_TEMP						113		// [INT_16]  1°C Resolution						(R)
// ^ Min Pack Temperature [8 bits Low Byte - INT_8]  | Max Pack Temperature [8 bits High Byte - INT_8]

#define LAST_BMS_RESET_SLEEP_EVENT				114		// [UINT_16] 									(R)
// ^ * Last BMS Reset Event [8 bits Low Byte - UINT_8]  | ** Last Wakeup from BMS Sleep Mode Event [8 bits High Byte - UINT_8]
// ^ *  0x00-Unknown, 0x01-Low Power Reset, 0x02-Window Watchdog Reset, 0x03-Independent Watchdog Reset,
//  0x04-Software Reset, 0x05-POR/PDR Reset, 0x06-PIN Reset, 0x07-Option Bytes Loading Reset
// ^ ** 0x00-Charger Connected, 0x01-Ignition, 0x02-Discharging Detected, 0x03-UART communication detected

/* RESERVED 115 */
#define STATS_LAST_CLEARED_TIMESTAMP			116		// [UINT_32] 1s Resolution						(R)		//High Word: 117
/* RESERVED 118-199 */

/*********** Tiny BMS Events Data ***********/
/* For each Event(0-48): The Timestamp information is stored as [UINT_24] 3 Bytes, with the Message ID contained in the 4th byte.
 * The 24-bit (bytes 1-3) Timestamp resolution is 1s, while the 4th byte (High Byte of High Word) contains the Message ID (no resolution).
 * The bitwidth of a single register is only 16 bits (2 Bytes or Word) wide, so two registers (2 words or double word)
 * are needed to contain all of the event info.
 * The data can be read, extracted, and reconstructed in multiple ways: 16+16, 16+8+8, 8+8+8+8, 32, 24+8 etc.
 * The 'Events messages ID list' is listed further down in this file under: 'Tiny BMS Events Messages'. */
#define EVENT0_TIMESTAMP_LOW16					200		// [UINT_16] 1s Resolution 						(R)
#define EVENT0_TIMESTAMP_HIGH8_MSGID			201		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT1_TIMESTAMP_LOW16					202		// [UINT_16] 1s Resolution 						(R)
#define EVENT1_TIMESTAMP_HIGH8_MSGID			203		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT2_TIMESTAMP_LOW16					204		// [UINT_16] 1s Resolution 						(R)
#define EVENT2_TIMESTAMP_HIGH8_MSGID			205		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT3_TIMESTAMP_LOW16					206		// [UINT_16] 1s Resolution 						(R)
#define EVENT3_TIMESTAMP_HIGH8_MSGID			207		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT4_TIMESTAMP_LOW16					208		// [UINT_16] 1s Resolution 						(R)
#define EVENT4_TIMESTAMP_HIGH8_MSGID			209		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT5_TIMESTAMP_LOW16					210		// [UINT_16] 1s Resolution 						(R)
#define EVENT5_TIMESTAMP_HIGH8_MSGID			211		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT6_TIMESTAMP_LOW16					212		// [UINT_16] 1s Resolution 						(R)
#define EVENT6_TIMESTAMP_HIGH8_MSGID			213		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT7_TIMESTAMP_LOW16					214		// [UINT_16] 1s Resolution 						(R)
#define EVENT7_TIMESTAMP_HIGH8_MSGID			215		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT8_TIMESTAMP_LOW16					216		// [UINT_16] 1s Resolution 						(R)
#define EVENT8_TIMESTAMP_HIGH8_MSGID			217		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT9_TIMESTAMP_LOW16					218		// [UINT_16] 1s Resolution 						(R)
#define EVENT9_TIMESTAMP_HIGH8_MSGID			219		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT10_TIMESTAMP_LOW16					220		// [UINT_16] 1s Resolution 						(R)
#define EVENT10_TIMESTAMP_HIGH8_MSGID			221		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT11_TIMESTAMP_LOW16					222		// [UINT_16] 1s Resolution 						(R)
#define EVENT11_TIMESTAMP_HIGH8_MSGID			223		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT12_TIMESTAMP_LOW16					224		// [UINT_16] 1s Resolution 						(R)
#define EVENT12_TIMESTAMP_HIGH8_MSGID			225		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT13_TIMESTAMP_LOW16					226		// [UINT_16] 1s Resolution 						(R)
#define EVENT13_TIMESTAMP_HIGH8_MSGID			227		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT14_TIMESTAMP_LOW16					228		// [UINT_16] 1s Resolution 						(R)
#define EVENT14_TIMESTAMP_HIGH8_MSGID			229		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT15_TIMESTAMP_LOW16					230		// [UINT_16] 1s Resolution 						(R)
#define EVENT15_TIMESTAMP_HIGH8_MSGID			231		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT16_TIMESTAMP_LOW16					232		// [UINT_16] 1s Resolution 						(R)
#define EVENT16_TIMESTAMP_HIGH8_MSGID			233		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT17_TIMESTAMP_LOW16					234		// [UINT_16] 1s Resolution 						(R)
#define EVENT17_TIMESTAMP_HIGH8_MSGID			235		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT18_TIMESTAMP_LOW16					236		// [UINT_16] 1s Resolution 						(R)
#define EVENT18_TIMESTAMP_HIGH8_MSGID			237		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT19_TIMESTAMP_LOW16					238		// [UINT_16] 1s Resolution 						(R)
#define EVENT19_TIMESTAMP_HIGH8_MSGID			239		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT20_TIMESTAMP_LOW16					240		// [UINT_16] 1s Resolution 						(R)
#define EVENT20_TIMESTAMP_HIGH8_MSGID			241		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT21_TIMESTAMP_LOW16					242		// [UINT_16] 1s Resolution 						(R)
#define EVENT21_TIMESTAMP_HIGH8_MSGID			243		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT22_TIMESTAMP_LOW16					244		// [UINT_16] 1s Resolution 						(R)
#define EVENT22_TIMESTAMP_HIGH8_MSGID			245		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT23_TIMESTAMP_LOW16					246		// [UINT_16] 1s Resolution 						(R)
#define EVENT23_TIMESTAMP_HIGH8_MSGID			247		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT24_TIMESTAMP_LOW16					248		// [UINT_16] 1s Resolution 						(R)
#define EVENT24_TIMESTAMP_HIGH8_MSGID			249		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT25_TIMESTAMP_LOW16					250		// [UINT_16] 1s Resolution 						(R)
#define EVENT25_TIMESTAMP_HIGH8_MSGID			251		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT26_TIMESTAMP_LOW16					252		// [UINT_16] 1s Resolution 						(R)
#define EVENT26_TIMESTAMP_HIGH8_MSGID			253		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT27_TIMESTAMP_LOW16					254		// [UINT_16] 1s Resolution 						(R)
#define EVENT27_TIMESTAMP_HIGH8_MSGID			255		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT28_TIMESTAMP_LOW16					256		// [UINT_16] 1s Resolution 						(R)
#define EVENT28_TIMESTAMP_HIGH8_MSGID			257		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT29_TIMESTAMP_LOW16					258		// [UINT_16] 1s Resolution 						(R)
#define EVENT29_TIMESTAMP_HIGH8_MSGID			259		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT30_TIMESTAMP_LOW16					260		// [UINT_16] 1s Resolution 						(R)
#define EVENT30_TIMESTAMP_HIGH8_MSGID			261		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT31_TIMESTAMP_LOW16					262		// [UINT_16] 1s Resolution 						(R)
#define EVENT31_TIMESTAMP_HIGH8_MSGID			263		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT32_TIMESTAMP_LOW16					264		// [UINT_16] 1s Resolution 						(R)
#define EVENT32_TIMESTAMP_HIGH8_MSGID			265		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT33_TIMESTAMP_LOW16					266		// [UINT_16] 1s Resolution 						(R)
#define EVENT33_TIMESTAMP_HIGH8_MSGID			267		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT34_TIMESTAMP_LOW16					268		// [UINT_16] 1s Resolution 						(R)
#define EVENT34_TIMESTAMP_HIGH8_MSGID			269		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT35_TIMESTAMP_LOW16					270		// [UINT_16] 1s Resolution 						(R)
#define EVENT35_TIMESTAMP_HIGH8_MSGID			271		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT36_TIMESTAMP_LOW16					272		// [UINT_16] 1s Resolution 						(R)
#define EVENT36_TIMESTAMP_HIGH8_MSGID			273		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT37_TIMESTAMP_LOW16					274		// [UINT_16] 1s Resolution 						(R)
#define EVENT37_TIMESTAMP_HIGH8_MSGID			275		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT38_TIMESTAMP_LOW16					276		// [UINT_16] 1s Resolution 						(R)
#define EVENT38_TIMESTAMP_HIGH8_MSGID			277		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT39_TIMESTAMP_LOW16					278		// [UINT_16] 1s Resolution 						(R)
#define EVENT39_TIMESTAMP_HIGH8_MSGID			279		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT40_TIMESTAMP_LOW16					280		// [UINT_16] 1s Resolution 						(R)
#define EVENT40_TIMESTAMP_HIGH8_MSGID			281		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT41_TIMESTAMP_LOW16					282		// [UINT_16] 1s Resolution 						(R)
#define EVENT41_TIMESTAMP_HIGH8_MSGID			283		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT42_TIMESTAMP_LOW16					284		// [UINT_16] 1s Resolution 						(R)
#define EVENT42_TIMESTAMP_HIGH8_MSGID			285		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT43_TIMESTAMP_LOW16					286		// [UINT_16] 1s Resolution 						(R)
#define EVENT43_TIMESTAMP_HIGH8_MSGID			287		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT44_TIMESTAMP_LOW16					288		// [UINT_16] 1s Resolution 						(R)
#define EVENT44_TIMESTAMP_HIGH8_MSGID			289		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT45_TIMESTAMP_LOW16					290		// [UINT_16] 1s Resolution 						(R)
#define EVENT45_TIMESTAMP_HIGH8_MSGID			291		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT46_TIMESTAMP_LOW16					292		// [UINT_16] 1s Resolution 						(R)
#define EVENT46_TIMESTAMP_HIGH8_MSGID			293		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT47_TIMESTAMP_LOW16					294		// [UINT_16] 1s Resolution 						(R)
#define EVENT47_TIMESTAMP_HIGH8_MSGID			295		// [UINT_16] (Low Byte: 1s Resolution)			(R)
#define EVENT48_TIMESTAMP_LOW16					296		// [UINT_16] 1s Resolution 						(R)
#define EVENT48_TIMESTAMP_HIGH8_MSGID			297		// [UINT_16] (Low Byte: 1s Resolution)			(R)
/* RESERVED 298 */
/* RESERVED 299 */

/*********** Tiny BMS Settings ***********/
#define FULLYCHARGED_VOLTAGE					300		// [UINT_16] [1200 to  4500] 1mV Resolution		(R/W)
#define FULLYDISCHARGED_VOLTAGE					301		// [UINT_16] [1000 to  3500] 1mV Resolution		(R/W)
/* RESERVED 302 */
#define EARLY_BALANCING_THRESHOLD				303		// [UINT_16] [1000 to  4500] 1mV Resolution		(R/W)
#define CHARGE_FINISHED_CURRENT					304		// [UINT_16] [ 100 to  5000] 1mA Resolution		(R/W) *
// *Tiny BMS device internally changes this setting's min and max values according to current sensor used

/* RESERVED 305 */
#define BATTERY_CAPACITY						306		// [UINT_16] [  10 to 65500] 0.01Ah Resolution	(R/W)
#define NUMBER_OF_SERIES_CELLS					307		// [UINT_16] [   4 to    16] 1 Cell Resolution	(R/W)
#define ALLOWED_DISBALANCE						308		// [UINT_16] [  15 to   100] 1mV Resolution		(R/W)
/* RESERVED 309-311 */
#define PULSES_PER_UNIT							312		// [UINT_32] [ 1 to  100000] 1 pulse per unit Resolution (R/W)  //High Word: 313
#define DISTANCE_UNIT_NAME						314		// [UINT_16] [1200 to  4500] 1mV Resolution		(R/W)
// ^ 0x01-Meter, 0x02-Kilometer, 0x03-Feet, 0x04-Mile, 0x05-Yard

#define OVERVOLTAGE_CUTOFF						315		// [UINT_16] [1200 to  4500] 1mV Resolution		(R/W)
#define UNDERVOLTAGE_CUTOFF						316		// [UINT_16] [ 800 to  3500] 1mV Resolution		(R/W)
#define DISCHARGE_OVERCURRENT_CUTOFF			317		// [UINT_16] [   1 to   750] 1A  Resolution		(R/W) *
// *Tiny BMS device internally changes this setting's min and max values according to current sensor used

#define CHARGE_OVERCURRENT_CUTOFF				318		// [UINT_16] [   1 to   750] 1A  Resolution		(R/W) *
// *Tiny BMS device internally changes this setting's min and max values according to current sensor used

#define OVERTEMP_CUTOFF							319		// [INT_16]  [ +20 to   +90] 1°C Resolution		(R/W)
#define LOWTEMP_CHARGER_CUTOFF					320		// [INT_16]  [ -40 to   +10] 1°C Resolution		(R/W)
/* RESERVED 321-327 */
#define STATE_OF_CHARGE_SETTING					328		// [UINT_16] [   0 to 50000] 0.002% Resolution  (R/W)
/* RESERVED 329 */

/* ** The High Bytes of the following Registers (330-343) are all RESERVED..   **
 * ** ONLY the Low Byte (bits 0-7) is used for R/W of the following registers. ** */
#define CHARGER_TYPE							330		// [UINT_8]  [1200 to  4500] 1mV Resolution		(R/W)
// ^ 0x01-Variable (Reserved), 0x01-CC/CV, 0x02-CAN (Reserved)

#define LOAD_SWITCH_TYPE						331		// [UINT_8]										(R/W)
// ^ 0x00-FET, 0x01-AIDO1, 0x02-AIDO2, 0x03-DIDO1, 0x04-DIDO2, 0x05-AIHO1 Active Low,
// ^ 0x06-AIHO1 Active High, 0x07-AIHO2 Active Low, 0x08-AIHO2 Active High

#define AUTOMATIC_RECOVERY						332		// [UINT_8]  [   1 to    30] 1s Resolution		(R/W)

#define CHARGER_SWITCH_TYPE						333		// [UINT_8] 									(R/W)
// ^ 0x01-Charge FET, 0x02-AIDO1, 0x03-AIDO2, 0x04-DIDO1, 0x05-DIDO2, 0x06-AIHO1 Active Low,
// ^ 0x07-AIHO1 Active High, 0x08-AIHO2 Active Low, 0x09-AIHO2 Active High

#define IGNITION								334		// [UINT_8] 									(R/W)
// ^ 0x00-Disabled, 0x01-AIDO1, 0x02-AIDO2, 0x03-DIDO1, 0x04-DIDO2, 0x05-AIHO1, 0x06-AIHO2

#define CHARGER_DETECTION						335		// [UINT_8] 									(R/W)
// ^ 0x01-Internal, 0x02-AIDO1, 0x03-AIDO2, 0x04-DIDO1, 0x05-DIDO2, 0x06-AIHO1, 0x07-AIHO2

#define SPEED_SENSOR_INPUT						336		// [UINT_8]										(R/W)
// ^ 0x00-Disabled, 0x01-DIDO1, 0x02-DIDO2

#define PRECHARGE_PIN							337		// [UINT_8]										(R/W)
// ^ 0x00-Disabled, 0x02-Discharge FET, 0x03-AIDO1, 0x04-AIDO2, 0x05-DIDO1, 0x06-DIDO2,
// ^ 0x07-AIHO1 Active Low, 0x08-AIHO1 Active High, 0x09-AIHO2 Active Low, 0x10-AIHO2 Active High

#define PRECHARGE_DURATION						338		// [UINT_8]										(R/W)
// ^ 0x00-0.1sec, 0x01-0.2sec, 0x02-0.5sec, 0x03-1sec, 0x04-2sec, 0x05-3sec, 0x06-4sec, 0x07-5sec

#define TEMPERATURE_SENSOR_TYPE					339		// [UINT_8]										(R/W)
// ^ 0x00-Dual 10K NTC, 0x01-Multipoint Active Sensor

#define BMS_OPERATION_MODE						340		// [UINT_8] 									(R/W)
// ^ 0x00-Dual Port Operation, 0x01-Single Port Operation

#define SINGLE_PORT_SWITCH_TYPE					341		// [UINT_8] 									(R/W)
// ^ 0x00-FET, 0x01-AIDO1, 0x02-AIDO2, 0x03-DIDO1, 0x04-DIDO2, 0x05-AIHO1 Active Low,
// ^ 0x06-AIHO1 Active High, 0x07-AIHO2 Active Low, 0x08-AIHO2 Active High

#define BROADCAST_TIME							342		// [UINT_8] [1200 to 4500] 1mV Resolution		(R/W)
// ^ 0x00-Disabled, 0x01-0.1sec, 0x02-0.2sec, 0x03-0.5sec, 0x04-1sec, 0x05-2sec, 0x06-5sec, 0x07-10sec

#define PROTOCOL								343		// [UINT_8] 									(R/W)
// ^ 0x00-CA V3, 0x01-ASCII, 0x02-SOC BAR

/* RESERVED 344-399 */

/*********** Tiny BMS Version Data ***********/
#define HARDWARE_VER							500		// [UINT_16]									(R)
// ^ Hardware Version [8 bits LSB] | Hardware Changes Version [8 bits MSB]

#define PR_FIRMWARE_VER_BPT_BCS					501		// [UINT_16]									(R)
// ^ Public Release Firmware Version [8 bits LSB] | BPT (1 bit :8)* | BCS (2 bits :9-10)** | Reserved :11-15
// ^  *BPT - BMS Power Type:          00-Low Power, 01-High Power
// ^ **BCS - BMS Current Sensor Used: 00-Internal Resistor, 01-Internal HALL, 02-External

#define INTERNAL_FIRMWARE_VER					502 	// [UINT_16] 									(R)
#define BOOTLOADER_PROFILE_VER					503 	// [UINT_16]									(R)
// ^ Bootloader Version [8 bits LSB]  |  Profile Version [8 bits MSB]

#define PRODUCT_SERIAL_NUMBER					504 	// [UINT_16] * 6 = [96 bits] 					(R)
// ^ Word2: 505 | Word3: 506 | Word4: 507 | Word5 :508 | Word6: 509

/* RESERVED 510-599 */

/**************************** Tiny BMS Events Messages ****************************/

/*********** Tiny BMS Fault Messages ***********/
//Fault ID (0x01 - 0x30)
#define FAULT_UNDERVOLTAGE								0x02  //Under-Voltage Cutoff Occurred
#define FAULT_OVERVOLTAGE								0x03  //Over-Voltage Cutoff Occurred
#define FAULT_OVERTEMP									0x04  //Over-Temperature Cutoff Occurred
#define FAULT_DISCHARGING_OVERCURRENT					0x05  //Discharging Over-Current Cutoff Occurred
#define FAULT_CHARGING_OVERCURRENT						0x06  //Charging Over-Current Cutoff Occurred
#define FAULT_REGEN_OVERCURRENT							0x07  //Regeneration Over-Current Cutoff Occurred
#define FAULT_LOWTEMP									0x0A  //Low Temperature Cutoff Occurred
#define FAULT_CHARGER_SWITCH_ERROR						0x0B  //Charger Switch Error Detected
#define FAULT_LOAD_SWITCH_ERROR							0x0C  //Load Switch Error Detected
#define FAULT_SINGLE_PORT_SWITCH_ERROR					0x0D  //Single Port Switch Error Detected
#define FAULT_EXTERNAL_CURRENT_SENSOR_DISCONNECTED		0x0E  //External Current Sensor Disconnected (BMS Restart Required)
#define FAULT_EXTERNAL_CURRENT_SENSOR_CONNECTED			0x0F  //External Current Sensor Connected (BMS Restart Required)

/*********** Tiny BMS Warning Messages ***********/
//Warning ID (0x31 - 0x60)
#define WARNING_FULLYDISCHARGED							0x31  //Fully Discharged Cutoff Occurred
#define WARNING_LOW_TEMP_CHARGING						0x37  //Low Temperature Charging Cutoff Occurred
#define WARNING_CHARGING_DONE_TOOHIGH					0x38  //Charger Voltage Too High
#define WARNING_CHARGING_DONE_TOOLOW					0x39  //Charger Voltage Too Low

/*********** Tiny BMS Information Messages ***********/
//Info ID (0x61 - 0x90)
#define INFO_SYSTEM_STARTED								0x61  //System Started
#define INFO_CHARGING_STARTED							0x62  //Charging Started
#define INFO_CHARGING_DONE								0x63  //Charging Done
#define INFO_CHARGER_CONNECTED							0x64  //Charger Connected
#define INFO_CHARGER_DISCONNECTED						0x65  //Charger Disconnected
#define INFO_DUAL_PORT_OPERATION_MODE					0x66  //Dual Port Operation Mode Activated
#define INFO_SINGLE_PORT_OPERATION_MODE					0x67  //Single Port Operation Mode Activated
#define INFO_OVERTEMP_FAULT_RECOVERY					0x73  //Recovered From Over-Temperature Fault Condition
#define INFO_LOWTEMP_WARNING_RECOVERY					0x74  //Recovered From Low Temperature Warning Condition
#define INFO_LOWTEMP_FAULT_RECOVERY						0x75  //Recovered From Low Temperature Fault Condition
#define INFO_CHARGING_OVERCURRENT_FAULT_RECOVERY		0x76  //Recovered From Charging Over-Current Fault Condition
#define INFO_DISCHARGING_OVERCURRENT_FAULT_RECOVERY		0x77  //Recovered From Discharging Over-Current Fault Condition
#define INFO_REGEN_OVERCURRENT_FAULT_RECOVERY			0x78  //Recovered From Regeneration Over-Current Fault Condition
#define INFO_OVERVOLTAGE_FAULT_RECOVERY					0x79  //Recovered From Over-Voltage Fault Condition
#define INFO_FULLYDISCHARGED_VOLTAGE_WARNING_RECOVERY	0x7A  //Recovered From Fully Discharged Voltage Warning Condition
#define INFO_UNDERVOLTAGE_FAULT_RECOVERY				0x7B  //Recovered From Under-Voltage Fault Condition
#define INFO_EXTERNAL_CURRENT_SENSOR_CONNECTED			0x7C  //External Current Sensor Connected
#define INFO_EXTERNAL_CURRENT_SENSOR_DISCONNECTED		0x7D  //External Current Sensor Disconnected

/*********** CRC Table ***********/
const static uint16_t crcTable[256]={
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/* TinyBMS UART API Function Prototypes */
uint8_t TinyBMS_UART_ACK(UART_HandleTypeDef *huart);
uint8_t TinyBMS_UART_ReadRegBlock(UART_HandleTypeDef *huart, uint8_t rl, uint16_t addr);
uint8_t TinyBMS_UART_ReadRegIndividual(UART_HandleTypeDef *huart, uint8_t pl, uint16_t addr[]);
uint8_t TinyBMS_UART_WriteRegBlock(UART_HandleTypeDef *huart, uint8_t pl, uint16_t addr, uint16_t data[]);
uint8_t TinyBMS_UART_WriteRegIndividual(UART_HandleTypeDef *huart, uint8_t pl, uint16_t addr[], uint16_t data[]);
uint8_t TinyBMS_UART_ReadRegBlockMODBUS(UART_HandleTypeDef *huart, uint16_t addr, uint8_t rl);
uint8_t TinyBMS_UART_WriteRegBlockMODBUS(UART_HandleTypeDef *huart, uint16_t addr, uint8_t rl, uint8_t pl, uint16_t data[]);

uint8_t TinyBMS_UART_ResetClearEventsStatistics(UART_HandleTypeDef *huart, uint8_t option);
uint8_t TinyBMS_UART_ReadNewestEvents(UART_HandleTypeDef *huart);
uint8_t TinyBMS_UART_ReadAllEvents(UART_HandleTypeDef *huart);
float TinyBMS_UART_ReadBatteryPackVoltage(UART_HandleTypeDef *huart);
float TinyBMS_UART_ReadBatteryPackCurrent(UART_HandleTypeDef *huart);
uint16_t TinyBMS_UART_ReadBatteryPackMaxCellVoltage(UART_HandleTypeDef *huart);
uint16_t TinyBMS_UART_ReadBatteryPackMinCellVoltage(UART_HandleTypeDef *huart);
uint16_t TinyBMS_UART_ReadOnlineStatus(UART_HandleTypeDef *huart);
uint32_t TinyBMS_UART_ReadLifetimeCounter(UART_HandleTypeDef *huart);
uint32_t TinyBMS_UART_ReadEstimatedSOCValue(UART_HandleTypeDef *huart);
uint8_t TinyBMS_UART_ReadDeviceTemperatures(UART_HandleTypeDef *huart);
uint8_t TinyBMS_UART_ReadBatteryPackCellVoltages(UART_HandleTypeDef *huart);
uint8_t TinyBMS_UART_ReadSettingsValues(UART_HandleTypeDef *huart, uint8_t option, uint8_t rl);
uint8_t TinyBMS_UART_ReadVersion(UART_HandleTypeDef *huart);
uint8_t TinyBMS_UART_ReadVersionExtended(UART_HandleTypeDef *huart);
uint8_t TinyBMS_UART_ReadCalcSpeedDistanceLeftEstTimeLeft(UART_HandleTypeDef *huart);

/* TinyBMS CAN API Function Prototypes */
uint8_t TinyBMS_CAN_ResetClearEventsStatistics(CAN_HandleTypeDef *hcan, uint8_t option);
uint8_t TinyBMS_CAN_ReadRegBlock(CAN_HandleTypeDef *hcan, uint8_t rl, uint16_t addr);
uint8_t TinyBMS_CAN_WriteRegBlock(CAN_HandleTypeDef *hcan, uint8_t rl, uint16_t addr, uint16_t data[]);
uint8_t TinyBMS_CAN_ReadNewestEvents(CAN_HandleTypeDef *hcan);
uint8_t TinyBMS_CAN_ReadAllEvents(CAN_HandleTypeDef *hcan);
float TinyBMS_CAN_ReadBatteryPackVoltage(CAN_HandleTypeDef *hcan);
float TinyBMS_CAN_ReadBatteryPackCurrent(CAN_HandleTypeDef *hcan);
uint16_t TinyBMS_CAN_ReadBatteryPackMaxCellVoltage(CAN_HandleTypeDef *hcan);
uint16_t TinyBMS_CAN_ReadBatteryPackMinCellVoltage(CAN_HandleTypeDef *hcan);
uint16_t TinyBMS_CAN_ReadOnlineStatus(CAN_HandleTypeDef *hcan);
uint32_t TinyBMS_CAN_ReadLifetimeCounter(CAN_HandleTypeDef *hcan);
uint32_t TinyBMS_CAN_ReadEstimatedSOCValue(CAN_HandleTypeDef *hcan);
uint8_t TinyBMS_CAN_ReadDeviceTemperatures(CAN_HandleTypeDef *hcan);
uint8_t TinyBMS_CAN_ReadBatteryPackCellVoltages(CAN_HandleTypeDef *hcan);
uint8_t TinyBMS_CAN_ReadSettingsValues(CAN_HandleTypeDef *hcan, uint8_t option, uint8_t rl);
uint8_t TinyBMS_CAN_ReadVersion(CAN_HandleTypeDef *hcan);
uint8_t TinyBMS_CAN_ReadCalcSpeedDistanceLeftEstTimeLeft(CAN_HandleTypeDef *hcan);
uint8_t TinyBMS_CAN_ReadNodeID(CAN_HandleTypeDef *hcan);
uint8_t TinyBMS_CAN_WriteNodeID(CAN_HandleTypeDef *hcan, uint8_t nodeID);

#endif /* INC_TINYBMS_H_ */
