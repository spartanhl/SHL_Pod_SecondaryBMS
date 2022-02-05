# SHL_Pod_SecondaryBMS
### STM32 Nucleo-F746xx test environment with a library of supported API for the Secondary BMS: *Energus TinyBMS s516-150A*


## Connections / Wiring
**STM32 Nucleo-F746ZG**    <-**UART**->   **Energus TinyBMS s516-150A** \
							  or \
**STM32 Nucleo-F746ZG**   <-**CANBUS**->  **Energus TinyBMS s516-150A**


## Battery Pack Configuration
**Secondary Pack Characteristics** \
24V 50Ah 15C (7S 20P)  -- *Specced for a minimum of 2 hours runtime @ continuous load.*

**Individual Cell**  \
*Samsung INR18650-25R* - Lithium Ion "18650" cylindrical cell

**Electrical Ratings** \
I_cell_rating = 20A_ctns,   I_pack_rating = 400A_ctns \
V_cell_empty = 3.0V,   V_cell_nominal = 3.6V,   V_cell_full = 4.2V \
V_pack_empty = 21.0V,   V_pack_nominal = 25.2V,   V_pack_full = 29.4V

**Module Construction** \
Each 'super-cell', in actuality, is a '10x2' (10 by 2) module which is comprised of 20 '18650' cells connected in parallel, but constructed as 2 rows of 10 cells.

**Pack Configuration** \
The secondary battery pack is 7 of these 'super-cells' connected in series.


## Supported API
- UART API Testing:
	- 1.1.1 		TinyBMS_UART_ACK
	- 1.1.2 		TinyBMS_UART_ReadRegBlock
	- 1.1.3 		TinyBMS_UART_ReadRegIndividual
	- 1.1.4 		TinyBMS_UART_WriteRegBlock
	- 1.1.5 		TinyBMS_UART_WriteRegIndividual
	- 1.1.6 		TinyBMS_UART_ReadRegBlockMODBUS
	- 1.1.7 		TinyBMS_UART_WriteRegBlockMODBUS
	- 1.1.8 		TinyBMS_UART_ResetClearEventsStatistics
	- 1.1.9 		TinyBMS_UART_ReadNewestEvents
	- 1.1.10 		TinyBMS_UART_ReadAllEvents
	- 1.1.11 		TinyBMS_UART_ReadBatteryPackVoltage
	- 1.1.12 		TinyBMS_UART_ReadBatteryPackCurrent
	- 1.1.13 		TinyBMS_UART_ReadBatteryPackMaxCellVoltage
	- 1.1.14 		TinyBMS_UART_ReadBatteryPackMinCellVoltage
	- 1.1.15 		TinyBMS_UART_ReadOnlineStatus
	- 1.1.16 		TinyBMS_UART_ReadLifetimeCounter
	- 1.1.17 		TinyBMS_UART_ReadEstimatedSOCValue
	- 1.1.18 		TinyBMS_UART_ReadDeviceTemperatures
	- 1.1.19 		TinyBMS_UART_ReadBatteryPackCellVoltages
	- 1.1.20 		TinyBMS_UART_ReadSettingsValues
	- 1.1.21 		TinyBMS_UART_ReadVersion
	- 1.1.22 		TinyBMS_UART_ReadVersionExtended
	- 1.1.23 		TinyBMS_UART_ReadCalcSpeedDistanceLeftEstTimeLeft
 
- CAN API Testing:
	- 2.1.1 		TinyBMS_CAN_ResetClearEventsStatistics
	- 2.1.2 		TinyBMS_CAN_ReadRegBlock
	- 2.1.3 		TinyBMS_CAN_WriteRegBlock
	- 2.1.4 		TinyBMS_CAN_ReadNewestEvents
	- 2.1.5 		TinyBMS_CAN_ReadAllEvents
	- 2.1.6 		TinyBMS_CAN_ReadBatteryPackVoltage
	- 2.1.7 		TinyBMS_CAN_ReadBatteryPackCurrent
	- 2.1.8 		TinyBMS_CAN_ReadBatteryPackMaxCellVoltage
	- 2.1.9 		TinyBMS_CAN_ReadBatteryPackMinCellVoltage
	- 2.1.10 		TinyBMS_CAN_ReadOnlineStatus
	- 2.1.11 		TinyBMS_CAN_ReadLifetimeCounter
	- 2.1.12 		TinyBMS_CAN_ReadEstimatedSOCValue
	- 2.1.13 		TinyBMS_CAN_ReadDeviceTemperatures
	- 2.1.14 		TinyBMS_CAN_ReadBatteryPackCellVoltages
	- 2.1.15 		TinyBMS_CAN_ReadSettingsValues
	- 2.1.16 		TinyBMS_CAN_ReadVersion
	- 2.1.17 		TinyBMS_CAN_ReadCalcSpeedDistanceLeftEstTimeLeft
	- 2.1.18 		TinyBMS_CAN_ReadNodeID
	- 2.1.19 		TinyBMS_CAN_WriteNodeID 
