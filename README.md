# SHL_Pod_SecondaryBMS
### STM32 Nucleo-F746xx application with a library of supported commands for the Secondary BMS: *Energus TinyBMS s516-150A*

Low Current Application (150A_peak discharging, 30A_peak charging) 

## Connections / Wiring
**STM32 Nucleo-F746ZG**    <-**UART**->   **Energus TinyBMS s516-150A** \
							  or \
**STM32 Nucleo-F746ZG**   <-**CAN-UART**->  **Energus TinyBMS s516-150A**

ElCon Charger (CAN-enabled), TinyBMS (via CAN-UART converter), and STM32F7 communication via CAN2.0b (extended ID)

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
Each 'super-cell' is a '10x2' (10 by 2) module which is comprised of 20 '18650' cells connected in parallel, but arranged in 2 rows of 10 cells.

**Pack Configuration** \
The secondary battery pack is 7 of these 'super-cells' connected in series.

## BMS Settings Registers Configuration
| Settings Register					|	Value		|
| :--- 								|	:----:		|
| Fully Charged Voltage: 			|4000mV			|
| Fully Discharged Voltage: 		|3000mV			|
| Early Balancing Threshold: 		|3200mV			|
| Charge Finished Current:			|1000mA			|
| Battery Capacity:					|5000mAh		|
| Number of Series Cells:			|7				|
| Allowed Disbalance:				|15mV			|
| Pulses Per Unit:					|1				|
| Distance Unit Name:				|km				|
| Over-Voltage Cutoff:				|4200mV			|
| Under-Voltage Cutoff:				|2900mV			|
| Discharge Over-Current Cutoff:	|60A			|
| Charge Over-Current Cutoff:		|30A			|
| Over-Temp Cutoff:					|60C			|
| Low Temperature Charger Cutoff:	|1C				|
| Charger Type:						|CAN			|
| Load Switch Type:					|FET			|
| Automatic Recovery:				|5s				|
| Charger Switch Type:				|Charge FET		|
| Ignition:							|Disabled		|
| Charger Detection:				|Internal		|
| Speed Sensor Input:				|Disabled		|
| Precharge Pin:					|Disabled		|
| Precharge Duration:				|100ms			|
| Temperature Sensor Type:			|Dual 10K NTC	|
| BMS Operation Mode:				|Dual Port		|
| Single Port Switch Type:			|N/A			|
| Broadcast Time:					|Disabled		|
| Protocol:							|CAV3			|

## Application Menu
    1: UART Command Test
    2: CAN Command Test
    3: Monitor Charging (UART)
    4: Monitor Discharging (UART)
    5: Monitor Charging (CAN)
    6: Monitor Discharging (CAN)
    7: Quit

## Supported Commands
- UART Command Testing:
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
 
- CAN Command Testing:
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

## Directory Contents
```
**SHL_Pod_SecondaryBMS**
|
└───Includes 
└───Core 
|   └───Inc 
|   |   | - it.h
|   |   | - main.h
|   |   | - stm32f7xx_hal_conf.h
|   |   | - TinyBMS.h
|   |
|   └───Src
|   |   | - it.c
|   |   | - main.c
|   |   | - msp.c
|   |   | - syscalls.c
|   |   | - sysmem.c
|   |   | - system_stm32f7xx.c
|   |   | - TinyBMS.c
|   |
|   └───Startup
|       | - startup_stm32f746zgtx.s
|
└───Drivers
|   └───CMSIS
|   |	└───Device
|   |	|   └───ST
|   |	|       └───STM32F7xx
|   |	|           └───Include
|   |	└───Include
|   └───STM32F7xx_HAL_Driver
|   	└───Inc
|   	└───Src
|
└───Debug 
└───Reference Docs
    └───ARM & STM32 Docs
    └───Screenshots
    └───Traces
```
