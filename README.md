# SHL_Pod_SecondaryBMS
### A library of supported API for the Secondary BMS: *Energus TinyBMS s516-150A*


## Connections / Wiring
**STM32 Nucleo-F746ZG**    <->  **CAN-UART Converter**   <->  **Energus TinyBMS s516-150A**


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
TBD
