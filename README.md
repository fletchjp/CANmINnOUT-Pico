<img align="right" src="arduino_cbus_logo.png"  width="150" height="75">

# CANmINnOUT

A version of the Arduino IDE program for the Raspberry Pi Pico to allocate all available
device pins as either switch input or LED output. To this end, the number of input pins
is regarded as 'm' and the number of output pins as 'n'.

Key Features:
- MERG CBUS interface.
- Uses the Raspberry Pi Pico Arduino core for all RP2040 boards by Earle Philhower
  https://github.com/earlephilhower/arduino-pico
- Uses the Arduino library wrapper for can2040 by Duncan Greenwood
  https://github.com/obdevel/ACAN2040
- Uses Software CAN bus implementation for rp2040 micro-controllers by Kevin O'Connor
  https://github.com/KevinOConnor/can2040 
- Uses both cores of the Pico.
- LED flash rate selectable from event variables.
- Switch function controllable by node variables.
- Modular construction to ease adaptation to your application.
- Start-of-Day reporting selectable from event variables.

## Overview

The program is written in C++ but you do not need to understand this to use the program.
The program includes a library that manages the LED functionality.
The program allows for the inclusion of a CBUS setup switch or LEDs or not as the user
requires. 

Even if they are not used, CBUSSwitch and CBUSLED libraries must be available for access
by the CBUS and CBUSConfig libraries for reason of backwards library compatibility. 

To ensure compatability of the CBUS library suite, only use the latest version 
produced by Duncan Greenwood.

## Using CANmINnOUT

Two pins are required for interface to a suitable CAN Bus Transceiver (e.g. MCP2562).
Three pins have also been reserved for the CBUS Switch and LEDs. Thus, the total number of
pins available for input or output is 21 or 24 if the CBUS Switch and LEDs are not used.

The total of input and output pins (m + n) cannot exceed these numbers.

**It is the users responsibility that the total current that the Pico is asked to supply 
stays within the capacity of the on board regulator.  Failure to do this will result in 
terminal damage to your Pico.**

Pins defined as inputs are active low.  That is to say that they are pulled up by an 
internal resistor. The input switch should connect the pin to 0 Volts.

Pins defined as outputs are active high.  They will source current to (say) an LED. It is 
important that a suitable current limiting resistor is fitted between the pin and the LED 
anode.  The LED cathode should be connected to ground.

Both cores of the RP2040 are used.  Core 0 is used to manage all aspects of the CAN/CBUS.
Core 1 is used for the application.

### Library Dependencies

The following third party libraries are required:

Library | Purpose
---------------|-----------------
Streaming.h     |*C++ stream style output, v5, (http://arduiniana.org/libraries/streaming/)*
Bounce2.h       |*Debounce of switch inputs*
ACAN2040.h      |*library to support the MCP2515/25625 CAN controller IC*
CBUSACAN2040.h  |*CAN controller and CBUS class*
CBUSconfig.h    |*module configuration*
CBUS.h          |*CBUS Class*
cbusdefs.h      |*Definition of CBUS codes*
CBUSParams.h    |*Manage CBUS parameters*
CBUSSwitch.h    |*library compatibility*
CBUSLED.h       |*library compatibility*

### Application Configuration

The module can be configured to the users specific configuration in a section of code 
starting at line 117 with the title DEFINE MODULE. The following parameters can be changed 
as necessary:
```
#define DEBUG 0       // set to 0 for no serial debug
```
This define at circa line 73 allows various output reports to be made to the serial monitor 
for use in debugging.  To enable these, change the value of DEBUG from 0 to 1.

```
// module name
unsigned char mname[7] = { 'm', 'I', 'N', 'n', 'O', 'U', 'T' };
```
This can be adjusted as required to reflect the module configuration.  For example, 'm' & 'n' 
could be changed to any number between 0 & 9. However, only one character is allowed between 
each pair of ' ' and the total number of characters must not exceed seven.

```
//Module pins available for use are any of the 21 pins marked as "Not Used" in the Pin Use Map.
const byte LED[] = {8, 7};        // Example LED pin connections through typ. 1K8 resistor
const byte SWITCH[] = {9, 6};     // Example Module Switch takes input to 0V.
```
Insert the pin numbers being used for inputs and outputs between the appropriate pair of braces.
Pin numbers must be seperated by a comma.

### CBUS Op Codes

The following Op codes are supported:

OP_CODE | HEX | Function
----------|---------|---------
 OPC_ACON | 0x90 | On event
 OPC_ACOF | 0x91 | Off event
 OPC_ASON | 0x98 | Short event on
 OPC_ASOF | 0x99 | Short event off

### Event Variables

Event Variables control the action to take when an event is received.
The number of Event Variables (EV) is equal to the number of LEDs plus 1.

The first Event Variable (EV1) controls Start-of-Day reporting. Set EV1 to 1 to enable 
reporting of state of the switches.

The remaining Event Variables control the LEDs. 
Event Variable 2 (EV2) controls the first LED pin in the ```LED``` array. 
EV3 controls the second LED pin, etc.

The following EV values are defined to control the LEDs:

 EV Value | Function
--------|-----------
 0 | LED off
 1 | LED on
 2 | LED flash at 500mS
 3 | LED flash at 250mS
 
### Node Variables

Node Variables control the action to take when a switch is pressed or depressed.

The number of Node Variables (NV) is equal to the number of switches.
NV1 corresponds to the first pin defined in the array ```SWITCH```, 
NV2 corresponds to the second pin in that array, etc.

The following NV values define input switch function:

NV Value | Function
--------|--------
 0 | On/Off switch
 1 | On only push button
 2 | Off only push button
 3 | On/Off toggle push button
 
### Start Of Day

The module will respond to a Start Of Day event if that event has been taught and EV1 is set to 1.
Upon receipt of this SoD message, the unit will report the status of each switch that has an NV value
of 0 or 3. Clearly, "on" only and "off" only switch NV values are of no relevance to SoD.

There is no restriction on setting other EVs to cause LEDs to indicate receipt of an SoD event in
addition to setting EV1.
 
## Set Up and the Serial Monitor

The option to not include the CBUS switch and LEDs is chosen simply by not allocating pins for
these components. It is then necessary to have another means of registering the module on 
the CBUS and acquiring a node number.  This is accomplished by sending a single character to 
the Pico using the serial send facility in the serial monitor of the Arduino IDE (or similar).
Note that this method can be used even if the CBUS Switch is present.

#### 'r'
This character will cause the module to renegotiate its CBUS status by requesting a node number.
The FCU will respond as it would for any other unrecognised module.

#### 'z'
This character needs to be sent twice within 2 seconds so that its action is confirmed.
This will reset the module and clear the EEPROM.  It should thus be used with care.

Other information is available using the serial monitor using other commands:

#### 'n'
This character will return the node configuration.

#### 'e'
This character will return the learned event table in the EEPROM.

#### 'v'
This character will return the node variables.

#### 'c'
This character will return the CAN bus status.

#### 'h'
This character will return the event hash table.

#### 'y'
This character will reset the CAN bus and CBUS message processing.

#### '\*'
This character will reboot the module.

#### 'm'
This character will return the amount of free memory. 
 
 
 
 
 
