
// CANmINnOUT
// Version for use with Raspberry Pi Pico with software CAN Controller.
// This uses both cores of the RP2040

/*
  Copyright (C) 2022 Martin Da Costa
  Including copyrights from CBUS_1in1out and Arduino CBUS Libraries


  This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                  and indicate if changes were made. You may do so in any reasonable manner,
                  but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                 your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                 legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

/*
      3rd party libraries needed for compilation:

      Streaming   -- C++ stream style output, v5, (http://arduiniana.org/libraries/streaming/)
      ACAN2040    -- library wrapper for can2040 RP2040 CAN driver
      CBUSSwitch  -- library access required by CBUS and CBUS module_config
      CBUSLED     -- library access required by CBUS and CBUS module_config
*/
///////////////////////////////////////////////////////////////////////////////////
// Pin Use map:
// Pin 1   GP0  Not Used
// Pin 2   GP1  Not Used
// Pin 3        0V
// Pin 4   GP2  Not Used
// Pin 5   GP3  Not Used
// Pin 6   GP4  Not Used
// Pin 7   GP5  Not Used
// Pin 8        0V
// Pin 9   GP6  Not Used
// Pin 10  GP7  Not Used
// Pin 11  GP8  CAN Rx
// Pin 12  GP9  CAN Tx
// Pin 13   0V
// Pin 14  GP10 Not Used
// Pin 15  GP11 Not Used
// Pin 16  GP12 Not Used
// Pin 17  GP13 Request Switch (CBUS)
// Pin 18       0V
// Pin 19  GP14 CBUS green SLiM LED pin
// Pin 20  GP15 CBUS yellow FLiM LED pin
// Pin 21  GP16 Not Used
// Pin 22  GP17 Not Used
// Pin 23       0V
// Pin 24  GP18 Not Used
// Pin 25  GP19 Not Used
// Pin 26  GP20 Not Used
// Pin 27  GP21 Not Used
// Pin 28       0V
// Pin 29  GP22 
// Pin 30  RUN Reset (active low)
// Pin 31  GP26 ADC0 Not Used
// Pin 32  GP27 ADC1 Not Used
// Pin 33       0V
// Pin 34  GP28 ADC2 Not Used
// Pin 35   ADC_VREF
// Pin 36   3V3
// Pin 37   3V3_EN
// Pin 38   0V
// Pin 39   VSYS
// Pin 40   VBUS
//////////////////////////////////////////////////////////////////////////

#define DEBUG 0       // set to 0 for no serial debug

#if DEBUG
#define DEBUG_PRINT(S) Serial << S << endl
#else
#define DEBUG_PRINT(S)
#endif

// 3rd party libraries
#include <Streaming.h>
#include <Bounce2.h>

// Module library header files
#include "LEDControl.h"
// CBUS library header files
#include <CBUSACAN2040.h>           // CAN controller and CBUS class
#include <CBUSswitch.h>             // pushbutton switch
#include <CBUSLED.h>                // CBUS LEDs
#include <CBUSconfig.h>             // module module_configuration
#include <CBUSParams.h>             // CBUS parameters
#include <cbusdefs.h>               // MERG CBUS constants

////////////DEFINE MODULE/////////////////////////////////////////////////

// module name
unsigned char mname[7] = { 'm', 'I', 'N', 'n', 'O', 'U', 'T' };

// constants
const byte VER_MAJ = 2;         // code major version
const char VER_MIN = 'b';       // code minor version
const byte VER_BETA = 0;        // code beta sub-version
const byte MODULE_ID = 99;      // CBUS module type

const byte LED_GRN = 14;             // CBUS green SLiM LED pin
const byte LED_YLW = 15;             // CBUS yellow FLiM LED pin
const byte SWITCH0 = 13;            // CBUS push button switch pin

const int GLOBAL_EVS = 1;        // Number event variables for the module
// EV1 - StartOfDay

// Variables for SOD state reporting event dispatching.
// They indicate which switch (index of) to report next and at what time to send the next event.
int nextSodSwitchIndex = -1;  // An index of -1 indicates that there is no SOD reporting going on.
unsigned int nextSodMessageTime = 0;
const unsigned int SOD_INTERVAL = 20;    // milliseconds between SOD events.

//Module pins available for use are any of the 20 pins marked as "Not Used" in the Pin Use Map.
const byte LED[] = {22, 26};      // Example LED pin connections through typ. 1K8 resistor
const byte SWITCH[] = {18, 19};   // Example Module Switch takes input to 0V.

const bool active = 0; // 0 is for active low LED drive. 1 is for active high

const int NUM_LEDS = sizeof(LED) / sizeof(LED[0]);
const int NUM_SWITCHES = sizeof(SWITCH) / sizeof(SWITCH[0]);

// CBUS objects
CBUSConfig module_config;           // module_configuration object
CBUSACAN2040 CBUS(&module_config);  // CBUS object
CBUSLED ledGrn, ledYlw;             // two LED objects
CBUSSwitch pb_switch;               // switch object

// module objects
Bounce moduleSwitch[NUM_SWITCHES];  //  switch as input
LEDControl moduleLED[NUM_LEDS];     //  LED as output
byte switchState[NUM_SWITCHES];

//////////////////////////////////////////////////////////////////////////
//
///  setup CBUS - runs once at power on called from setup()
//
void setupCBUS(){
  // set module_config layout parameters
  module_config.EE_NVS_START = 10;
  module_config.EE_NUM_NVS = NUM_SWITCHES;
  module_config.EE_EVENTS_START = 40;
  module_config.EE_MAX_EVENTS = 20;
  module_config.EE_NUM_EVS = GLOBAL_EVS + NUM_LEDS;
  module_config.EE_BYTES_PER_EVENT = (module_config.EE_NUM_EVS + 4);

  // initialise and load module_configuration
  module_config.setEEPROMtype(EEPROM_INTERNAL);
  module_config.begin();

  Serial << get_core_num() << F("> mode = ") << ((module_config.FLiM) ? "FLiM" : "SLiM") << F(", CANID = ") << module_config.CANID;
  Serial << get_core_num() << F(", NN = ") << module_config.nodeNum << endl;

  // show code version and copyright notice
  printConfig();

  // set module parameters
  CBUSParams params(module_config);
  params.setVersion(VER_MAJ, VER_MIN, VER_BETA);
  params.setModuleId(MODULE_ID);
  params.setFlags(PF_FLiM | PF_COMBI);

  // assign to CBUS
  CBUS.setParams(params.getParams());
  CBUS.setName(mname);
  
  // set CBUS LED pins and assign to CBUS
  ledGrn.setPin(LED_GRN);
  ledYlw.setPin(LED_YLW);
  CBUS.setLEDs(ledGrn, ledYlw);

  // initialise CBUS switch and assign to CBUS
  pb_switch.setPin(SWITCH0, LOW);
  pb_switch.run();
  CBUS.setSwitch(pb_switch);

  // module reset - if switch is depressed at startup and module is in SLiM mode
  if (pb_switch.isPressed() && !module_config.FLiM) {
    Serial << F("> switch was pressed at startup in SLiM mode") << endl;
    module_config.resetModule(ledGrn, ledYlw, pb_switch);
  }

  // opportunity to set default NVs after module reset
  if (module_config.isResetFlagSet()) {
    Serial << F("> module has been reset") << endl;
    module_config.clearResetFlag();
  }

  // register our CBUS event handler, to receive event messages of learned events
  CBUS.setEventHandler(eventhandler);
  
  // set CBUS LEDs to indicate mode
  CBUS.indicateMode(module_config.FLiM);

  // module_configure and start CAN bus and CBUS message processing
  CBUS.setNumBuffers(16, 4);         // more buffers = more memory used, fewer = less
  CBUS.setPins(1, 0);           // select pins for CAN Tx & Rx
  
  if (!CBUS.begin()) {
    Serial << get_core_num() << F("> can2040 init fail") << endl;
  } else {
    Serial << get_core_num() << F("> can2040 init ok") << endl;
  }
}
//
///  setup Module - runs once at power on called from setup()
//

void setupModule(){
  // module_configure the module switches, active low
  for (int i = 0; i < NUM_SWITCHES; i++)  {
    moduleSwitch[i].attach(SWITCH[i], LOW);
	moduleSwitch[i].interval(5);
	switchState[i] = false;
  }

  // module_configure the module LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    moduleLED[i].setPin(LED[i], active); //Second arguement sets 0 = active low, 1 = active high. Default if no second arguement is active high.
	moduleLED[i].off();
  }

  Serial << get_core_num() << "> Module has " << NUM_LEDS << " LEDs and " << NUM_SWITCHES << " switches." << endl;
}


void setup(){
  Serial.begin (115200);
  delay(2000);
  Serial << endl << endl << get_core_num() << F("> ** CBUS m in n out v1 ** ") << __FILE__ << endl;

  setupCBUS();

  // end of setup
  DEBUG_PRINT(get_core_num() << F("> ready"));
  delay(20);
}

void setup1() {
  delay(2010);
  setupModule();

  // end of setup
  DEBUG_PRINT(get_core_num() << F("> Module ready"));
  delay(25);
}

void loop() {
  bool isSuccess = true;
  // do CBUS message, switch and LED processing
  CBUS.process();

  // process console commands
  processSerialInput();

  processStartOfDay();
  
   if (rp2040.fifo.available() == 2) {
    uint8_t sendData1 = (uint8_t)rp2040.fifo.pop();
    uint16_t sendData2 = (uint16_t)rp2040.fifo.pop();
    isSuccess = sendEvent(sendData1, sendData2);
  }

  if (!isSuccess) {
    DEBUG_PRINT(get_core_num() << F("> One of the send message events failed"));
  }
}

void loop1() {
	 if (rp2040.fifo.available() > 0) {
    byte msgLen = rp2040.fifo.pop();
// Ensure Core 0 has loaded complete message into FIFO
     while((msgLen + 1) != rp2040.fifo.available()){
  }
    receivedData(msgLen);  // Action FIFO contents
  }

  // Run the LED code
  for (int i = 0; i < NUM_LEDS; i++) {
    moduleLED[i].run();
  }

  // test for switch input
  processSwitches();
}

void processSwitches(void){
  bool isSuccess = true;
  for (int i = 0; i < NUM_SWITCHES; i++){
    moduleSwitch[i].update();
    if (moduleSwitch[i].changed()){
      byte nv = i + 1;
      byte nvval = module_config.readNV(nv);
      byte opCode;

      DEBUG_PRINT(get_core_num() << F("> Button ") << i << F(" state change detected"));
      Serial << get_core_num() << F ("> NV = ") << nv << F(" NV Value = ") << nvval << endl;

      switch (nvval){
        case 0:
          // ON and OFF
          opCode = (moduleSwitch[i].fell() ? OPC_ACON : OPC_ACOF);
          DEBUG_PRINT(get_core_num() << F("> Button ") << i
              << (moduleSwitch[i].fell() ? F(" pressed, send 0x") : F(" released, send 0x")) << _HEX(opCode));
           rp2040.fifo.push(opCode);
          rp2040.fifo.push(i + 1);
          break;

        case 1:
          // Only ON
          if (moduleSwitch[i].fell()) {
            opCode = OPC_ACON;
            DEBUG_PRINT(get_core_num() << F("> Button ") << i << F(" pressed, send 0x") << _HEX(OPC_ACON));
             rp2040.fifo.push(opCode);
          rp2040.fifo.push(i + 1);
          }
          break;

        case 2:
          // Only OFF
          if (moduleSwitch[i].fell()) {
            opCode = OPC_ACOF;
            DEBUG_PRINT(get_core_num() << F("> Button ") << i << F(" pressed, send 0x") << _HEX(OPC_ACOF));
             rp2040.fifo.push(opCode);
          rp2040.fifo.push(i + 1);
          }
          break;

        case 3:
          // Toggle button
          if (moduleSwitch[i].fell()) {
            switchState[i] = !switchState[i];
            opCode = (switchState[i] ? OPC_ACON : OPC_ACOF);
            DEBUG_PRINT(get_core_num() << F("> Button ") << i
                << (moduleSwitch[i].fell() ? F(" pressed, send 0x") : F(" released, send 0x")) << _HEX(opCode));
             rp2040.fifo.push(opCode);
          rp2040.fifo.push(i + 1);
          }

          break;

        default:
          DEBUG_PRINT(get_core_num() << F("> Invalid NV value."));
          break;
      }
    }
  }
  if (!isSuccess) {
    DEBUG_PRINT(get_core_num() << F("> One of the send message events failed"));
  }
}

// Send an event routine according to Module Switch
bool sendEvent(byte opCode, unsigned int eventNo) {
  CANFrame msg;
  msg.id = module_config.CANID;
  msg.len = 5;
  msg.data[0] = opCode;
  msg.data[1] = highByte(module_config.nodeNum);
  msg.data[2] = lowByte(module_config.nodeNum);
  msg.data[3] = highByte(eventNo);
  msg.data[4] = lowByte(eventNo);

  bool success = CBUS.sendMessage(&msg);
  if (success) {
    DEBUG_PRINT(get_core_num() << F("> sent CBUS message with Event Number ") << eventNo);
  } else {
    DEBUG_PRINT(get_core_num() << F("> error sending CBUS message"));
  }
  return success;
}

//
/// called from the CBUS library when a learned event is received
//
void eventhandler(byte index, CANFrame *msg) {
	rp2040.fifo.push(msg->len);
  rp2040.fifo.push(index);
  for (byte n = 0; n < msg->len; n++) {
    rp2040.fifo.push(msg->data[n]);
  }
  DEBUG_PRINT(get_core_num() << F("> event handler: index = ") << index << F(", opcode = 0x") << _HEX(msg->data[0]));
  DEBUG_PRINT(get_core_num() << F("> event handler: length = ") << msg->len);
}

void receivedData(byte length) {
	byte rcvdData[length];
  byte index = rp2040.fifo.pop();
  for (byte n = 0; n < length; n++) {
    rcvdData[n] = rp2040.fifo.pop();
  }

  byte opc = rcvdData[0];

  DEBUG_PRINT(get_core_num() << F("> event handler: index = ") << index << F(", opcode = 0x") << _HEX(rcvdData[0]));
  DEBUG_PRINT(get_core_num() << F("> event handler: length = ") << msg->len);

#if DEBUG
  unsigned int node_number = (rcvdData[1] << 8 ) + rcvdData[2];
  unsigned int event_number = (rcvdData[3] << 8 ) + rcvdData[4];
#endif

  DEBUG_PRINT(get_core_num() << F("> NN = ") << node_number << F(", EN = ") << event_number);
  DEBUG_PRINT(get_core_num() << F("> op_code = ") << opc);

  switch (opc) {
    case OPC_ACON:
    case OPC_ASON:
      for (int i = 0; i < NUM_LEDS; i++) {
        byte ev = i + 1 + GLOBAL_EVS;
        byte evval = module_config.getEventEVval(index, ev);

        switch (evval) {
          case 1:
            moduleLED[i].on();
            break;

          case 2:
            moduleLED[i].flash(500);
            break;

          case 3:
            moduleLED[i].flash(250);
            break;

          default:
            break;
        }
      }
      {
        byte sodVal = module_config.getEventEVval(index, 1);
        if (sodVal == 1) {
          if (nextSodSwitchIndex < 0) // Check if a SOD is already in progress.
          {
            nextSodSwitchIndex = 0;
            nextSodMessageTime = millis() + SOD_INTERVAL;
          }
        }
      }
      break;

    case OPC_ACOF:
    case OPC_ASOF:
      for (int i = 0; i < NUM_LEDS; i++) {
        byte ev = i + 1 + GLOBAL_EVS;
        byte evval = module_config.getEventEVval(index, ev);

        if (evval > 0) {
          moduleLED[i].off();
        }
      }
      break;
  }
}

void processStartOfDay() {
  if (nextSodSwitchIndex >= 0
      && nextSodMessageTime < millis()) {
    byte nv =  nextSodSwitchIndex + 1;
    byte nvval = module_config.readNV(nv);
    byte opCode;
    bool isSuccess = true;

    switch (nvval) {
      case 0:
        // ON and OFF
        opCode = (moduleSwitch[nextSodSwitchIndex].read() == LOW ? OPC_ACON : OPC_ACOF);
    #if DEBUG
        if (active) {
          Serial << get_core_num() << F("> SOD: Push Button ") << nextSodSwitchIndex
                 << " is " << (moduleSwitch[nextSodSwitchIndex].read() ? F("pressed, send 0x") : F(" released, send 0x")) << _HEX(opCode) << endl;
        } else {
          Serial << get_core_num() << F("> SOD: Push Button ") << nextSodSwitchIndex
                 << " is " << (moduleSwitch[nextSodSwitchIndex].read() ? F("released, send 0x") : F(" pressed, send 0x")) << _HEX(opCode) << endl;
        }
    #endif
        isSuccess = sendEvent(opCode, (nextSodSwitchIndex + 1));
        break;

      case 3:
        // Toggle button - use saved state.
        opCode = (switchState[nextSodSwitchIndex] ? OPC_ACON : OPC_ACOF);
    #if DEBUG
        if (active) {
          Serial << get_core_num() << F("> SOD: Toggle Button ") << nextSodSwitchIndex
                 << " is " << (moduleSwitch[nextSodSwitchIndex].read() ? F("pressed, send 0x") : F(" released, send 0x")) << _HEX(opCode) << endl;
        } else {
          Serial << get_core_num() << F("> SOD: Toggle Button ") << nextSodSwitchIndex
                 << " is " << (moduleSwitch[nextSodSwitchIndex].read() ? F("released, send 0x") : F(" pressed, send 0x")) << _HEX(opCode) << endl;
        }
    #endif
        isSuccess = sendEvent(opCode, (nextSodSwitchIndex + 1));
        break;
    }
    if (!isSuccess) {
      DEBUG_PRINT(F("> One of the send message events failed"));
    }    

    if (++nextSodSwitchIndex >= NUM_SWITCHES) {
      DEBUG_PRINT(F("> Done  all SOD events."));
      nextSodSwitchIndex = -1;
    } else {
      DEBUG_PRINT(F("> Prepare for next SOD event."));
      nextSodMessageTime = millis() + SOD_INTERVAL;
    }
  }
}

void printConfig(void) {
  // code version
  Serial << get_core_num() << F("> code version = ") << VER_MAJ << VER_MIN << F(" beta ") << VER_BETA << endl;
  Serial << get_core_num() << F("> compiled on ") << __DATE__ << F(" at ") << __TIME__ << F(", compiler ver = ") << __cplusplus << endl;

  // copyright
  Serial << get_core_num() << F("> © Martin Da Costa (MERG M6223) 2023") << endl;
  Serial << get_core_num() << F("> © Duncan Greenwood (MERG M5767) 2022") << endl;
  Serial << get_core_num() << F("> © John Fletcher (MERG M6777) 2021") << endl;
  Serial << get_core_num() << F("> © Sven Rosvall (MERG M3777) 2021") << endl;
}

//
/// command interpreter for serial console input
//

void processSerialInput(void)
{
  byte uev = 0;
  char msgstr[32];

  if (Serial.available()) {
    char c = Serial.read();

    switch (c) {

      case 'n':
        // node config
        printConfig();

        // node identity
        Serial << F("> CBUS node module_configuration") << endl;
        Serial << F("> mode = ") << (module_config.FLiM ? "FLiM" : "SLiM") << F(", CANID = ") << module_config.CANID << F(", node number = ") << module_config.nodeNum << endl;
        Serial << endl;
        break;

      case 'e':
        // EEPROM learned event data table
        Serial << F("> stored events ") << endl;
        Serial << F("  max events = ") << module_config.EE_MAX_EVENTS << F(" EVs per event = ") << module_config.EE_NUM_EVS << F(" bytes per event = ") << module_config.EE_BYTES_PER_EVENT << endl;

        for (byte j = 0; j < module_config.EE_MAX_EVENTS; j++) {
          if (module_config.getEvTableEntry(j) != 0) {
            ++uev;
          }
        }

        Serial << F("  stored events = ") << uev << F(", free = ") << (module_config.EE_MAX_EVENTS - uev) << endl;
        Serial << F("  using ") << (uev * module_config.EE_BYTES_PER_EVENT) << F(" of ") << (module_config.EE_MAX_EVENTS * module_config.EE_BYTES_PER_EVENT) << F(" bytes") << endl << endl;

        Serial << F("  Ev#  |  NNhi |  NNlo |  ENhi |  ENlo | ");

        for (byte j = 0; j < (module_config.EE_NUM_EVS); j++) {
          sprintf(msgstr, "EV%03d | ", j + 1);
          Serial << msgstr;
        }

        Serial << F("Hash |") << endl;

        Serial << F(" --------------------------------------------------------------") << endl;

        // for each event data line
        for (byte j = 0; j < module_config.EE_MAX_EVENTS; j++) {
          if (module_config.getEvTableEntry(j) != 0) {
            sprintf(msgstr, "  %03d  | ", j);
            Serial << msgstr;

            // for each data byte of this event
            for (byte e = 0; e < (module_config.EE_NUM_EVS + 4); e++) {
              sprintf(msgstr, " 0x%02hx | ", module_config.readEEPROM(module_config.EE_EVENTS_START + (j * module_config.EE_BYTES_PER_EVENT) + e));
              Serial << msgstr;
            }

            sprintf(msgstr, "%4d |", module_config.getEvTableEntry(j));
            Serial << msgstr << endl;
          }
        }

        Serial << endl;

        break;

      // NVs
      case 'v':
        // note NVs number from 1, not 0
        Serial << "> Node variables" << endl;
        Serial << F("   NV   Val") << endl;
        Serial << F("  --------------------") << endl;

        for (byte j = 1; j <= module_config.EE_NUM_NVS; j++) {
          byte v = module_config.readNV(j);
          sprintf(msgstr, " - %02d : %3hd | 0x%02hx", j, v, v);
          Serial << msgstr << endl;
        }

        Serial << endl << endl;

        break;

      // CAN bus status
      case 'c':
        CBUS.printStatus();
        break;

      case 'h':
        // event hash table
        module_config.printEvHashTable(false);
        break;

      case 'y':
        // reset CAN bus and CBUS message processing
        CBUS.reset();
        break;

      case '*':
        // reboot
        module_config.reboot();
        break;

      case 'm':
        // free memory
        Serial << F("> free SRAM = ") << module_config.freeSRAM() << F(" bytes") << endl;
        break;

      case 'r':
        // renegotiate
        CBUS.renegotiate();
        break;

      case 'z':
        // Reset module, clear EEPROM
        static bool ResetRq = false;
        static unsigned long ResWaitTime;
        if (!ResetRq) {
          // start timeout timer
          Serial << F(">Reset & EEPROM wipe requested. Press 'z' again within 2 secs to confirm") << endl;
          ResWaitTime = millis();
          ResetRq = true;
        }
        else {
          // This is a confirmed request
          // 2 sec timeout
          if (ResetRq && ((millis() - ResWaitTime) > 2000)) {
            Serial << F(">timeout expired, reset not performed") << endl;
            ResetRq = false;
          }
          else {
            //Request confirmed within timeout
            Serial << F(">RESETTING AND WIPING EEPROM") << endl;
            module_config.resetModule();
            ResetRq = false;
          }
        }
        break;

      default:
        // Serial << F("> unknown command ") << c << endl;
        break;
    }
  }
}
