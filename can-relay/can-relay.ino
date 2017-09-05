#include <Arduino.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include "mcp_can.h"
#include "mcp_can_dfs.h"


#include "CANMessage.h"

uint8_t i;
boolean rcvFlag;
uint8_t currentGear = 0x0;
uint8_t previousGear = 0x0;

void MCP2515_ISR()
{
    rcvFlag = true;
}

void attachCAN()
{
  #if defined(__AVR_ATmega32U4__) // Arduino Pro Micro
    pinMode(7, INPUT);
    attachInterrupt(digitalPinToInterrupt(7), MCP2515_ISR, FALLING); // start interrupt
  #else // Other Arduinos (Nano in my case)
    pinMode(2, INPUT);
    attachInterrupt(digitalPinToInterrupt(2), MCP2515_ISR, FALLING); // start interrupt
  #endif
}

void detachCAN()
{
  #if defined(__AVR_ATmega32U4__) // Arduino Pro Micro
    pinMode(7, INPUT);
    detachInterrupt(digitalPinToInterrupt(7));
  #else // Other Arduinos (Nano in my case)
    pinMode(2, INPUT);
    detachInterrupt(digitalPinToInterrupt(2));
  #endif
}

void setup()
{
  Serial.begin(115200);

  START_INIT:
    if(CAN_OK == CAN.begin(MCP_STDEXT, CAN_125KBPS, MCP_8MHZ)) {
      Serial.println(F("CAN ok!"));
    } else {
      Serial.println(F("CAN fail"));
      delay(100);
      goto START_INIT;
    }

    attachCAN();

    CAN.setMode(MCP_LOOPBACK);

    CAN.init_Mask(0, CAN_STDID, 0x07FF0000);   // there are 2 mask in mcp2515, you need to set both of them
    CAN.init_Mask(1, CAN_STDID, 0x07FF0000);
    for (i=0;i<5;i++)
      CAN.init_Filt(i, CAN_STDID, 0x03B00000);   // Gear

    CAN.init_Filt(filtNo++, CAN_STDID, 0x03B00000);   // Gear

    CAN.setMode(MCP_NORMAL);

    pinMode(3, OUTPUT);

}

void loop()
{
  if (rcvFlag) {
    rcvFlag = false;
    while (CAN_MSGAVAIL == CAN.checkReceive()) {

      CAN.readMsgBuf(&rcvCanId, &rcvLen, rcvBuf);

      switch (rcvCanId) {
        case 0x3b0: { // Gear
              noInterrupts();
              previousGear = currentGear;
              currentGear = rcvBuf[0];
              interrupts();
            }
        }
          break;
      }
  }

  noInterrupts();
  if ( (currentGear == 0x0D) && (previousGear != 0x0D) ) {  // Moved to parking
    digitalWrite(3, HIGH);
  }

  if ( (currentGear != 0x0D) && (previousGear == 0x0D) ) {  // Moved from parking
    digitalWrite(3, LOW);
  }
  interrupts();

}
