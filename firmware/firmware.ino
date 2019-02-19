#include <SPI.h>

#include "libs/arduino-mcp2515/can.h"
#include "libs/arduino-mcp2515/mcp2515.h"
#include "libs/arduino-mcp2515/mcp2515.cpp" // the arduino linker is not very smart, so I have to put this in...


#define CAN_ID 0x6


MCP2515 mcp2515(10);

void setup() {
  Serial.begin(115200);
  SPI.begin();
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS);
  mcp2515.setNormalMode();
}

void loop() {
  // put your main code here, to run repeatedly:

}
