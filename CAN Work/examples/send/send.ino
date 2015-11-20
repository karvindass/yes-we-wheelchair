// demo: CAN-BUS Shield, send data
#include "mcp_can.h"
#include <SPI.h>

MCP_CAN CAN(10);                                      // Set CS to pin 10

void setup()
{
    Serial.begin(500);

START_INIT:

    if(CAN_OK == CAN.begin(CAN_500KBPS))                   // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init ok!");
    }
    else
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
        goto START_INIT;
    }
}

unsigned char stmp[5] = {0, 0, 0, 0, 1};
void loop()
{
    // send data:  id = 0x00, standrad flame, data len = 8, stmp: data buf
    CAN.sendMsgBuf(0x00, 0, 8, stmp);
    delay(100);                       // send data per 100ms
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
