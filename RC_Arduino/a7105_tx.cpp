#include "a7105.h"
#include "a7105_tx.h"

unsigned int id = 0x9000295C;

void A7105_Transmitt(uint8_t tx_channels)
{
        A7105_BuildPacket();
        A7105_SetPower(TXPOWER_100mW);
        A7105_WriteData(packet, 11, tx_channels);
      
}

void A7105_BuildPacket()
{
    packet[0] = 0x55;
    packet[1] = (id >>  0) & 0xff;
    packet[2] = (id >>  8) & 0xff;
    packet[3] = (id >> 16) & 0xff;
    packet[4] = (id >> 24) & 0xff;
    packet[5] = 0xAA;
    packet[6] = 0xBB;
    packet[7] = 0xCC;
    packet[8] = 0xDD;
    packet[9] = 0xEE;
    packet[10]= 0xFF;

}