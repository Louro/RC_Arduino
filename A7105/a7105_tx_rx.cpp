#include "a7105.h"
#include "a7105_tx_rx.h"

unsigned long id = 0x11111111;//0x00000000;//0x12345679;

void A7105_Transmitt(uint8_t tx_channels)
{
	u8 cnt;
    cnt = A7105_BuildPacket();
        A7105_SetPower(TXPOWER_100mW);
        A7105_WriteData(packet, cnt, tx_channels);
      
}

u8 A7105_BuildPacket()
{
	u8 cnt = 0;
    packet[cnt++] = 0x55;
    packet[cnt++] = (id >>  0) & 0xff;
    packet[cnt++] = (id >>  8) & 0xff;
    packet[cnt++] = (id >> 16) & 0xff;
    packet[cnt++] = (id >> 24) & 0xff;
    packet[cnt++] = 0xAA;
    packet[cnt++] = 0xBB;
    packet[cnt++] = 0xCC;
    packet[cnt++] = 0xDD;
    packet[cnt++] = 0xEE;
    packet[cnt++]= 0xFF;
	return cnt;
}