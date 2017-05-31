#ifndef _A7105_TX_H_
#define _A7105_TX_H_

static u8 packet[21];



void A7105_Transmitt(uint8_t tx_channels);
u8 A7105_BuildPacket();

#endif