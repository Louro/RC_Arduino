#ifndef _IFACE_CONFIG_H_
#define _IFACE_CONFIG_H_

#include <stdint.h>

#define NUM_OUT_CHANNELS 12
#define CHAN_MULTIPLIER 100
#define CHAN_MAX_VALUE (100 * CHAN_MULTIPLIER)

#define s16 int16_t
#define u16 uint16_t
#define s32 int32_t
#define u32 uint32_t
#define u8 uint8_t


enum TxPower {
    TXPOWER_100uW,
    TXPOWER_300uW,
    TXPOWER_1mW,
    TXPOWER_3mW,
    TXPOWER_10mW,
    TXPOWER_30mW,
    TXPOWER_100mW,
    TXPOWER_150mW,
    TXPOWER_LAST,
};

enum RadioStatus {
    RADIO_INIT,
    RADIO_BIND,
    RADIO_RX_WAIT,
    RADIO_RX,
    RADIO_TX,
    RADIO_IDLE,
};


void RADIOCOL_SetBindState(u32 msec);

#endif