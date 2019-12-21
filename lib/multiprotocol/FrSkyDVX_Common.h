
#ifndef _FRSKYDVX_COMMON_H_
#define _FRSKYDVX_COMMON_H_

#include <stdint.h>
#include "radio.h"

#ifdef STM32_BOARD
#define PROGMEM
#define pgm_read_byte_near(_X) (*(const unsigned char *)(_X))
#endif

#if defined(FRSKYD_CC2500_INO) || defined(FRSKYX_CC2500_INO)
enum {
	FRSKY_BIND		= 0,
	FRSKY_BIND_DONE	= 1000,
	FRSKY_DATA1,
	FRSKY_DATA2,
	FRSKY_DATA3,
	FRSKY_DATA4,
	FRSKY_DATA5,
};
#endif

extern const PROGMEM uint8_t FRSKYD_cc2500_conf[];

void Frsky_init_hop(radio_t *radio);
void FRSKY_init_cc2500(const uint8_t *ptr, radio_t *radio);
void Frsky_SetPower(radio_t *radio);
uint16_t convert_channel_frsky(uint8_t num, radio_t *radio);

#endif /* #ifndef _FRSKYDVX_COMMON_H_ */