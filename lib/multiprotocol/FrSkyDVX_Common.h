
#ifndef _FRSKYDVX_COMMON_H_
#define _FRSKYDVX_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "multiprotocol.h"

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

void Frsky_init_hop(void);
void FRSKY_init_cc2500(const uint8_t *ptr);
void Frsky_SetPower(void);
uint16_t convert_channel_frsky(uint8_t num);

//FrSkyD
uint16_t initFrSky_2way(void);
uint16_t ReadFrSky_2way(void);


#ifdef __cplusplus
}
#endif

#endif /* #ifndef _FRSKYDVX_COMMON_H_ */