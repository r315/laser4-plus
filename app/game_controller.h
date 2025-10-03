
#ifndef _GAME_CONTROLLER_H_
#define _GAME_CONTROLLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "multiprotocol.h"

#define LOGICAL_MINIMUM         0
#define LOGICAL_MAXIMUM         2047
#define REPORT_SIZE             13
#define REPORT_CHANNELS_SIZE    6
#define REPORT_SWITCHES_SIZE    1

#pragma pack (1)
typedef struct controller{
/* must follow HID report structure */
  int8_t  buttons;
  int16_t  pitch;
  int16_t  roll;
  int16_t  throttle;
  int16_t  yaw;
  int16_t  aux1;
  int16_t  aux2;
}controller_t;

uint16_t USBHID_init(radio_t *radio);
uint16_t USBHID_callback(radio_t *radio);

#ifdef __cplusplus
}
#endif

#endif