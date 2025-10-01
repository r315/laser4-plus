#ifndef _app_h_
#define _app_h_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "console.h"

//Channel MIN MAX values
#define CHANNEL_MAX_100         1844    // 100%
#define CHANNEL_MIN_100         844     // 100%
#define CHANNEL_MAX_125         2047    // 125%
#define CHANNEL_MIN_125         0       // 125%
#define CHANNEL_SWITCH          1104    // 1550us

#define PPM_MAX_100             (2012 << 1) // 0.5us
#define PPM_MIN_100             (988 << 1)  // 0.5us

#define PPM_DEFAULT_VALUE       0
#define AUX_SWITCH_NUM          3
#define AUX_ENC_NUM             1

#define DEFAULT_VOLTAGE_DIV     0x3e27ef9e  // 0.164
#define DEFAULT_SENSE_RESISTOR  0x3db851ec  // 0.09

#define TIMER_BATTERY_TIME  10000U  // ms
#define TIMER_LOWBAT_TIME   500U    // ms
#define WATCHDOG_TIME       3000U   // ms
#define TIMER_PPM_TIME      500U

#define NO                  0
#define YES                 1
#define ON                  0
#define OFF                 1

typedef enum app_mode{
#ifndef MODE_SERIAL
    MODE_SERIAL = 0,
#endif
    MODE_CHANGE_REQ = 1,
    MODE_CC2500 = 10,
    MODE_HID = 13,
    MODE_PPM = 14,
    MODE_NONE = 15,
}app_mode_t;

typedef enum app_state{
    APP_STATE_INIT,
    APP_STATE_LOOP,
    APP_STATE_MODE_REQ
}app_state_t;

typedef union {
  float f;
  uint32_t u;
}f2u_u;

void appModeRequest(app_mode_t new_mode);
app_mode_t appGetCurrentMode(void);
uint32_t appGetBatConsumed(void);
uint32_t appGetUpTime(void);

#ifdef __cplusplus
#if defined(ENABLE_CLI)
extern ConsoleCommand *laser4_commands[];
#endif /* ENABLE_CLI */
}
#endif

#endif /* _APP_H_ */