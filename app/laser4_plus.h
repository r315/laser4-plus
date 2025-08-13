#ifndef _app_h_
#define _app_h_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "console.h"

#ifdef ENABLE_VCP
#include "usbd_cdc_if.h"
#endif

#ifdef ENABLE_GAME_CONTROLLER
#include "game_controller.h"
#endif

/* Indexes of constants in eeprom */
#define EEPROM_ID_OFFSET        0UL
#define IDX_BUZ_VOLUME          28
#define EEPROM_BIND_FLAG        29 //EEPROM_SIZE - 1
// 32-bit indexes
#define IDX_BAT_VOLTAGE_DIV     2
#define IDX_SENSE_RESISTOR      4
// 16-bit indexes
#define IDX_CHANNEL_MAX_100     6
#define IDX_CHANNEL_MIN_100     7
#define IDX_CHANNEL_MAX_125     8
#define IDX_CHANNEL_MIN_125	    9
#define IDX_PPM_MAX_100         10
#define IDX_PPM_MIN_100         11
#define IDX_CHANNEL_SWITCH      12
#define IDX_PPM_DEFAULT_VALUE   13

#define DEFAULT_ID              0x2AD141A7
#define DEFAULT_VOLTAGE_DIV     0x3e27ef9e  // 0.164
#define DEFAULT_SENSE_RESISTOR  0x3db851ec  // 0.09

#define APP_FLAGS               app_flags
#define IS_BAT_LOW              APP_FLAGS & (1<<0)
#define IS_BAT_ICO_ON           APP_FLAGS & (1<<1)
#define IS_ERROR_ICO_ON         APP_FLAGS & (1<<2)
#define IS_BIND_ICO_ON          APP_FLAGS & (1<<3)
#define IS_LCD_UPDATE           APP_FLAGS & (1<<4)

#define SET_BAT_LOW             APP_FLAGS = (APP_FLAGS | (1<<0))
#define CLR_BAT_LOW             APP_FLAGS = APP_FLAGS & ~(1<<0)

#define SET_BAT_ICO             APP_FLAGS = (APP_FLAGS | (1<<1))
#define CLR_BAT_ICO             APP_FLAGS = APP_FLAGS & ~(1<<1)

#define SET_ERROR_ICO           APP_FLAGS = (APP_FLAGS | (1<<2))
#define CLR_ERROR_ICO           APP_FLAGS = APP_FLAGS & ~(1<<2)

#define SET_BIND_ICO            APP_FLAGS = (APP_FLAGS | (1<<3))
#define CLR_BIND_ICO            APP_FLAGS = APP_FLAGS & ~(1<<3)

#define SET_LCD_UPDATE          APP_FLAGS = (APP_FLAGS | (1<<4))
#define CLR_LCD_UPDATE          APP_FLAGS = APP_FLAGS & ~(1<<4)

#define EEPROM_SIZE             31

#define STATE_BITS          4
#define STATE_MASK          ((1<<STATE_BITS) - 1)

#define TIMER_BATTERY_TIME  10000U  // ms
#define TIMER_LOWBAT_TIME   500U    // ms
#define WATCHDOG_TIME       3000U   // ms
#define TIMER_PPM_TIME      500U

#define NO                  0
#define YES                 1
#define ON                  0
#define OFF                 1

enum {
    STARTING = 0,
    MODE_MULTIPROTOCOL,
    MODE_HID,
    REQ_MODE_CHANGE,
};

typedef union {
  float f;
  uint32_t u;
}f2u_u;

extern uint16_t *eeprom_data;
extern uint32_t app_flags;

void appReqModeChange(uint8_t new_mode);
uint8_t appGetCurrentMode(void);
void appLoadEEPROM(void);
void appSaveEEPROM(void);
void appDefaultEEPROM(void);

#ifdef __cplusplus
#if defined(ENABLE_CLI)
extern ConsoleCommand *laser4_commands[];
#endif /* ENABLE_CLI */
}
#endif

#endif /* _APP_H_ */