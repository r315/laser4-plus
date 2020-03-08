#ifndef _app_h_
#define _app_h_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <nvdata.h>
#include <stdout.h>
#include <fifo.h>
#include <console.h>
#include <dbg.h>
#include "board.h"

#ifdef ENABLE_VCOM
#include "usbd_cdc_if.h"
#endif

#ifdef ENABLE_USART
#include "usart.h"
#endif

#ifdef ENABLE_GAME_CONTROLLER
#include "game_controller.h"
#endif

/* Indexes of constants in eeprom */
#define EEPROM_ID_OFFSET        0
#define EEPROM_BIND_FLAG        29 //EEPROM_SIZE - 1
// Two byte indexes
#define IDX_BAT_VOLTAGE_DIV     2
#define IDX_CHANNEL_MAX_100     4
#define IDX_CHANNEL_MIN_100     5
#define IDX_CHANNEL_MAX_125     6
#define IDX_CHANNEL_MIN_125	    7
#define IDX_CHANNEL_SWITCH      8
#define IDX_PPM_MAX_100         9
#define IDX_PPM_MIN_100         10
#define IDX_PPM_DEFAULT_VALUE   12

#define DEFAULT_ID              0x2AD141A7
#define BAT_VOLTAGE_DIV         0x3e27ef9e  //0.164

//#define USE_FREERTOS

#if defined(ENABLE_DEBUG)
    #define DBG_PRINT dbg_printf
    #define DBG_DUMP_LINE dbg_HexDumpLine
#else
    #define DBG_PRINT(...)
    #define DBG_DUMP_LINE(...)
#endif


#define STATE_BITS          4
#define STATE_MASK          ((1<<STATE_BITS) - 1)

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

extern uint16_t eeprom_data[];

void appReqModeChange(uint8_t new_mode);
uint8_t appGetCurrentMode(void);
void appInitEEPROM(uint8_t *dst);
void appSaveEEPROM(void);

#ifdef __cplusplus
#ifdef ENABLE_CLI
extern Console con;
extern ConsoleCommand *laser4_commands[];
#endif /* ENABLE_CLI */
}
#endif

#endif /* _APP_H_ */