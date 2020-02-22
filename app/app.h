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

//#define USE_FREERTOS

#if defined(ENABLE_DEBUG)
    #define DBG_PRINT dbg_printf
    #define DBG_DUMP_LINE dbg_HexDumpLine
#else
    #define DBG_PRINT(...)
    #define DBG_DUMP_LINE(...)
#endif

enum {
    STARTING = 0,
    MODE_MULTIPROTOCOL,
    MODE_HID,
    REQ_MODE_CHANGE,
};

#define STATE_BITS          4
#define STATE_MASK          ((1<<STATE_BITS) - 1)

void reqModeChange(uint8_t new_mode);
uint8_t getCurrentMode(void);

#ifdef __cplusplus
#ifdef ENABLE_CONSOLE
extern Console con;
extern ConsoleCommand *laser4_commands[];
#endif /* ENABLE_CONSOLE */
}
#endif

#endif /* _APP_H_ */