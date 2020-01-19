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

#ifdef ENABLE_GAME_CONTROLLER
#include "usbd_hid_if.h"
#include "game_controller.h"
#endif

//#define USE_FREERTOS

#if defined(ENABLE_DEBUG) && defined(DEBUG_TO_CONSOLE)
    #define DBG_PRINT con.print
#elif defined(ENABLE_DEBUG)
    #define DBG_PRINT dbg_printf
    #define DBG_DUMP_LINE dbgHexDumpLine
#else
    #define DBG_PRINT(...)
#endif

enum {
    MULTIPROTOCOL,
    HID,
    REQ_MODE_CHANGE,
};

#if defined(ENABLE_VCOM)
extern stdout_t vcom;
#define IO_CHAR &vcom
#elif defined(ENABLE_USART)
extern stdout_t pcom;
#define IO_CHAR &pcom
#endif

void reqModeChange(void *ptr);

#ifdef __cplusplus
#ifdef ENABLE_CONSOLE
extern Console con;
extern ConsoleCommand *laser4_commands[];
#endif /* ENABLE_CONSOLE */
}
#endif

#endif /* _APP_H_ */