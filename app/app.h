#ifndef _app_h_
#define _app_h_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "board.h"
#include <nvdata.h>
#include <stdout.h>
#include <fifo.h>

#ifdef ENABLE_CONSOLE
#include <console.h>
#include "usbd_cdc_if.h"
#endif

#ifdef ENABLE_GAME_CONTROLLER
#include "usbd_hid_if.h"
#include "game_controller.h"
#endif

//#define USE_FREERTOS

#ifdef DEBUG_SERIAL
#ifdef ENABLE_CONSOLE
    #define DBG_PRINT con.print
#else
//#include <stdio.h>
    #define DBG_PRINT(...) //printf
#endif
#else
#define DBG_PRINT(...)
#endif

void app_setup(void);
void app_loop(void *ptr);
extern void (*device_process)(void);

#if defined(ENABLE_VCOM)
extern stdout_t vcom;
#define IO_CHAR &vcom
#elif defined(ENABLE_USART)
extern stdout_t pcom;
#define IO_CHAR &pcom
#endif

#ifdef __cplusplus
#ifdef ENABLE_CONSOLE
extern ConsoleCommand *laser4_commands[];
#endif
}
#endif

#endif