#ifndef _app_h_
#define _app_h_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "board.h"
#include <FreeRTOS.h>
#include <console.h>
#include <task.h>
#include <queue.h>
#include <stdout.h>
#include "usbd_cdc_if.h"

#define USE_FREERTOS

#ifdef DEBUG_SERIAL
#define DBG_PRINT con.print
#else
#define DBG_PRINT(...)
#endif

extern stdout_t vcom;

void app_setup(void);
void app_loop(void *ptr);

#ifdef __cplusplus
extern ConsoleCommand *laser4_commands[];
}
#endif

#endif