#ifndef _app_h_
#define _app_h_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "board.h"

#define USE_FREERTOS

#ifdef USE_FREERTOS
#include <FreeRTOS.h>
#include <console.h>
#include <task.h>
#include <queue.h>
#include <stdout.h>
#include "usbd_cdc_if.h"

extern stdout_t vcom;
#ifdef __cplusplus
extern ConsoleCommand *laser4_commands[];
#endif
void app_setup(void);
void app_loop(void *ptr);

#else
void app_main(void);
#endif

#ifdef __cplusplus
}
#endif

#endif