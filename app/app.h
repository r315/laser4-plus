#ifndef _app_h_
#define _app_h_

#include <stdint.h>
#include "board.h"

#define USE_FREERTOS

#ifdef USE_FREERTOS
#include <FreeRTOS.h>
#include <task.h>
void app_setup(void);
void app_loop(void *ptr);
#else
void app_main(void);
#endif
#endif