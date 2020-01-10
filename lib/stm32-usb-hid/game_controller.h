
#ifndef _GAME_CONTROLLER_H_
#define _GAME_CONTROLLER_H_

#include <stdint.h>

#define IS_PPM_FRAME_READY      (gflags & (1<<1))
#define SET_PPM_FRAME           (gflags |= (1<<1))
#define CLR_PPM_FRAME           (gflags &= ~(1<<1))
#define PAUSE_CAPTURE           PPM_TIM->DIER &= ~(TIM_DIER_CC4IE)
#define RESUME_CAPTURE          PPM_TIM->DIER |=  (TIM_DIER_CC4IE)

#pragma pack (1)
// must follow HID report structure
typedef struct controller{
  int8_t  buttons;
  int16_t  pitch;
  int16_t  roll;
  int16_t  throttle;
  int16_t  yaw;
  int16_t  aux1;
  int16_t  aux2;

  uint16_t max_pulse;
  uint16_t min_pulse;
}controller_t;

void CONTROLLER_Process(void);
void CONTROLLER_Init(void);

#define LOGICAL_MINIMUM     0
#define LOGICAL_MAXIMUM     2047
#define REPORT_SIZE         13

#endif