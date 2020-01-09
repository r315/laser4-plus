
#ifndef _GAME_CONTROLLER_H_
#define _GAME_CONTROLLER_H_

#include <stdint.h>

#define IS_PPM_FRAME_READY      (gflags & (1<<1))
#define SET_PPM_FRAME           (gflags |= (1<<1))
#define CLR_PPM_FRAME           (gflags &= ~(1<<1))
#define PAUSE_CAPTURE           PPM_TIM->DIER &= ~(TIM_DIER_CC4IE)
#define RESUME_CAPTURE          PPM_TIM->DIER |=  (TIM_DIER_CC4IE)

#define MIN_RADIO_CHANNELS      4

#if defined(ENABLE_PPM)
#define PPM_MAX_CHANNELS        8
#define PPM_MAX_PULSE           2100
#define PPM_MIN_PULSE           900
#define PPM_CENTER_PULSE      ((PPM_MAX_PULSE - PPM_MIN_PULSE)/2)
#define PPM_TIM_IRQn            TIM3_IRQn
#define PPM_TIM                 TIM3          //PB5 -> TIM3_CH2
#define PPM_TIM_IRQHandler      TIM3_IRQHandler
#endif

#if defined(ENABLE_PWM)
/**
 * PA0 <- CH1
 * PA1 <- CH2
 * PA2 <- CH3
 * PA3 <- CH4
 * */
#define TIM_CAP_POL(ch) (2 << (ch - 1) * 4)
// in PWM mode half of the buffer is to store
// the first captured values
#define THROTTLE_OFFSET       200
#define PWM_MAX_PULSE         3000
#define PWM_MIN_PULSE         1000
#define PWM_CENTER_PULSE      ((PWM_MAX_PULSE - PWM_MIN_PULSE)/2)
#endif

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