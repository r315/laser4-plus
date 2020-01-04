
#ifndef _GAME_CONTROLLER_H_
#define _GAME_CONTROLLER_H_

#include <stdint.h>

// in PWM mode half of the buffer is to store
// the first captured values
#define MAX_REMOTE_CHANNELS 8
#define THROTTLE_OFFSET 200
#define PWM_MAX_PULSE 2000
#define PWM_CENTER_PULSE 1500
#define PWM_MIN_PULSE 1000


#if defined(ENABLE_PPM)
#define PPM_CHANNELS 6
#define PPM_TIM_IRQn TIM3_IRQn
#define PPM_TIM TIM3    //PB9 -> TIM4 Pxx->TIM3
#define PPM_TIM_IRQHandler TIM3_IRQHandler
#endif

#if defined(ENABLE_PWM)
/**
 * PA0 <- CH1
 * PA1 <- CH2
 * PA2 <- CH3
 * PA3 <- CH4
 * */
#define TIM_CAP_POL(ch) (2 << (ch - 1) * 4)
#endif

#define TEST_PIN (1 << 5)  //PB5
#define TEST_PIN_PORT GPIOB
#define CFG_TEST_PIN TEST_PIN_PORT->CRL &= ~(0x0F << 20); TEST_PIN_PORT->CRL |= (2 << 20);
#define TOGGLE_TEST_PIN TEST_PIN_PORT->ODR ^= TEST_PIN

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
}controller_t;

void CONTROLLER_Process(void);
void CONTROLLER_Init(void);

#define LOGICAL_MINIMUM 0 //-127
#define LOGICAL_MAXIMUM 2047 // 127

#endif