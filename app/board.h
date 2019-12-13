#ifndef _board_h_
#define _board_h_

#ifdef __cplusplus
extern "C" {
#endif

//#include "stm32f1xx.h"
#include <stm32f103xb.h>


/* GPIO definitions */
#define GPIO_MODE_MASK  15
#define GPIO_MODE_OUT   (2 << 0)     // max speed 2MHz
#define GPIO_CFG_OOD    (1 << 2)     // Output open drain

#if 0
#define LED_PORT    GPIOB
#define LED_PIN     GPIO_PIN_3
#define LED_INIT    GPIOB->CRL = (GPIOB->CRL & ~(15<<12)) | (2<<12) // assume swd is already enabled
#define LED_SET     GPIO_PIN_RESET
#define LED_RESET   GPIO_PIN_SET
#else// Blue pill LED
#define LED_PORT    GPIOC
#define LED_PIN     (1<<13) //GPIO_PIN_13
#define LED_ON      LED_PORT->BSRR = LED_PIN
#define LED_OFF     LED_PORT->BRR = LED_PIN
#endif

/* TODO: Fix shifts */
#if  LED_PIN < (1 << 8) //GPIO_PIN_8
#define LED_INIT    LED_PORT->CRL = (LED_PORT->CRL & ~(15<<12)) | (2<<12)
#else
#define LED_INIT    LED_PORT->CRH = (LED_PORT->CRH & ~(15<<20)) | (2<<20);                    
#endif

#define DBG_LED_TOGGLE LED_PORT->ODR ^= LED_PIN
#define DBG_LED_ON LED_ON
#define DBG_LED_OFF LED_OFF
#define DBG_LED_INIT LED_INIT

#define GPIO_ENABLE RCC->APB2ENR |= (1<<4) | (1 << 3) | (1 << 2) | (1 << 0)

//PB6
#define DBG_PIN_INIT GPIOB->CRL = (GPIOB->CRL & ~(GPIO_MODE_MASK<<24)) | (GPIO_MODE_OUT << 24)
#define DBG_PIN_TOGGLE GPIOB->ODR ^= (1 << 6)


#define MCO_EN GPIOA->CRH = (GPIOA->CRH & ~(15<<0)) | (11 << 0); \
            RCC->APB2ENR |= (1 << 0)
 
#ifdef __cplusplus
}
#endif

#endif