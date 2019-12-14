#ifndef _board_h_
#define _board_h_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f1xx.h"
//#include <stm32f103xb.h>


/* GPIO definitions */
#define GPIO_MODE_MASK  15
#define GPIO_MODE_OUT   (2 << 0)     // max speed 2MHz
#define GPIO_CFG_OOD    (1 << 2)     // Output open drain

#define GPIO_ENABLE RCC->APB2ENR |= (1<<4) | (1 << 3) | (1 << 2) | (1 << 0)

#define GPO_INIT(_IO, _PIN) BOARD_GPO_Init(_IO,_PIN)
#define GPO_SET(_IO, _PIN) _IO->BSRR = (1 << _PIN)
#define GPO_CLEAR(_IO, _PIN) _IO->BRR = (1 << _PIN)

#if 0
#define LED_PORT    GPIOB
#define LED_PIN     GPIO_PIN_3
#define LED_INIT    GPIOB->CRL = (GPIOB->CRL & ~(15<<12)) | (2<<12) // assume swd is already enabled
#define LED_SET     GPIO_PIN_RESET
#define LED_RESET   GPIO_PIN_SET
#else// Blue pill LED
#define LED_PORT    GPIOC
#define LED_PIN     13
#define LED_ON      GPO_SET(LED_PORT, LED_PIN)
#define LED_OFF     GPO_CLEAR(LED_PORT, LED_PIN)
#endif

#define DBG_LED_TOGGLE LED_PORT->ODR ^= (1<<LED_PIN)
#define DBG_LED_ON LED_ON
#define DBG_LED_OFF LED_OFF
#define DBG_LED_INIT GPO_INIT(LED_PORT, LED_PIN)

#define GPIO_ENABLE RCC->APB2ENR |= (1<<4) | (1 << 3) | (1 << 2) | (1 << 0)

//PB6
#define DBG_PIN_INIT GPIOB->CRL = (GPIOB->CRL & ~(GPIO_MODE_MASK<<24)) | (GPIO_MODE_OUT << 24)
#define DBG_PIN_TOGGLE GPIOB->ODR ^= (1 << 6)


#define MCO_EN GPIOA->CRH = (GPIOA->CRH & ~(15<<0)) | (11 << 0); \
            RCC->APB2ENR |= (1 << 0)
 

void BOARD_GPO_Init(GPIO_TypeDef *port, uint8_t pin);
void SPI_Write(uint8_t data);
uint8_t SPI_Read(void);

#ifdef __cplusplus
}
#endif

#endif