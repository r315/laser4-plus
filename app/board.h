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

#define GPO_INIT(_IO, _PIN)     BOARD_GPO_Init(_IO,_PIN)
#define GPO_SET(_IO, _PIN)      _IO->BSRR = (1 << _PIN)
#define GPO_CLEAR(_IO, _PIN)    _IO->BRR = (1 << _PIN)
#define GPO_TOGGLE(_IO, _PIN)   _IO->ODR ^= (1<<_PIN)

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
#define LED_INIT    GPO_INIT(LED_PORT, LED_PIN); LED_OFF
#define LED_TOGGLE  GPO_TOGGLE(LED_PORT, LED_PIN)
#endif

#define DBG_LED_TOGGLE  LED_TOGGLE 
#define DBG_LED_ON      LED_ON
#define DBG_LED_OFF     LED_OFF
#define DBG_LED_INIT    LED_INIT


//PB6
#define CC25_CS_PIN     6
#define CC25_CS_PORT    GPIOB
#define CC25_CS_INIT    GPO_INIT(CC25_CS_PORT, CC25_CS_PIN); CC25_CS_FALSE
#define CC25_CS_FALSE   GPO_SET(CC25_CS_PORT, CC25_CS_PIN)
#define CC25_CS_TRUE    GPO_CLEAR(CC25_CS_PORT, CC25_CS_PIN)
#define DBG_PIN_TOGGLE  GPO_TOGGLE(CC25_CS_PORT, CC25_CS_PIN)


/* SPI */
#define SPI_PINS_INIT GPIOB->CRH = (GPIOB->CRH & ~(0xFFF << 20)) | (0xB4B << 20); //Output AF_PP, IN no pull

//Main Clock Outpu, requires prior MCO bit in RCC_CFG
#define MCO_EN GPIOA->CRH = (GPIOA->CRH & ~(15<<0)) | (11 << 0); \
               RCC->APB2ENR |= (1 << 0)

#define HW_BIND_BUTTON
#define IS_HW_BIND_BUTTON_PRESSED 0
#define HW_CC2500_MODULE_RESET

void BOARD_Init(void);
void BOARD_DelayMs(uint32_t ms);
uint32_t BOARD_GetTick(void);
void BOARD_GPO_Init(GPIO_TypeDef *port, uint8_t pin);
void SPI_Write(uint8_t data);
uint8_t SPI_Read(void);

uint16_t BOARD_EEPROM_Write(uint16_t address, uint8_t *data, uint16_t len);
uint16_t BOARD_EEPROM_Read(uint16_t address, uint8_t *data, uint16_t len);


/* Definitions for Multiprotocol */
#define delayMilliseconds(x) BOARD_DelayMs(x)
#define millis BOARD_GetTick
#define IS_LED_on   (LED_PORT->IDR & (1<<LED_PIN))
#define LED_off     LED_OFF
#define LED_toggle  LED_TOGGLE
#define EE_ADDR     uint16_t

#define cli     __disable_irq
#define sei     __enable_irq

#ifdef __cplusplus
}
#endif

#endif