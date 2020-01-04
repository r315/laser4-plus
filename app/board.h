#ifndef _board_h_
#define _board_h_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f1xx.h"

/* GPIO definitions */
#define GPIO_MODE_MASK          15
#define GPIO_MODE_O2MHZ         (2 << 0)
#define GPIO_MODE_O10MHZ        (1 << 0)
#define GPIO_MODE_O50MHZ        (3 << 0)
#define GPIO_CNF_OD             (1 << 2)     // Output open drain
#define GPIO_CNF_AF             (2 << 2)

#define GPIO_ENABLE RCC->APB2ENR |=      \
                      RCC_APB2ENR_IOPCEN \
                    | RCC_APB2ENR_IOPBEN \
                    | RCC_APB2ENR_IOPAEN \
                    | RCC_APB2ENR_AFIOEN;

#define GPO_INIT(_IO, _PIN)     BOARD_GPIO_Init(_IO,_PIN, GPIO_MODE_O2MHZ)
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


/* CC250 Chip select PB6 */
#define CC25_CS_PIN     6
#define CC25_CS_PORT    GPIOB
#define CC25_CS_INIT    GPO_INIT(CC25_CS_PORT, CC25_CS_PIN); CC25_CS_FALSE
#define CC25_CS_FALSE   GPO_SET(CC25_CS_PORT, CC25_CS_PIN)
#define CC25_CS_TRUE    GPO_CLEAR(CC25_CS_PORT, CC25_CS_PIN)
#define DBG_PIN_TOGGLE  GPO_TOGGLE(CC25_CS_PORT, CC25_CS_PIN)


/* SPI2 */
#define SPI_PINS_INIT GPIOB->CRH = (GPIOB->CRH & ~(0xFFF << 20)) | (0xB4B << 20); //Output AF_PP, IN no pull

//Main Clock Output, requires prior MCO bit in RCC_CFG
#define MCO_EN GPIOA->CRH = (GPIOA->CRH & ~(15<<0)) | (11 << 0); \
               RCC->APB2ENR |= (1 << 0)

/* Button pin PB5 */
#define HW_BIND_BUTTON_PIN        5
#define HW_BIND_BUTTON_INIT       BOARD_GPIO_Init(GPIOB, HW_BIND_BUTTON_PIN, GPIO_CNF_AF); GPIOB->ODR |= (1<<HW_BIND_BUTTON_PIN)
#define IS_HW_BIND_BUTTON_PRESSED (GPIOB->IDR & (1 << HW_BIND_BUTTON_PIN)) == 0
#define HW_CC2500_MODULE_RESET

/* fast code */
#define RAM_CODE __attribute__((section(".ram_code")))

/* Symbols for NVDATA */
extern uint32_t _seeprom, _eeeprom;     //declared on linker script
#define NVDATA_SECTOR_INIT      
#define NVDATA_SECTOR_START     &_seeprom
#define NVDATA_SECTOR_END       &_eeeprom
#define NVDATA_SECTOR_READ      memcpy
#define NVDATA_SECTOR_WRITE     flash_write
#define NVDATA_SECTOR_ERASE     FLASH_PageErase
#define EEPROM_Read             NV_Read
#define EEPROM_Write(_A,_B,_C)  NV_Write(_A,_B,_C); NV_Sync()

/* Definitions for Multiprotocol */
#define delayMilliseconds(x) BOARD_DelayMs(x)
#define millis      BOARD_GetTick
#define IS_LED_on   (LED_PORT->IDR & (1<<LED_PIN))
#define LED_off     LED_OFF
#define LED_toggle  LED_TOGGLE
#define EE_ADDR     uint16_t

#define cli     __disable_irq
#define sei     __enable_irq

#define MOCK_PPM

/* Function prototyes */
void BOARD_Init(void);
void BOARD_DelayMs(uint32_t ms);
uint32_t BOARD_GetTick(void);
void BOARD_GPIO_Init(GPIO_TypeDef *port, uint8_t pin, uint8_t mode);
void SPI_Write(uint8_t data);
uint8_t SPI_Read(void);

uint32_t flash_write(uint8_t *dst, uint8_t *data, uint16_t count);
void FLASH_PageErase(uint32_t PageAddress);

#ifdef ENABLE_USART
void usart_init(void);
#endif

#ifdef __cplusplus
}
#endif

#endif