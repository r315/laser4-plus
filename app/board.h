#ifndef _board_h_
#define _board_h_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f1xx_hal.h"

/* GPIO definitions */
#define GPIO_MODE_MASK          15
//#define GPIO_MODE_INPUT         (0 << 0) // Input, defined on HAL
#define GPIO_MODE_O2MHZ         (2 << 0)
#define GPIO_MODE_O10MHZ        (1 << 0)
#define GPIO_MODE_O50MHZ        (3 << 0)
#define GPIO_CNF_OD             (1 << 2)     // OUPUT: Open drain, INPUT: Floating
#define GPIO_CNF_AF             (2 << 2)     // OUTPUT: Alternative function, INPUT: pull-up/pull-down

/* Default push-pull */
#define GPO_2MHZ                GPIO_MODE_O2MHZ 
#define GPO_10MHZ               GPIO_MODE_O10MHZ
#define GPO_50MHZ               GPIO_MODE_O50MHZ
#define GPO_OD                  GPIO_CNF_OD
#define GPO_AF                  (2 << 2)
#define GPI_ANALOG              0
#define GPI_OD                  (1 << 2) // floating
#define GPI_PD                  (2 << 2)
#define GPI_PU                  (6 << 2)                

#define GPIO_ENABLE RCC->APB2ENR |=      \
                      RCC_APB2ENR_IOPCEN \
                    | RCC_APB2ENR_IOPBEN \
                    | RCC_APB2ENR_IOPAEN \
                    | RCC_APB2ENR_AFIOEN;

#define GPO_INIT(_IO, _PIN)     gpioInit(_IO, _PIN, GPO_2MHZ)
#define GPO_SET(_IO, _PIN)      _IO->BSRR = (1 << _PIN)
#define GPO_CLR(_IO, _PIN)      _IO->BRR = (1 << _PIN)
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
#define LED_OFF     GPO_CLR(LED_PORT, LED_PIN)
#define LED_INIT    GPO_INIT(LED_PORT, LED_PIN); LED_OFF
#define LED_TOGGLE  GPO_TOGGLE(LED_PORT, LED_PIN)
#endif

#define DBG_PIN         10
#define DBG_PORT        GPIOB
#define DBG_PIN_ON      GPO_SET(DBG_PORT, DBG_PIN)
#define DBG_PIN_OFF     GPO_CLR(DBG_PORT, DBG_PIN)
#define DBG_PIN_INIT    GPO_INIT(DBG_PORT, DBG_PIN); DBG_PIN_OFF
#define DBG_PIN_TOGGLE  GPO_TOGGLE(DBG_PORT, DBG_PIN)

/* CC250 Chip select PB6 */
#define CC25_CS_PIN     6
#define CC25_CS_PORT    GPIOB
#define CC25_CS_INIT    GPO_INIT(CC25_CS_PORT, CC25_CS_PIN); CC25_CS_FALSE
#define CC25_CS_FALSE   GPO_SET(CC25_CS_PORT, CC25_CS_PIN)
#define CC25_CS_TRUE    GPO_CLR(CC25_CS_PORT, CC25_CS_PIN)
#define HW_CC2500_MODULE_RESET


/** 
 * SPI2
 * 
 * PB13 (SCK)
 * PB14 (MISO)
 * PB15 (MOSI)
 * 
 * */
#define SPI_PINS_INIT GPIOB->CRH = (GPIOB->CRH & ~(0xFFF << 20)) | (0xB4B << 20); //Output AF_PP, IN no pull

//Main Clock Output, requires prior MCO bit in RCC_CFG
#define MCO_EN GPIOA->CRH = (GPIOA->CRH & ~(15<<0)) | (11 << 0); \
               RCC->APB2ENR |= (1 << 0)

/* Definitions for Multiprotocol */
/* Button pin PB4 */
#define HW_BIND_BUTTON_PIN        4
#define HW_BIND_BUTTON_INIT       gpioInit(GPIOB, HW_BIND_BUTTON_PIN, GPI_PU);
#define IS_HW_BIND_BUTTON_PRESSED (GPIOB->IDR & (1 << HW_BIND_BUTTON_PIN)) == 0
/**
 * Switches 
 * PA4 AUX1
 * PA5 AUX2
 * PA6 AUX3
 * PA7 AUX4
 * */
#define HW_READ_SWITCHES          readSwitches()
#define HW_SW_AUX1                (1 << 0)
#define HW_SW_AUX2                (1 << 1)
#define HW_SW_AUX3                (1 << 2)
#define HW_SW_AUX4                (1 << 3)
#define HW_SW_MASK                (HW_SW_AUX1 | HW_SW_AUX2 | HW_SW_AUX3 | HW_SW_AUX4)
#define HW_SW_PORT                ~(GPIOA->IDR >> 4)
#define HW_SW_INIT                GPIOA->CRL = (GPIOB->CRL & ~(0xFFFF << 16)) | (0x8888 << 16); \
                                  GPIOA->BSRR =  HW_SW_MASK << 4;   /* Input pull-down */
/** RF enable for 35MHz transmiter */
#define HW_TX_35MHZ_EN_PIN        2
#define HW_TX_35MHZ_EN_PORT       GPIOA
#define HW_TX_35MHZ_EN_INIT       gpioInit(HW_TX_35MHZ_EN_PORT, HW_TX_35MHZ_EN_PIN, GPO_2MHZ); HW_TX_35MHZ_OFF
#define HW_TX_35MHZ_ON            GPO_SET(HW_TX_35MHZ_EN_PORT, HW_TX_35MHZ_EN_PIN)
#define HW_TX_35MHZ_OFF           GPO_CLR(HW_TX_35MHZ_EN_PORT, HW_TX_35MHZ_EN_PIN)

/* PPM input pin PB5 */
#define HW_PPM_INPUT_PIN          5
#define HW_PPM_INPUT_PORT         GPIOB
#define millis                    getTick
#define IS_LED_on                 (LED_PORT->IDR & (1<<LED_PIN))
#define LED_off                   LED_OFF
#define LED_toggle                LED_TOGGLE
#define EE_ADDR                   uint16_t

#define PPM_TIM_IRQn              TIM3_IRQn
#define PPM_TIM                   TIM3          //PB5 -> TIM3_CH2
#define PPM_TIM_IRQHandler        TIM3_IRQHandler
#define TIMER_BASE                PPM_TIM
#define DEMO_TIM_IRQn             TIM2_IRQn
#define DEMO_TIM                  TIM2
#define DEMO_TIM_IRQHandler       TIM2_IRQHandler

#define cli     __disable_irq
#define sei     __enable_irq

#define HW_PROTOCOL_SWITCH  10      // 1...14
#define HW_BANK_SWITCH      0       //bank_switch(); 

#define MOCK_PPM
                                  
#define MIN_RADIO_CHANNELS      4

#if defined(ENABLE_PPM)
#define PPM_MAX_CHANNELS        8
#define PPM_MAX_PULSE           2100
#define PPM_MIN_PULSE           900
#define PPM_CENTER_PULSE      ((PPM_MAX_PULSE - PPM_MIN_PULSE)/2)
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

/* Analog input */
#define ADC_VREF              1500 // mV
#define HW_VBAT_CHANNEL       0

/* fast code */
#define RAM_CODE __attribute__((section(".ram_code")))

/* Symbols for NVDATA */
extern uint32_t _seeprom, _eeeprom;     //declared on linker script
#define NVDATA_SECTOR_INIT      
#define NVDATA_SECTOR_START     &_seeprom
#define NVDATA_SECTOR_END       &_eeeprom
#define NVDATA_SECTOR_READ      memcpy
#define NVDATA_SECTOR_WRITE     flashWrite
#define NVDATA_SECTOR_ERASE     flashPageErase
#define EEPROM_Read             NV_Read
#define EEPROM_Write(_A,_B,_C)  NV_Write(_A,_B,_C)
#define EEPROM_Sync             NV_Sync


/* Function prototyes */
void delayMs(uint32_t ms);
uint32_t getTick(void);
void SPI_Write(uint8_t data);
uint8_t SPI_Read(void);
void gpioInit(GPIO_TypeDef *port, uint8_t pin, uint8_t mode);
void gpioAttachInterrupt(GPIO_TypeDef *port, uint8_t pin, uint8_t edge, void(*)(void));
void gpioRemoveInterrupt(GPIO_TypeDef *port, uint8_t pin);

uint32_t flashWrite(uint8_t *dst, uint8_t *data, uint16_t count);
uint32_t flashPageErase(uint32_t PageAddress);
void FLASH_PageErase(uint32_t PageAddress);       // HAL Function

void enableWatchDog(uint32_t interval);
void reloadWatchDog(void);

void laser4Init(void);

uint32_t readSwitches(void);
uint32_t readBatteryVoltage(void);

#ifdef ENABLE_USART
void usart_init(void);
#endif

#ifdef __cplusplus
}
#endif

#endif