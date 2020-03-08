#ifndef _board_h_
#define _board_h_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "fifo.h"

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
#define GPI_PU                  (6 << 2) // 2 | 4               

#define GPIO_ENABLE             RCC->APB2ENR |=      \
                                RCC_APB2ENR_IOPCEN \
                                | RCC_APB2ENR_IOPBEN \
                                | RCC_APB2ENR_IOPAEN \
                                | RCC_APB2ENR_AFIOEN;

#define GPO_INIT(_IO, _PIN)     gpioInit(_IO, _PIN, GPO_2MHZ)
#define GPO_SET(_IO, _PIN)      _IO->BSRR = (1 << _PIN)
#define GPO_CLR(_IO, _PIN)      _IO->BRR = (1 << _PIN)
#define GPO_TOGGLE(_IO, _PIN)   _IO->ODR ^= (1<<_PIN)

#if 0
#define LED_PORT                GPIOB
#define LED_PIN                 GPIO_PIN_3
#define LED_INIT                GPIOB->CRL = (GPIOB->CRL & ~(15<<12)) | (2<<12) // assume swd is already enabled
#define LED_SET                 GPIO_PIN_RESET
#define LED_RESET               GPIO_PIN_SET
#else// Blue pill LED
#define LED_PORT                GPIOC
#define LED_PIN                 13
#define LED_ON                  GPO_SET(LED_PORT, LED_PIN)
#define LED_OFF                 GPO_CLR(LED_PORT, LED_PIN)
#define LED_INIT                GPO_INIT(LED_PORT, LED_PIN); LED_OFF
#define LED_TOGGLE              GPO_TOGGLE(LED_PORT, LED_PIN)
#endif

#define DBG_PIN                 10
#define DBG_PORT                GPIOB
#define DBG_PIN_HIGH            GPO_SET(DBG_PORT, DBG_PIN)
#define DBG_PIN_LOW             GPO_CLR(DBG_PORT, DBG_PIN)
#define DBG_PIN_INIT            GPO_INIT(DBG_PORT, DBG_PIN); DBG_PIN_LOW
#define DBG_PIN_TOGGLE          GPO_TOGGLE(DBG_PORT, DBG_PIN)

/* CC250 Chip select PB6 */
#define CC25_CS_PIN             12
#define CC25_CS_PORT            GPIOB
#define CC25_CS_INIT            GPO_INIT(CC25_CS_PORT, CC25_CS_PIN); CC25_CS_FALSE
#define CC25_CS_FALSE           GPO_SET(CC25_CS_PORT, CC25_CS_PIN)
#define CC25_CS_TRUE            GPO_CLR(CC25_CS_PORT, CC25_CS_PIN)
#define HW_CC2500_MODULE_RESET


/** 
 * SPI2
 * 
 * PB13 (SCK)
 * PB14 (MISO)
 * PB15 (MOSI)
 * 
 * */

//Output AF_PP, IN no pull
#define SPI_PINS_INIT           GPIOB->CRH = (GPIOB->CRH & ~(0xFFF << 20)) | (0xB4B << 20); 

//Main Clock Output, requires prior MCO bit in RCC_CFG
#define MCO_EN                  GPIOA->CRH = (GPIOA->CRH & ~(15<<0)) | (11 << 0); \
                                RCC->APB2ENR |= (1 << 0)

/**
 * Switches 
 * AUX1 -> PC14
 * AUX2 -> PC15
 * AUX3 -> PB4
 * */
#define HW_SW_AUX1_PIN          14
#define HW_SW_AUX2_PIN          15
#define HW_SW_AUX3_PIN          4
#define HW_SW_AUX1_PORT         GPIOC
#define HW_SW_AUX2_PORT         GPIOC
#define HW_SW_AUX3_PORT         GPIOB
#define HW_SW_INIT              gpioInit(HW_SW_AUX1_PORT, HW_SW_AUX1_PIN, GPI_PU); \
                                gpioInit(HW_SW_AUX2_PORT, HW_SW_AUX2_PIN, GPI_PU); \
                                gpioInit(HW_SW_AUX3_PORT, HW_SW_AUX3_PIN, GPI_PU); \

#define HW_SW_READ              readSwitches()
#define HW_SW_AUX1_VAL          ((HW_SW_AUX1_PORT->IDR & (1 << HW_SW_AUX1_PIN)) == 0)
#define HW_SW_AUX2_VAL          ((HW_SW_AUX2_PORT->IDR & (1 << HW_SW_AUX2_PIN)) == 0)
#define HW_SW_AUX3_VAL          ((HW_SW_AUX3_PORT->IDR & (1 << HW_SW_AUX3_PIN)) == 0)
#define IS_HW_SW_AUX1_PRESSED   (HW_SW_AUX1_PORT->IDR & (1 << HW_SW_AUX1_PIN)) == 0
#define IS_HW_SW_AUX3_PRESSED   (HW_SW_AUX3_PORT->IDR & (1 << HW_SW_AUX3_PIN)) == 0
#define IS_BIND_BUTTON_PRESSED  IS_HW_SW_AUX1_PRESSED

/** RF enable for 35MHz transmiter */
#define TX35_MHZ_INSTALLED
#define HW_TX_35MHZ_EN_PIN      13
#define HW_TX_35MHZ_EN_PORT     GPIOC
#define HW_TX_35MHZ_EN_INIT     gpioInit(HW_TX_35MHZ_EN_PORT, HW_TX_35MHZ_EN_PIN, GPO_2MHZ); HW_TX_35MHZ_OFF
#define HW_TX_35MHZ_ON          GPO_SET(HW_TX_35MHZ_EN_PORT, HW_TX_35MHZ_EN_PIN)
#define HW_TX_35MHZ_OFF         GPO_CLR(HW_TX_35MHZ_EN_PORT, HW_TX_35MHZ_EN_PIN)

/* PPM input pin PB5 */
#define HW_PPM_INPUT_PIN        5
#define HW_PPM_INPUT_PORT       GPIOB
#define millis                  getTick
#define IS_LED_on               (LED_PORT->IDR & (1<<LED_PIN))
#define LED_off                 LED_OFF
#define LED_toggle              LED_TOGGLE
#define EE_ADDR                 uint16_t

#define BUZ_TIM                 TIM1
#define BUZ_DEFAULT_VOLUME      9     // 10us pulse.
#define FREQ_TO_US(_F)          (1000000/_F)

#define ENC_TIM_IRQn            TIM2_IRQn
#define ENC_TIM                 TIM2
#define ENC_TIM_IRQHandler      TIM2_IRQHandler

#define TIMER_BASE              TIM3          //PB5 -> TIM3_CH2
#define TIMER_BASE_IRQn         TIM3_IRQn
#define TIMER_BASE_IRQHandler   TIM3_IRQHandler

#define PPM_TIM                 TIM4

#define cli                     __disable_irq
#define sei                     __enable_irq

#ifdef TX35_MHZ_INSTALLED
#define HW_PROTOCOL_SWITCH      (IS_HW_SW_AUX3_PRESSED)? 14 : 10      // 1...14
#else
#define HW_PROTOCOL_SWITCH      10      // 1...14
#endif
#define HW_BANK_SWITCH          0       //bank_switch();                           

#if defined(ENABLE_PPM)
#define MIN_PPM_CHANNELS        4
#define MAX_PPM_CHANNELS        6
#define PPM_MAX_PERIOD          4400 // 2200
#define PPM_MIN_PERIOD          1600 // 800
#define PPM_MED_PERIOD          (PPM_MAX_PERIOD - PPM_MIN_PERIOD)

#define PPM_PULSE_WIDTH         600     // 300us
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
#define THROTTLE_OFFSET         200
#define PWM_MAX_PULSE           3000
#define PWM_MIN_PULSE           1000
#define PWM_CENTER_PULSE        ((PWM_MAX_PULSE - PWM_MIN_PULSE)/2)
#endif

/* Analog input */
#define HW_VBAT_CHANNEL         0
#define HW_ISENSE_CHANNEL       2
#define HW_VBAT_CH_INIT         gpioInit(GPIOA, HW_VBAT_CHANNEL, GPI_ANALOG); \
                                gpioInit(GPIOA, HW_ISENSE_CHANNEL, GPI_ANALOG)
#define HW_VREFINT_CHANNEL      17
#define VREFINT_VALUE           1200.0f

/* fast code */
#define RAM_CODE                __attribute__((section(".ram_code")))

/* Symbols for NVDATA */
#define NVDATA_SECTOR_INIT      
#define NVDATA_SECTOR_START     &_seeprom
#define NVDATA_SECTOR_END       &_eeeprom
#define NVDATA_SECTOR_READ      memcpy
#define NVDATA_SECTOR_WRITE     flashWrite
#define NVDATA_SECTOR_ERASE     flashPageErase
#define EEPROM_Read             NV_Read
#define EEPROM_Write(_A,_B,_C)  NV_Write(_A,_B,_C)
#define EEPROM_Sync             NV_Sync
#define EEPROM_SIZE             30

/* General symbols */
#define ADC_RDY                 (1 << 0)
#define ADC_DIV                 (1 << 1)
#define ADC_CAL                 (1 << 2)
#define ADC_RES                 (1 << 3)
#define ADC_CR2_EXTSEL_SWSTART  (15 << 17)
#define ADC_SEQ_LEN             2

#define BUZ_PLAYING             (1 << 0)

#define SWTIM_NUM               4
#define SWTIM_RUNNING           (1 << 0)
#define SWTIM_AUTO              (1 << 1)
#define SWTIM_IN_USE            (1 << 2)


typedef struct tone{
  uint16_t f;
  uint16_t t;
}tone_t;

/* Public variables */
extern uint32_t SystemCoreClock;
extern uint32_t _seeprom, _eeeprom;     //declared on linker script

#ifdef ENABLE_SERIAL_FIFOS
extern fifo_t serial_tx_fifo;
extern fifo_t serial_rx_fifo;
#endif

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

float adcGetResolution(void);
void adcSetVdivRacio(float r);
float adcGetVdivRacio(void);
uint32_t adcCalibrate(void);
float getInstantCurrent(void);
uint32_t batteryGetVoltage(void);
uint32_t batteryReadVoltage(uint32_t *dst);
uint32_t batteryGetCurrent(void);

void ppmOut(uint16_t *data);

void buzPlayTone(uint16_t freq, uint16_t duration);
void buzPlay(tone_t *tones);
void buzSetLevel(uint16_t level);
void buzWaitEnd(void);

uint32_t xrand(void);

void processTimers(void);
uint32_t startTimer(uint32_t time, uint32_t flags, void (*cb)(void));
void stopTimer(uint32_t tim);

#ifdef ENABLE_USART
void usart_init(void);
#endif

#ifdef __cplusplus
}
#endif

#endif