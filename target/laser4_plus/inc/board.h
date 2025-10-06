#ifndef _board_h_
#define _board_h_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f1xx.h"
#include "stdinout.h"
#include "tone.h"
#include "nvdata.h"

#ifdef ENABLE_DISPLAY
#include "ssd1306.h"
#include "font.h"
#endif

#define GPIO_ENABLE             RCC->APB2ENR |= \
                                RCC_APB2ENR_IOPAEN |\
                                RCC_APB2ENR_IOPBEN |\
                                RCC_APB2ENR_IOPCEN |\
                                RCC_APB2ENR_AFIOEN;

#define GPO_INIT(_IO, _PIN)     gpioInit(_IO, _PIN, GPO_LS)
#define GPO_SET(_IO, _PIN)      _IO->BSRR = (1 << _PIN)
#define GPO_CLR(_IO, _PIN)      _IO->BRR = (1 << _PIN)
#define GPO_TOGGLE(_IO, _PIN)   _IO->ODR ^= (1<<_PIN)

/**
 * LED's
 * */
#ifdef BOARD_LASER4PLUS
#define LED_PORT                GPIOC /* Actually laser4+ board does not have any led */
#define LED_PIN                 13
#define LED_INIT                GPO_INIT(LED_PORT, LED_PIN); LED_OFF;
#define LED_ON                  GPO_CLR(LED_PORT, LED_PIN)
#define LED_OFF                 GPO_SET(LED_PORT, LED_PIN)
#define LED_TOGGLE              GPO_TOGGLE(LED_PORT, LED_PIN)
#define DBG_PIN_INIT
#define DBG_PIN_HIGH
#define DBG_PIN_LOW
#define DBG_PIN2_TOGGLE
#elif defined(BOARD_BLUEPILL)
#define LED_PORT                GPIOC   // Blue pill LED
#define LED_PIN                 13
#define LED_ON                  GPO_SET(LED_PORT, LED_PIN)
#define LED_OFF                 GPO_CLR(LED_PORT, LED_PIN)
#define LED_INIT                GPO_INIT(LED_PORT, LED_PIN); LED_OFF; DBG_PIN_INIT; DBG_PIN_HIGH
#define LED_TOGGLE              GPO_TOGGLE(LED_PORT, LED_PIN)

#define DBG_PIN                 9
#define DBG_PORT                GPIOB
#define DBG_PIN_HIGH            GPO_SET(DBG_PORT, DBG_PIN)
#define DBG_PIN_LOW             GPO_CLR(DBG_PORT, DBG_PIN)
#define DBG_PIN_TOGGLE          GPO_TOGGLE(DBG_PORT, DBG_PIN)

#define DBG_PIN_INIT            GPO_INIT(DBG_PORT, DBG_PIN); DBG_PIN_LOW; \
                                GPO_INIT(DBG_PIN2_PORT, DBG_PIN2); \

#define DBG_PIN2                8
#define DBG_PIN2_PORT           GPIOB
#define DBG_PIN2_HIGH           GPO_SET(DBG_PIN2_PORT, DBG_PIN2)
#define DBG_PIN2_LOW            GPO_CLR(DBG_PIN2_PORT, DBG_PIN2)
#define DBG_PIN2_TOGGLE         GPO_TOGGLE(DBG_PIN2_PORT, DBG_PIN2)
#else
#error "No board defined"
#endif

#define IS_LED_on               (LED_PORT->IDR & (1<<LED_PIN))


/**
 * SPI2
 * PB12 -> CS
 * PB13 -> SCK
 * PB14 <- MISO
 * PB15 -> MOSI
 * */
#define CC25_CS_PIN             12
#define CC25_CS_PORT            GPIOB
#define CC25_CS_INIT            GPO_INIT(CC25_CS_PORT, CC25_CS_PIN); CC25_CS_FALSE
#define CC25_CS_FALSE           GPO_SET(CC25_CS_PORT, CC25_CS_PIN)
#define CC25_CS_TRUE            GPO_CLR(CC25_CS_PORT, CC25_CS_PIN)
#define HW_CC2500_MODULE_RESET
//Output AF_PP, IN no pull
#define SPI_PINS_INIT           GPIOB->CRH = (GPIOB->CRH & ~(0xFFF << 20)) | (0xB4B << 20);

/**
 * Switches
 * AUX1 <- PC14
 * AUX2 <- PC15
 * AUX3 <- PB4 (encoder press)
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

#define HW_SW_AUX1_VAL          (HW_SW_AUX1_PORT->IDR & (1 << HW_SW_AUX1_PIN))
#define HW_SW_AUX2_VAL          (HW_SW_AUX2_PORT->IDR & (1 << HW_SW_AUX2_PIN))
#define HW_SW_AUX3_VAL          (HW_SW_AUX3_PORT->IDR & (1 << HW_SW_AUX3_PIN))
#define IS_HW_SW_AUX1_PRESSED   !HW_SW_AUX1_VAL
#define IS_HW_SW_AUX2_PRESSED   !HW_SW_AUX2_VAL
#define IS_HW_SW_AUX3_PRESSED   !HW_SW_AUX3_VAL
#define IS_BIND_BUTTON_PRESSED  IS_HW_SW_AUX1_PRESSED

/** RF enable for 35MHz transmiter */
#define HW_TX_35MHZ_EN_PIN      3
#define HW_TX_35MHZ_EN_PORT     GPIOA
#define HW_TX_35MHZ_EN_INIT     gpioInit(HW_TX_35MHZ_EN_PORT, HW_TX_35MHZ_EN_PIN, GPO_LS); HW_TX_35MHZ_OFF
#define HW_TX_35MHZ_ON          GPO_SET(HW_TX_35MHZ_EN_PORT, HW_TX_35MHZ_EN_PIN)
#define HW_TX_35MHZ_OFF         GPO_CLR(HW_TX_35MHZ_EN_PORT, HW_TX_35MHZ_EN_PIN)

/**
 * PPM
 * PB5 <- input
 * PB7 -> output
 */
#define HW_PPM_INPUT_PIN        5
#define HW_PPM_INPUT_PORT       GPIOB
#define HW_PPM_INPUT            HW_PPM_INPUT_PORT,HW_PPM_INPUT_PIN
#define PPM_TIM                 TIM4
#define PPM_TIM_CH              (2 - 1)
#define PPM_DMA_REQ             DMA1_REQ_TIM4_UP
#define TIME_BASE               TIM3

#define PPM_CH_IN_NUM           4
#define PPM_CH_OUT_NUM          6
// PPM values are board specific [us]
#ifdef BOARD_LASER4PLUS
#define PPM_MAX_PERIOD          2012
#define PPM_MIN_PERIOD          988
#define PPM_PULSE_WIDTH         300
#define SWITCH_ACTIVE           1750
#define SWITCH_INACTIVE         1250
#else
#define PPM_MAX_PERIOD          2100
#define PPM_MIN_PERIOD          900
#define PPM_PULSE_WIDTH         300
#define SWITCH_ACTIVE           1750
#define SWITCH_INACTIVE         1250
#endif

/**
 * Buzzer
 * PA8 -> BUZ
 */
#define BUZ_TIM                 TIM1
#define BUZ_TIM_CH              (1 - 1)
#define BUZ_PIN                 PA_8
#define BUZ_PIN_IDLE            1
#define BUZ_DEFAULT_VOLUME      9     // 10us pulse.

/**
 * Rotary encoder
 * PB3  <- A
 * PA15 <- B
 */
#define ENC_TIM_IRQn            TIM2_IRQn
#define ENC_TIM                 TIM2
#define ENC_TIM_IRQHandler      TIM2_IRQHandler

#define HW_BANK_SWITCH          0       /* bank_switch(); */

#ifdef ENABLE_PWM
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

/* General symbols */
#define RAM_CODE                __attribute__((section(".ram_code")))
#define cli                     __disable_irq
#define sei                     __enable_irq

/**
 * Optional Main Clock Output on PA8
 * Requires prior MCO bit in RCC_CFG
 */
#define MCO_EN                  GPIOA->CRH = (GPIOA->CRH & ~(15<<0)) | (11 << 0); \
                                RCC->APB2ENR |= (1 << 0)

#define ADC_RDY                 (1 << 0)
#define ADC_DIV                 (1 << 1)
#define ADC_CAL                 (1 << 2)
#define ADC_RES                 (1 << 3)
#define ADC_CR2_EXTSEL_SWSTART  (15 << 17)
#define ADC_SEQ_LEN             2


#define ISENSE_GAIN             40
#define BATTERY_VOLTAGE_MIN     3100U
#define BATTERY_VOLTAGE_MAX     4200U
#define BATTERY_CURRENT_MAX     600U

#define BUZ_PLAYING             (1 << 0)

/* This is not eeprom size, but it should be > than requested eeprom size and be 4byte aligned */
#define L4P_EEPROM_SZ           128
#define EEPROM_Read             NV_Read
#define EEPROM_Write            NV_Write
#define EEPROM_Sync             NV_Sync
#define EEPROM_Erase            NV_Erase

uint8_t* EEPROM_Init(uint16_t size);

typedef struct vires {
    uint32_t vbat;
    uint32_t cur;
}vires_t;

/**
 * Functions implemented by board file
 */

/* Percision timer with 0.5us resolution */
uint16_t ticksGet(void);
uint16_t ticksGetElapsed(uint16_t start);
void ticksSetInterval(uint16_t interval);
void ticksResetInterval(void);
int16_t ticksGetIntervalRemaining(void);
uint16_t ticksIsIntervalTimedout(void);

/* Generic delay function */
void DelayMs(uint32_t ms);
uint32_t millis(void);

/* SPI API */
void SPI_Write(uint8_t data);
uint8_t SPI_Read(void);
/* Gpio API */
void gpioInit(GPIO_TypeDef *port, uint8_t pin, uint8_t mode);
void gpioAttachInterrupt(GPIO_TypeDef *port, uint8_t pin, uint8_t edge, void(*)(void));
void gpioRemoveInterrupt(GPIO_TypeDef *port, uint8_t pin);
/* Random number generator and cpu unique id */
uint32_t xrand(void);
uint32_t cpuGetId(void);

/**
 * Optional API
 */
#ifdef ENABLE_BATTERY_MONITOR
float adcGetResolution(void);
void adcSetVdivRacio(float r);
float adcGetVdivRacio(void);
void adcSetSenseResistor(float rs);
float adcGetSenseResistor(void);
uint32_t adcCalibrate(void);
uint32_t batteryGetVoltage(void);
// uint32_t batteryReadVoltage(uint32_t *dst); remove when confirmed not necessary
uint32_t batteryGetCurrent(void);
// uint32_t batteryReadCurrent(uint32_t *dst);
uint32_t batteryReadVI(vires_t *dst);
#endif

#ifdef ENABLE_PPM_OUTPUT
void ppmOut(const uint16_t *data, uint8_t nch);
#endif

#ifdef ENABLE_BUZZER
void buzPlayTone(uint16_t freq, uint16_t duration);
void buzPlay(tone_t *tones);
uint16_t buzSetLevel(uint16_t level);
void buzWaitEnd(void);
#endif

#ifdef ENABLE_AUX_ENCODER
int16_t auxGetEncoder(void);
#endif

#ifdef ENABLE_AUX_SWITCHES
uint32_t auxGetSwitches(void);
#endif

#ifdef ENABLE_UART
int serial_available(void);
int serial_read(char *buf, int len);
int serial_write(const char *buf, int len);
#endif

#ifdef ENABLE_VCP
extern stdinout_t vcp;
#endif

#ifdef ENABLE_DISPLAY
#define DISPLAY_W       128
#define DISPLAY_H       32
uint32_t displayInit(void);
#endif

#ifdef __cplusplus
}
#endif

#endif