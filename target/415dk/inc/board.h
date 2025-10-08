#ifndef _board_h_
#define _board_h_


#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include "at32f4xx.h"
#include "gpio_at32f4xx.h"
#include "tone_at32f4xx.h"
#include "dma_at32f4xx.h"
#include "spi.h"
#include "gpio.h"
#include "stdinout.h"
#include "tone.h"
#include "nvdata.h"

#ifdef ENABLE_DISPLAY
#include "ssd1306.h"
#include "font.h"
#endif

#define SET_BIT(REG, BIT)       ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)     ((REG) &= ~(BIT))

#define GPO_SET(_IO, _PIN)      _IO->BSRE = (1 << _PIN)
#define GPO_CLR(_IO, _PIN)      _IO->BRE = (1 << _PIN)
#define GPO_TOGGLE(_IO, _PIN)   _IO->ODR ^= (1<<_PIN)

/* General symbols */
#define RAM_CODE                __attribute__((section(".ram_code")))
#define cli                     __disable_irq
#define sei                     __enable_irq

#define LED_PIN                 PB_8
#define LED_INIT                GPIO_Config(LED_PIN, GPO_LS);

#define LED_OFF                 GPIO_Write(LED_PIN, GPIO_PIN_HIGH)
#define LED_ON                  GPIO_Write(LED_PIN, GPIO_PIN_LOW)
#define LED_TOGGLE              GPIO_Toggle(LED_PIN)

#define DBG_PIN                 PB_9
#define DBG_PIN_INIT            GPIO_Config(DBG_PIN, GPO_LS);
#define DBG_PIN_HIGH            GPIO_Write(DBG_PIN, GPIO_PIN_HIGH)
#define DBG_PIN_LOW             GPIO_Write(DBG_PIN, GPIO_PIN_LOW)
#define DBG_PIN_TOGGLE          GPIO_Toggle(DBG_PIN)

#define IS_LED_on               (GPIOA->IPTDT & (1<<8))

/**
 * RF enable for 35MHz transmiter
 * */
#define HW_TX_35MHZ_EN_PIN      PA_3
#define HW_TX_35MHZ_EN_INIT     GPIO_Config(HW_TX_35MHZ_EN_PIN, GPO_LS); HW_TX_35MHZ_OFF
#define HW_TX_35MHZ_ON          GPIO_Write(HW_TX_35MHZ_EN_PIN, GPIO_PIN_HIGH)
#define HW_TX_35MHZ_OFF         GPIO_Write(HW_TX_35MHZ_EN_PIN, GPIO_PIN_LOW)

/**
 * CC2500 Module
 *
 * PB12 -> CS
 * PB13 -> SCK
 * PB14 <- MISO
 * PB15 -> MOSI
 * */
#define CC25_CS_PIN             12
#define CC25_CS_PORT            GPIOB
#define CC25_CS_FALSE           GPO_SET(CC25_CS_PORT, CC25_CS_PIN)
#define CC25_CS_TRUE            GPO_CLR(CC25_CS_PORT, CC25_CS_PIN)
#define CC2500_RESET
#define SPI_PINS_INIT           GPIO_Config(PB_12, GPO_MS); CC25_CS_FALSE

/**
 * PPM
 * PB5 <- input
 * PB7 -> output
 */
#define HW_PPM_INPUT            PB_5
#define HW_PPM_OUTPUT           PB_7
#define PPM_CH_IN_NUM           4
#define PPM_CH_OUT_NUM          6

#define PPM_MAX_PERIOD          2100
#define PPM_MIN_PERIOD          900
#define PPM_PULSE_WIDTH         300
#define SWITCH_ACTIVE           1750
#define SWITCH_INACTIVE         1250

/**
 * Switches
 *
 * AUX1 <- PC14
 * AUX2 <- PC15
 * AUX3 <- PB4 (encoder press)
 * */
#define HW_SW_AUX1_PIN          PC_14
#define HW_SW_AUX2_PIN          PC_15
#define HW_SW_AUX3_PIN          PB_4
#define HW_SW_INIT              GPIO_Config(HW_SW_AUX1_PIN, GPI_PU); \
                                GPIO_Config(HW_SW_AUX2_PIN, GPI_PU); \
                                GPIO_Config(HW_SW_AUX3_PIN, GPI_PU); \

#define HW_SW_AUX1_VAL          GPIO_Read(HW_SW_AUX1_PIN)
#define HW_SW_AUX2_VAL          GPIO_Read(HW_SW_AUX2_PIN)
#define HW_SW_AUX3_VAL          GPIO_Read(HW_SW_AUX3_PIN)
#define IS_HW_SW_AUX1_PRESSED   (HW_SW_AUX1_VAL == 0)
#define IS_HW_SW_AUX2_PRESSED   (HW_SW_AUX2_VAL == 0)
#define IS_HW_SW_AUX3_PRESSED   (HW_SW_AUX3_VAL == 0)
#define IS_BIND_BUTTON_PRESSED  IS_HW_SW_AUX1_PRESSED

#define HW_BANK_SWITCH          0       /* bank_switch(); */

/**
 * Buzzer
 * PA8 -> BUZ
 */
#define BUZ_PIN                 PA_8
#define BUZ_PIN_IDLE            1
#define BUZ_DEFAULT_VOLUME      9     // 10us pulse.

/**
 * ADC/Battery monitor
 * PA0 <- Vbat
 * PA1 <- Ibat
 */
#define ISENSE_GAIN             40
#define BATTERY_VOLTAGE_MIN     3100U
#define BATTERY_VOLTAGE_MAX     4200U
#define BATTERY_CURRENT_MAX     600U
#define VREFINT_VALUE           1200.0f

#define HW_VBAT_CHANNEL         0
#define HW_ISENSE_CHANNEL       2
#define HW_VREFINT_CHANNEL      17
#define HW_VBAT_CH_INIT         GPIO_Config(PA_0, GPI_ANALOG); GPIO_Config(PA_1, GPI_ANALOG)

/**
 * EEPROM
 */
/* This is not eeprom size, but it should be greater than requested eeprom size and be 4byte aligned */
#define L4P_EEPROM_SZ           128
#define EEPROM_Read             NV_Read
#define EEPROM_Write            NV_Write
#define EEPROM_Sync             NV_Sync
#define EEPROM_Erase            NV_Erase

/* Timers */
#define TIME_BASE               TMR3
#define PPM_TIM                 TMR4
#define PPM_DMA_REQ             DMA1_REQ_TIM4_UP
#define ENC_TIM                 TMR2
#define BUZ_TIM                 TMR1
#define BUZ_TIM_CH              (1 - 1)

typedef struct vires {
    uint32_t vbat;
    uint32_t cur;
}vires_t;

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
/* EEPROM API */
uint8_t* EEPROM_Init(uint16_t size);
/* Random number generator and cpu unique id */
uint32_t xrand(void);
uint32_t cpuGetId(void);
/* Gpio API */
void gpioInit(uint32_t name, uint8_t mode);
void gpioAttachInterrupt(uint32_t name, uint8_t edge, void(*)(void));
void gpioRemoveInterrupt(uint32_t name);
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
uint32_t batteryGetCurrent(void);
uint32_t batteryReadVI(vires_t *dst);
#endif

#ifdef ENABLE_PPM_OUTPUT
void ppmOut(const uint16_t *data, uint8_t nch);
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

#ifdef ENABLE_DISPLAY
#define DISPLAY_W       128
#define DISPLAY_H       32
uint32_t displayInit(void);
#endif

#ifdef __cplusplus
}
#endif

#endif