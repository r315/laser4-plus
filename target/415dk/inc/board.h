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
#include "spi.h"
#include "gpio.h"
#include "stdinout.h"
#include "tone.h"

#define SET_BIT(REG, BIT)       ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)     ((REG) &= ~(BIT))

#define GPO_SET(_IO, _PIN)      _IO->BSRE = (1 << _PIN)
#define GPO_CLR(_IO, _PIN)      _IO->BRE = (1 << _PIN)
#define GPO_TOGGLE(_IO, _PIN)   _IO->ODR ^= (1<<_PIN)

/* General symbols */
#define RAM_CODE                __attribute__((section(".ram_code")))
#define millis                  GetTick
#define cli                     __disable_irq
#define sei                     __enable_irq

#define LED_PIN                 PA_8
#define LED_INIT                GPIO_Config(LED_PIN, GPO_LS);

#define LED_OFF                 GPIO_Write(LED_PIN, GPIO_PIN_HIGH)
#define LED_ON                  GPIO_Write(LED_PIN, GPIO_PIN_LOW)
#define LED_TOGGLE              GPIO_Toggle(LED_PIN)
#define IS_LED_on               (GPIOA->IPTDT & (1<<8))


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
#define CC25_CS_INIT            GPIO_Config(PB_12, GPO_MS); CC25_CS_FALSE
#define CC25_CS_FALSE           GPO_SET(CC25_CS_PORT, CC25_CS_PIN)
#define CC25_CS_TRUE            GPO_CLR(CC25_CS_PORT, CC25_CS_PIN)
#define RFX240_TXEN             CC2500_00_IOCFG2
#define RFX240_RXEN             CC2500_02_IOCFG0
#define HW_CC2500_MODULE_RESET
//Output AF_PP, IN no pull
#define SPI_PINS_INIT           GPIOB->CRH = (GPIOB->CRH & ~(0xFFF << 20)) | (0xB4B << 20);


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

void DelayMs(uint32_t ms);
uint32_t GetTick(void);
uint32_t ElapsedTicks(uint32_t start_ticks);
void SW_Reset(void);
void __debugbreak(void);

void SPI_Write(uint8_t data);
uint8_t SPI_Read(void);

uint32_t xrand(void);
uint32_t cpuGetId(void);

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
//uint32_t batteryReadVI(vires_t *dst);
#endif

#ifdef ENABLE_PPM_OUTPUT
void ppmOut(uint16_t *data);
#endif

#ifdef ENABLE_AUX_ENCODER
int16_t auxGetEncoder(void);
#endif

#ifdef ENABLE_AUX_SWITCHES
uint32_t auxGetSwitches(void);
#endif

#ifdef ENABLE_BUZZER
void buzPlayTone(uint16_t freq, uint16_t duration);
void buzPlay(tone_t *tones);
uint16_t buzSetLevel(uint16_t level);
void buzWaitEnd(void);
#endif

#ifdef ENABLE_UART
extern stdinout_t pcom;
#endif

#ifdef __cplusplus
}
#endif

#endif