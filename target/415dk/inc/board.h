#ifndef _board_h_
#define _board_h_


#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include "at32f4xx.h"
#include "spi.h"
#include "gpio.h"
#include "gpio_at32f4xx.h"
#include "tone_at32f4xx.h"

#define SET_BIT(REG, BIT)       ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)     ((REG) &= ~(BIT))

#define GPO_SET(_IO, _PIN)      _IO->BSRE = (1 << _PIN)
#define GPO_CLR(_IO, _PIN)      _IO->BRE = (1 << _PIN)
#define GPO_TOGGLE(_IO, _PIN)   _IO->ODR ^= (1<<_PIN)

#define LED_PIN         PA_8
#define LED_INIT       GPIO_Config(LED_PIN, GPO_LS);

#define LED_OFF        GPIO_Write(LED_PIN, GPIO_PIN_HIGH)
#define LED_ON         GPIO_Write(LED_PIN, GPIO_PIN_LOW)
#define LED_TOGGLE     GPIO_Toggle(LED_PIN)


/**
 * SPI2
 * PB12 -> CS
 * PB13 -> SCK
 * PB14 <- MISO
 * PB15 -> MOSI
 *
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


void DelayMs(uint32_t ms);
uint32_t GetTick(void);
uint32_t ElapsedTicks(uint32_t start_ticks);
void SW_Reset(void);
void __debugbreak(void);

void SPI_Write(uint8_t data);
uint8_t SPI_Read(void);

uint32_t batteryGetVoltage(void);

#ifdef __cplusplus
}
#endif

#endif